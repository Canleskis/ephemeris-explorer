use crate::{
    load::{SolarSystem, UniqueAsset},
    prediction::{
        integration::{IntegrationState, PEFRL},
        FixedSegments, FixedSegmentsBuilder, Forward, Trajectory, TrajectoryData,
    },
    time::SimulationTime,
    ui::FixedUiSet,
    MainState,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use hifitime::{Duration, Epoch};
use particular::gravity::newtonian::Acceleration;
use particular::prelude::*;
use thousands::Separable;

pub struct EphemeridesDebugPlugin;

impl Plugin for EphemeridesDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            EphemeridesDebugWindow::show
                .after(FixedUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Default, Resource)]
pub struct EphemeridesDebugWindow;

impl EphemeridesDebugWindow {
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        window: Option<Res<Self>>,
        system: UniqueAsset<SolarSystem>,
        sim_time: Res<SimulationTime>,
        query_builder: Query<&FixedSegmentsBuilder<Forward>>,
        query: Query<(Entity, &Name, &Trajectory)>,
        mut errors: Local<Option<bevy::ecs::entity::EntityHashMap<f64>>>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut open = window.is_some();
        egui::Window::new("Ephemerides debug")
            .open(&mut open)
            .show(ctx, |ui| {
                let builder = query_builder.single();
                let dt = builder.integrator().delta();
                let system = system.get().unwrap();
                let start = system.epoch;
                let duration = Duration::from_days(365.0 * 2.0).min(sim_time.end() - start);
                let compute_error = || {
                    #[derive(Clone, Component, Position, Mass)]
                    struct DiscreteData {
                        pub velocity: DVec3,
                        pub position: DVec3,
                        pub mu: f64,
                    }

                    impl IntegrationState for DiscreteData {
                        #[inline]
                        fn velocity(&mut self) -> &mut DVec3 {
                            &mut self.velocity
                        }

                        #[inline]
                        fn position(&mut self) -> &mut DVec3 {
                            &mut self.position
                        }
                    }
                    // We collect the entities in the same order as the builder stores them.
                    let ((query, state), mut discrete): ((Vec<_>, Vec<_>), Vec<_>) = query
                        .iter()
                        .sort::<Entity>()
                        .filter(|(.., trajectory)| trajectory.read().as_any().is::<FixedSegments>())
                        .map(|(entity, name, trajectory)| {
                            (
                                (
                                    (entity, name, trajectory),
                                    DiscreteData {
                                        velocity: system.bodies[&**name].velocity,
                                        position: system.bodies[&**name].position,
                                        mu: system.bodies[&**name].mu,
                                    },
                                ),
                                vec![system.bodies[&**name].position],
                            )
                        })
                        .unzip();
                    let mut integrator = PEFRL::new(start.to_tai_seconds(), dt, state);

                    for _ in 0..(duration.to_seconds() / dt).ceil() as _ {
                        _ = integrator.step(|_, states, accs| {
                            accs.extend(states.brute_force_pairs(Acceleration::checked()))
                        });
                        for (state, data) in std::iter::zip(integrator.state(), &mut discrete) {
                            data.push(state.position);
                        }
                    }

                    query
                        .iter()
                        .zip(discrete)
                        .map(|((entity, _, traj), discrete)| {
                            (
                                *entity,
                                discrete
                                    .iter()
                                    .enumerate()
                                    .map(|(i, discrete_position)| {
                                        let epoch = start + Duration::from_seconds(dt) * i as i64;
                                        let traj_position = traj.position(epoch).unwrap();

                                        discrete_position.distance(traj_position) * 1e3
                                    })
                                    .fold(0.0, f64::max),
                            )
                        })
                        .collect::<bevy::ecs::entity::EntityHashMap<_>>()
                };

                let available_height = ui.available_height();
                egui_extras::TableBuilder::new(ui)
                    .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                    .column(egui_extras::Column::exact(100.0))
                    .column(egui_extras::Column::exact(80.0))
                    .column(egui_extras::Column::exact(80.0))
                    .column(egui_extras::Column::exact(200.0))
                    .column(egui_extras::Column::exact(200.0))
                    .column(egui_extras::Column::exact(120.0))
                    .min_scrolled_height(0.0)
                    .max_scroll_height(available_height)
                    .header(20.0, |mut header| {
                        header.col(|ui| {
                            ui.strong("Body");
                        });
                        header.col(|ui| {
                            ui.strong("Size");
                        });
                        header.col(|ui| {
                            ui.strong("Segments");
                        });
                        header.col(|ui| {
                            ui.strong("Start");
                        });
                        header.col(|ui| {
                            ui.strong("End");
                        });
                        header.col(|ui| {
                            let text = if errors.is_some() {
                                "Interpolation error"
                            } else {
                                "Compute error"
                            };
                            if ui
                                .add(egui::Button::new(text))
                                .on_hover_text(format!(
                                    "Computes interpolation error from {} to {}",
                                    start,
                                    start + duration
                                ))
                                .clicked()
                            {
                                *errors = Some(compute_error());
                            }
                        });
                    })
                    .body(|body| {
                        let queried = query
                            .iter()
                            .sort::<Entity>()
                            .map(|(entity, name, data)| {
                                (
                                    name.as_str(),
                                    data.size(),
                                    data.len(),
                                    data.start(),
                                    data.end(),
                                    errors.as_ref().and_then(|errors| errors.get(&entity)),
                                )
                            })
                            .collect::<Vec<_>>();
                        body.rows(16.0, queried.len() + 1, |mut row| {
                            let (name, size, count, start, end, error) = match row.index() {
                                0 => queried.iter().fold(
                                    ("Total", 0, 0, Epoch::default(), Epoch::default(), None),
                                    |(name, size, count, ..), (_, s, c, ..)| {
                                        (
                                            name,
                                            size + s,
                                            count + c,
                                            sim_time.start(),
                                            sim_time.end(),
                                            None,
                                        )
                                    },
                                ),

                                i => queried[i - 1],
                            };
                            row.col(|ui| {
                                ui.label(name.to_string());
                            });
                            row.col(|ui| {
                                ui.label(
                                    format!("{:.2} MB", (size as f32 / (1024.0 * 1024.0)))
                                        .separate_with_commas(),
                                );
                            });
                            row.col(|ui| {
                                ui.label(count.to_string());
                            });
                            row.col(|ui| {
                                ui.label(format!("{:x}", start));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:x}", end));
                            });
                            row.col(|ui| {
                                ui.label(
                                    error
                                        .map(|error| format!("{:.2} metres", error))
                                        .unwrap_or_else(|| "N/A".to_string()),
                                );
                            });
                        })
                    });
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}
