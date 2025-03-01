use crate::{
    dynamics::{Forward, NBodyPropagator, UniformSpline},
    load::{SolarSystemState, UniqueAsset},
    prediction::{PredictionContext, Trajectory},
    time::SimulationTime,
    ui::WindowsUiSet,
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use ephemeris::{BoundedTrajectory, EvaluateTrajectory, NBodyProblem, NewtonianGravity};
use hifitime::{Duration, Epoch};
use integration::prelude::*;
use thousands::Separable;

pub struct EphemeridesDebugPlugin;

impl Plugin for EphemeridesDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            EphemeridesDebugWindow::show
                .in_set(WindowsUiSet)
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
        system: UniqueAsset<SolarSystemState>,
        sim_time: Res<SimulationTime>,
        query_prediction: Query<&PredictionContext<NBodyPropagator<Forward>>>,
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
                let Some(system) = system.get() else {
                    return;
                };

                let prediction = query_prediction.single();
                let dt = prediction.propagator.delta();
                let start = system.epoch;
                let duration = Duration::from_days(365.0 * 2.0).min(sim_time.end() - start);

                let compute_error = || {
                    let (query, positions, velocities, mus): (Vec<_>, Vec<_>, Vec<_>, Vec<_>) =
                        query
                            .iter()
                            .sort::<Entity>()
                            .filter(|(.., trajectory)| {
                                trajectory.read().as_any().is::<UniformSpline>()
                            })
                            .map(|(entity, name, trajectory)| {
                                (
                                    (entity, name, trajectory),
                                    system.bodies[&**name].position,
                                    system.bodies[&**name].velocity,
                                    system.bodies[&**name].mu,
                                )
                            })
                            .collect();

                    let nbody = NBodyProblem {
                        time: start.to_tai_seconds(),
                        bound: start.to_tai_seconds() + duration.to_seconds(),
                        state: SecondOrderState::new(positions, velocities),
                        ode: NewtonianGravity {
                            gravitational_parameters: mus,
                        },
                    };

                    let method: QuinlanTremaine12 = QuinlanTremaine12::new(dt.to_seconds());
                    let mut integrator = method.integrate(nbody);

                    let mut errors = bevy::ecs::entity::EntityHashMap::<f64>::default();
                    while let Ok((t, state)) = integrator.advance() {
                        let epoch = Epoch::from_tai_seconds(t);
                        for ((entity, _, traj), position) in query.iter().zip(&state.y) {
                            let traj_position = traj.position(epoch).unwrap();

                            let error = position.distance(traj_position) * 1e3;
                            errors
                                .entry(*entity)
                                .and_modify(|e| *e = e.max(error))
                                .or_insert(error);
                        }
                    }

                    errors
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
                                    data.heap_size(),
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
                                ui.label(format!("{start:x}"));
                            });
                            row.col(|ui| {
                                ui.label(format!("{end:x}"));
                            });
                            row.col(|ui| {
                                ui.label(
                                    error
                                        .map(|error| format!("{error:.2} metres"))
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
