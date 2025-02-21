use crate::{
    camera::Followed,
    floating_origin::BigGridBundle,
    hierarchy::OrbitedBy,
    load::{LoadShipEvent, Ship, SystemRoot},
    plot::TrajectoryPlot,
    prediction::{
        ComputePredictionEvent, DiscreteStates, DiscreteStatesBuilder, Mu, StateVector, Trajectory,
        TrajectoryData,
    },
    time::SimulationTime,
    ui::{get_name, precision, show_tree, IdentedInfo, Labelled, WindowsUiSet},
    MainState,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::{Duration, Epoch};
use std::str::FromStr;

pub struct ShipSpawnerPlugin;

impl Plugin for ShipSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                ShipSpawnerWindow::show,
                ShipSpawnerWindow::on_close.run_if(resource_removed::<ShipSpawnerWindow>),
                load_solar_system_state,
            )
                .chain()
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

pub struct ShipFile;

#[derive(Component, Clone)]
pub struct ShipSpawnerData {
    pub start: Epoch,
    pub name: String,
    pub state_vector: StateVector<DVec3>,
    pub reference: Option<Entity>,
    pub preview_duration: Duration,
}

impl ShipSpawnerData {
    pub fn global_state_vector(
        &self,
        at: Epoch,
        query_trajectory: &Query<&Trajectory>,
    ) -> StateVector<DVec3> {
        self.reference
            .and_then(|entity| query_trajectory.get(entity).ok())
            .and_then(|traj| traj.state_vector(at))
            .unwrap_or_default()
            + self.state_vector
    }
}

#[derive(Default, Resource)]
pub struct ShipSpawnerWindow {
    pub valid_preview: bool,
}

impl ShipSpawnerWindow {
    #[expect(clippy::type_complexity)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        sim_time: Res<SimulationTime>,
        window: Option<ResMut<Self>>,
        root: Single<Entity, With<SystemRoot>>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&OrbitedBy>), Without<ShipSpawnerData>>,
        mut query_preview: Query<(
            Entity,
            &mut ShipSpawnerData,
            Option<(&mut Name, &mut DiscreteStatesBuilder, &mut TrajectoryPlot)>,
        )>,
        query_trajectory: Query<&Trajectory>,
        query_context: Query<(Entity, &Trajectory, &Mu)>,
        followed: Res<Followed>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let new_context = || {
            query_context
                .iter()
                .map(|(e, traj, mu)| (e, (traj.clone(), *mu)))
                .collect()
        };

        let Some(mut window) = window else {
            return;
        };

        let Ok((preview, mut data, Some((mut name, mut builder, mut plot)))) =
            query_preview.get_single_mut()
        else {
            let (entity, data) = match query_preview.get_single() {
                Ok((entity, data, _)) => (entity, data.clone()),
                _ => {
                    let data = ShipSpawnerData {
                        name: "Ship".to_string(),
                        start: sim_time.current().floor(Duration::from_seconds(1.0)),
                        state_vector: StateVector::new(
                            DVec3::new(10_000.0, 0.0, 0.0),
                            DVec3::new(0.0, 6.5, 0.0),
                        ),
                        reference: **followed,
                        preview_duration: Duration::from_days(5.0),
                    };
                    (commands.spawn(data.clone()).set_parent(*root).id(), data)
                }
            };

            let radius = 0.01;
            let sv = data.global_state_vector(data.start, &query_trajectory);

            commands.entity(entity).insert((
                Name::new(data.name.to_string()),
                BigGridBundle {
                    transform: Transform::from_translation(sv.position.as_vec3()),
                    ..default()
                },
                Labelled {
                    font: TextFont::from_font_size(12.0),
                    colour: TextColor(bevy::color::palettes::css::FUCHSIA.into()),
                    offset: Vec2::new(0.0, radius) * 1.1,
                    index: 99,
                },
                Trajectory::new(DiscreteStates::new(data.start, sv.velocity, sv.position)),
                DiscreteStatesBuilder::new(
                    entity,
                    data.start,
                    sv.velocity,
                    sv.position,
                    new_context(),
                ),
                TrajectoryPlot {
                    enabled: true,
                    color: bevy::color::palettes::css::FUCHSIA.into(),
                    start: Epoch::from_tai_duration(-Duration::MAX),
                    end: Epoch::from_tai_duration(Duration::MAX),
                    threshold: 0.5,
                    max_points: usize::MAX,
                    source: entity,
                    reference: data.reference,
                },
            ));

            window.valid_preview = false;

            return;
        };

        let mut open = true;
        egui::Window::new("Ship spawner")
            .open(&mut open)
            .show(ctx, |ui| {
                if ui.button("From file").clicked() {
                    commands.dialog().load_multiple_files::<ShipFile>();
                }

                ui.separator();

                ui.horizontal(|ui| {
                    ui.label("At:");
                    let mut start = data.start.to_tai_seconds();
                    if ui
                        .add(
                            egui::DragValue::new(&mut start)
                                .custom_formatter(|value, _| {
                                    Epoch::from_tai_seconds(value).to_string()
                                })
                                .custom_parser(|text| {
                                    Epoch::from_str(text).ok().map(|t| {
                                        t.clamp(sim_time.start(), sim_time.end())
                                            .floor(Duration::from_seconds(1.0))
                                            .to_tai_seconds()
                                    })
                                })
                                .speed(hifitime::Duration::from_hours(1.0).to_seconds()),
                        )
                        .changed()
                    {
                        data.start = Epoch::from_tai_seconds(start).round(precision());
                        window.valid_preview = false;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Name:");
                    if ui.add(egui::TextEdit::singleline(&mut data.name)).changed() {
                        name.set(data.name.to_string());
                    }
                });

                ui.add_space(5.0);

                let name = data
                    .reference
                    .map(|entity| get_name(entity, &query_hierarchy.transmute_lens().query()))
                    .unwrap_or_else(|| "None".to_string());
                egui::ComboBox::from_id_salt("Reference")
                    .wrap_mode(egui::TextWrapMode::Extend)
                    .selected_text(name)
                    .show_ui(ui, |ui| {
                        show_tree(
                            ui,
                            query_hierarchy.get(*root).unwrap(),
                            |&(e, name, _)| (e, name.clone()),
                            |i, _| i != 0,
                            |ui, _, (ref_entity, ref_name, children), _| {
                                ui.horizontal(|ui| {
                                    if ui
                                        .selectable_value(
                                            &mut data.reference,
                                            Some(ref_entity),
                                            ref_name.as_str(),
                                        )
                                        .changed()
                                    {
                                        window.valid_preview = false;
                                        plot.reference = data.reference;
                                    }
                                });

                                children.map(|c| query_hierarchy.iter_many(c))
                            },
                        );
                    });

                ui.add_space(5.0);

                egui::Grid::new("Spawn info")
                    .min_col_width(200.0)
                    .show(ui, |ui| {
                        let speed = |v| 1e-3f64.max(v * 1e-3);
                        IdentedInfo::new("Position", &mut data.state_vector.position)
                            .hover_text(format!("Position relative to {}", "me"))
                            .show(ui, |ui, pos| {
                                ui.horizontal(|ui| {
                                    let speed = speed(pos.x);
                                    ui.label("x: ");
                                    let x = ui.add(egui::DragValue::new(&mut pos.x).speed(speed));
                                    if x.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(pos.y);
                                    ui.label("y: ");
                                    let y = ui.add(egui::DragValue::new(&mut pos.y).speed(speed));
                                    if y.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(pos.z);
                                    ui.label("z: ");
                                    let z = ui.add(egui::DragValue::new(&mut pos.z).speed(speed));
                                    if z.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km");
                                });
                            });
                        IdentedInfo::new("Velocity", &mut data.state_vector.velocity)
                            .hover_text(format!("Velocity relative to {}", "me"))
                            .show(ui, |ui, vel| {
                                ui.horizontal(|ui| {
                                    let speed = speed(vel.x);
                                    ui.label("x: ");
                                    let x = ui.add(egui::DragValue::new(&mut vel.x).speed(speed));
                                    if x.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km/s");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(vel.y);
                                    ui.label("y: ");
                                    let y = ui.add(egui::DragValue::new(&mut vel.y).speed(speed));
                                    if y.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km/s");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(vel.z);
                                    ui.label("z: ");
                                    let z = ui.add(egui::DragValue::new(&mut vel.z).speed(speed));
                                    if z.changed() {
                                        window.valid_preview = false;
                                    }
                                    ui.label("km/s");
                                });
                            });
                    });

                ui.add_space(5.0);

                let button = egui::Button::new(format!("Spawn {}", data.name));
                if ui
                    .add_enabled(!data.name.is_empty(), button)
                    .on_disabled_hover_text("Name required")
                    .clicked()
                {
                    let sv = data.global_state_vector(data.start, &query_trajectory);

                    commands.trigger(LoadShipEvent(Ship::new(
                        data.name.to_string(),
                        data.start,
                        sv.position,
                        sv.velocity,
                        Vec::new(),
                    )));

                    commands.remove_resource::<Self>();
                }

                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size.x = 150.0;

                    let precision = Duration::from_seconds(1.0);
                    let mut duration = data.preview_duration.to_seconds();
                    let speed = 1e-1f64.max(duration * 1e-3);

                    ui.label("Preview length:");
                    if ui
                        .add(
                            egui::DragValue::new(&mut duration)
                                .speed(speed)
                                .range(0.0..=f64::INFINITY)
                                .custom_formatter(|value, _| {
                                    Duration::from_seconds(value).to_string()
                                })
                                .custom_parser(|text| {
                                    Duration::from_str(text).ok().map(|d| d.to_seconds())
                                }),
                        )
                        .changed()
                    {
                        window.valid_preview = false;
                    }
                    data.preview_duration = Duration::from_seconds(duration)
                        .round(precision)
                        .max(Duration::from_seconds(1.0));
                });
            });

        if !window.valid_preview {
            let sv = data.global_state_vector(data.start, &query_trajectory);
            *builder = DiscreteStatesBuilder::new(
                preview,
                data.start,
                sv.velocity,
                sv.position,
                new_context(),
            );

            commands.trigger(ComputePredictionEvent::<DiscreteStatesBuilder>::with(
                [preview],
                data.preview_duration,
                100,
            ));

            window.valid_preview = true;
        }

        if !open {
            commands.remove_resource::<Self>();
        }
    }

    fn on_close(mut commands: Commands, preview: Query<Entity, With<ShipSpawnerData>>) {
        for entity in preview.iter() {
            commands
                .entity(entity)
                .retain::<(ShipSpawnerData, Parent)>();
        }
    }
}

fn load_solar_system_state(
    mut commands: Commands,
    mut ev_loaded: EventReader<DialogFileLoaded<ShipFile>>,
) {
    if !ev_loaded.is_empty() {
        commands.remove_resource::<ShipSpawnerWindow>();
    }
    for loaded in ev_loaded.read() {
        if let Ok(ship) = serde_json::from_slice(&loaded.contents) {
            commands.trigger(LoadShipEvent(ship));
        }
    }
}
