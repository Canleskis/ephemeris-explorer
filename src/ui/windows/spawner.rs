use crate::{
    camera::Followed,
    compound_trajectory::TrajectoryReferenceTranslated,
    floating_origin::BigGridBundle,
    hierarchy,
    load::{LoadShipEvent, Ship, SystemRoot},
    plot::{PlotPoints, TrajectoryPlot},
    prediction::{
        ComputePredictionEvent, DiscreteStates, DiscreteStatesBuilder, StateVector, Trajectory,
        TrajectoryData,
    },
    time::SimulationTime,
    ui::{get_name, show_tree, FixedUiSet, IdentedInfo, Labelled, ParsedTextEdit},
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
            (ShipSpawnerWindow::show, load_solar_system_state)
                .chain()
                .after(FixedUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

pub struct ShipFile;

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
        query_trajectory: Query<&Trajectory>,
    ) -> StateVector<DVec3> {
        self.reference
            .and_then(|entity| query_trajectory.get(entity).ok())
            .and_then(|traj| traj.state_vector(at))
            .unwrap_or_default()
            + self.state_vector
    }
}

#[derive(Component)]
pub struct Preview;

#[derive(Default, Resource)]
pub struct ShipSpawnerWindow {
    pub valid_preview: bool,
}

impl ShipSpawnerWindow {
    #[expect(clippy::too_many_arguments)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        sim_time: Res<SimulationTime>,
        window: Option<ResMut<Self>>,
        root: Single<Entity, With<SystemRoot>>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>), Without<Preview>>,
        mut query_preview: Query<
            (
                Entity,
                &mut Name,
                &mut DiscreteStatesBuilder,
                &mut TrajectoryReferenceTranslated,
            ),
            With<Preview>,
        >,
        mut query_trajectory: Query<&Trajectory, Without<Preview>>,
        followed: Res<Followed>,
        mut data: Local<Option<ShipSpawnerData>>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut lens = query_trajectory.transmute_lens();
        let mut query_trajectory = lens.query();

        let Some(mut window) = window else {
            return;
        };

        let data = data.get_or_insert_with(|| ShipSpawnerData {
            name: "Ship".to_string(),
            start: sim_time.current().floor(Duration::from_seconds(1.0)),
            state_vector: StateVector::new(
                DVec3::new(10_000.0, 0.0, 0.0),
                DVec3::new(0.0, 8.8, 0.0),
            ),
            reference: **followed,
            preview_duration: Duration::from_days(5.0),
        });

        if window.is_added() {
            let radius = 0.01;
            let sv = data.global_state_vector(data.start, query_trajectory.reborrow());

            let mut entity = commands.spawn_empty();
            entity.insert((
                Preview,
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
                DiscreteStatesBuilder::new(entity.id(), data.start, sv.velocity, sv.position),
                TrajectoryReferenceTranslated::new(entity.id(), data.reference, data.start),
                TrajectoryPlot {
                    enabled: true,
                    color: bevy::color::palettes::css::FUCHSIA.into(),
                    start: Epoch::from_tai_duration(-Duration::MAX),
                    end: Epoch::from_tai_duration(Duration::MAX),
                    threshold: 0.5,
                    max_points: 10_000,
                },
                PlotPoints::default(),
            ));

            let entity = entity.id();
            commands.entity(*root).add_child(entity);
        };

        let Ok((preview, mut name, mut builder, mut trajectory)) = query_preview.get_single_mut()
        else {
            return;
        };

        let mut request_close = false;
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
                    let edit = ui.add(ParsedTextEdit::singleline(
                        &mut data.start,
                        |buf| {
                            Epoch::from_str(buf).ok().map(|t| {
                                t.clamp(sim_time.start(), sim_time.end())
                                    .floor(Duration::from_seconds(1.0))
                            })
                        },
                        Epoch::to_string,
                    ));
                    if edit.changed() {
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
                    .map(|entity| get_name(entity, query_hierarchy.transmute_lens()))
                    .unwrap_or_else(|| "None".to_string());
                egui::ComboBox::from_id_salt("Reference")
                    .wrap_mode(egui::TextWrapMode::Extend)
                    .selected_text(name)
                    .show_ui(ui, |ui| {
                        show_tree(
                            *root,
                            &query_hierarchy.transmute_lens().query(),
                            |_| true,
                            ui,
                            |ui, _, (ref_entity, ref_name, _), _| {
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
                                        trajectory.relative.reference = data.reference;
                                    }

                                    ui.add_space(10.0);
                                });
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
                    let sv = data.global_state_vector(data.start, query_trajectory.reborrow());

                    commands.trigger(LoadShipEvent(Ship::new(
                        data.name.to_string(),
                        data.start,
                        sv.position,
                        sv.velocity,
                        Vec::new(),
                    )));

                    request_close = true;
                }

                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size = [150.0, 18.0].into();

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
            let sv = data.global_state_vector(data.start, query_trajectory.reborrow());
            *builder = DiscreteStatesBuilder::new(preview, data.start, sv.velocity, sv.position);

            commands.trigger(ComputePredictionEvent::<DiscreteStatesBuilder>::with(
                [preview],
                data.preview_duration,
                100,
            ));

            window.valid_preview = true;
        }

        if !open || request_close {
            commands.remove_resource::<Self>();
            commands.entity(preview).despawn_recursive();
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
