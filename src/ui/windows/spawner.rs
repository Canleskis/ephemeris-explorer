use crate::{
    hierarchy,
    plot::TrajectoryPlot,
    prediction::{DiscreteStates, DiscreteStatesBuilder, StateVector, Trajectory, TrajectoryData},
    time::SimulationTime,
    ui::{get_name, show_tree, IdentedInfo, Labelled},
    MainState, SystemRoot,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use hifitime::{Duration, Epoch};

pub struct ShipSpawnerPlugin;

impl Plugin for ShipSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            ShipSpawnerWindow::show.run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Default, Resource)]
pub struct ShipSpawnerWindow;

impl ShipSpawnerWindow {
    #[expect(clippy::too_many_arguments)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        window: Option<Res<Self>>,
        root: Query<Entity, With<SystemRoot>>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        sim_time: Res<SimulationTime>,
        query_trajectory: Query<&Trajectory>,
        mut name_buffer: Local<String>,
        mut reference: Local<Option<Entity>>,
        mut spawn_sv: Local<StateVector<DVec3>>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let root = root.single();
        let reference = reference.get_or_insert(root);

        let mut open = window.is_some();
        egui::Window::new("Ship spawner")
            .open(&mut open)
            .show(ctx, |ui| {
                let time = sim_time.current().round(Duration::from_seconds(1.0));
                ui.label(format!("Current time: {}", time));

                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.add(egui::TextEdit::singleline(&mut *name_buffer));
                });

                ui.add_space(5.0);

                egui::ComboBox::from_id_source("Reference")
                    .wrap_mode(egui::TextWrapMode::Extend)
                    .selected_text(get_name(*reference, query_hierarchy.transmute_lens()))
                    .show_ui(ui, |ui| {
                        show_tree(
                            root,
                            &query_hierarchy,
                            |_| true,
                            ui,
                            |ui, _, (ref_entity, ref_name, _), _| {
                                ui.horizontal(|ui| {
                                    ui.selectable_value(
                                        &mut *reference,
                                        ref_entity,
                                        ref_name.as_str(),
                                    );

                                    ui.add_space(10.0);
                                });
                            },
                        );
                    });

                ui.add_space(5.0);

                egui::Grid::new("Spawn info")
                    .min_col_width(200.0)
                    .show(ui, |ui| {
                        let speed = |v| 1e-3f64.max(v / 100.0);
                        IdentedInfo::new("Position", &mut spawn_sv.position)
                            .hover_text(format!("Position relative to {}", "me"))
                            .show(ui, |ui, position| {
                                ui.horizontal(|ui| {
                                    let speed = speed(position.x);
                                    ui.label("x: ");
                                    ui.add(egui::DragValue::new(&mut position.x).speed(speed));
                                    ui.label("km");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(position.y);
                                    ui.label("y: ");
                                    ui.add(egui::DragValue::new(&mut position.y).speed(speed));
                                    ui.label("km");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(position.z);
                                    ui.label("z: ");
                                    ui.add(egui::DragValue::new(&mut position.z).speed(speed));
                                    ui.label("km");
                                });
                            });
                        IdentedInfo::new("Velocity", &mut spawn_sv.velocity)
                            .hover_text(format!("Velocity relative to {}", "me"))
                            .show(ui, |ui, velocity| {
                                ui.horizontal(|ui| {
                                    let speed = speed(velocity.x);
                                    ui.label("x: ");
                                    ui.add(egui::DragValue::new(&mut velocity.x).speed(speed));
                                    ui.label("km/s");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(velocity.y);
                                    ui.label("y: ");
                                    ui.add(egui::DragValue::new(&mut velocity.y).speed(speed));
                                    ui.label("km/s");
                                });
                                ui.horizontal(|ui| {
                                    let speed = speed(velocity.z);
                                    ui.label("z: ");
                                    ui.add(egui::DragValue::new(&mut velocity.z).speed(speed));
                                    ui.label("km/s");
                                });
                            });
                    });

                ui.add_space(5.0);

                let button = egui::Button::new(format!("Spawn {}", *name_buffer));
                if ui
                    .add_enabled(!name_buffer.is_empty(), button)
                    .on_disabled_hover_text("Name required")
                    .clicked()
                {
                    let ref_sv = query_trajectory
                        .get(*reference)
                        .ok()
                        .and_then(|traj| traj.state_vector(time));

                    let Some(ref_sv) = ref_sv else {
                        bevy::log::error!("failed to spawn ship at {}", time);
                        return;
                    };

                    let sv = ref_sv + *spawn_sv;
                    commands.entity(root).with_children(|commands| {
                        let mut ship = commands.spawn_empty();
                        let radius = 0.1;
                        let depth = 3;
                        ship.insert((
                            Name::new(name_buffer.clone()),
                            crate::floating_origin::BigReferenceFrameBundle {
                                transform: Transform::from_translation(sv.position.as_vec3()),
                                ..default()
                            },
                            crate::selection::Clickable {
                                radius,
                                index: depth + 1,
                            },
                            crate::camera::CanFollow {
                                min_distance: radius as f64 * 1.05,
                                max_distance: 5e10,
                            },
                            Labelled {
                                style: TextStyle {
                                    font_size: 15.0,
                                    color: Color::WHITE,
                                    ..default()
                                },
                                offset: Vec2::new(0.0, radius) * 1.1,
                                index: depth + 1,
                            },
                            TrajectoryPlot {
                                enabled: true,
                                color: LinearRgba::GREEN.into(),
                                start: Epoch::default() - Duration::MAX,
                                end: Epoch::default() + Duration::MAX,
                                threshold: 0.5,
                                max_points: 10_000,
                                reference: Some(*reference),
                            },
                            Trajectory::new(DiscreteStates::new(time, sv.velocity, sv.position)),
                            DiscreteStatesBuilder::new(ship.id(), time, sv.velocity, sv.position),
                        ));

                        let parent = *reference;
                        let child = ship.id();
                        commands.add_command(hierarchy::AddChild { parent, child });
                    });
                }
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}
