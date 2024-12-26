use crate::{
    hierarchy,
    load::{LoadShipEvent, Ship},
    prediction::{StateVector, Trajectory, TrajectoryData},
    time::SimulationTime,
    ui::{get_name, show_tree, FixedUiSet, IdentedInfo},
    MainState, SystemRoot,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::Duration;

pub struct ShipSpawnerPlugin;

impl Plugin for ShipSpawnerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                ShipSpawnerWindow::show.after(FixedUiSet),
                load_solar_system_state,
            )
                .chain()
                .run_if(in_state(MainState::Running)),
        );
    }
}

pub struct ShipFile;

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
                if ui.button("From file").clicked() {
                    commands.dialog().load_multiple_files::<ShipFile>();
                }

                ui.separator();

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
                        .and_then(|traj| traj.state_vector(time))
                        .unwrap_or_default();

                    let sv = ref_sv + *spawn_sv;

                    commands.trigger(LoadShipEvent(Ship {
                        name: name_buffer.clone(),
                        start: time,
                        position: sv.position,
                        velocity: sv.velocity,
                        burns: Vec::new(),
                    }));
                }
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}

fn load_solar_system_state(
    mut commands: Commands,
    mut ev_loaded: EventReader<DialogFileLoaded<ShipFile>>,
) {
    for loaded in ev_loaded.read() {
        if let Ok(ship) = serde_json::from_slice(&loaded.contents) {
            commands.trigger(LoadShipEvent(ship));
        }
    }
}
