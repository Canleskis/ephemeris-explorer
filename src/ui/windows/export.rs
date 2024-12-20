use crate::{
    hierarchy,
    load::LoadSolarSystemEvent,
    prediction::{Mu, Trajectory, TrajectoryData},
    time::SimulationTime,
    ui::{epoch_clamped_parser, show_tree, ParsedTextEdit},
    MainState, SystemRoot,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::Epoch;

pub struct SolarSystemDir;

pub struct ExportPlugin;

impl Plugin for ExportPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(
            FileDialogPlugin::new()
                .with_pick_directory::<SolarSystemDir>()
                .with_save_file::<ExportFile>(),
        )
        .add_event::<ExportSolarSystemEvent>()
        .add_systems(
            Update,
            (
                ExportWindow::show,
                load_solar_system_state,
                (export_solar_system, export_solar_system_result).chain(),
            )
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Default, Resource)]
pub struct ExportWindow;

impl ExportWindow {
    #[expect(clippy::too_many_arguments)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        mut event: EventWriter<ExportSolarSystemEvent>,
        window: Option<Res<Self>>,
        sim_time: Res<SimulationTime>,
        query_trajectory: Query<Entity, With<Trajectory>>,
        query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        root: Query<Entity, With<SystemRoot>>,
        mut bodies: Local<Option<bevy::utils::EntityHashSet<Entity>>>,
        mut cached_exports: Local<Option<[ExportType; 1]>>,
        mut current: Local<usize>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let root = root.single();

        let mut open = window.is_some();
        egui::Window::new("Export").open(&mut open).show(ctx, |ui| {
            let export = cached_exports.get_or_insert_with(|| {
                [ExportType::State {
                    epoch: sim_time.current(),
                }]
            });
            let bodies = bodies.get_or_insert_with(|| query_trajectory.iter().collect());

            ui.horizontal(|ui| {
                if ui.button("Export").clicked() {
                    event.send(ExportSolarSystemEvent {
                        export: export[*current],
                        bodies: bodies.iter().copied().collect(),
                    });
                }
                egui::ComboBox::from_id_source("export type").show_index(
                    ui,
                    &mut current,
                    export.len(),
                    |i| export[i].to_string(),
                );
            });

            ui.add_space(5.0);

            ui.spacing_mut().text_edit_width = 300.0;
            match &mut export[*current] {
                ExportType::State { epoch } => {
                    ui.horizontal(|ui| {
                        ui.label("Epoch:");
                        ui.add(ParsedTextEdit::singleline(
                            epoch,
                            epoch_clamped_parser(sim_time.start(), sim_time.end()),
                            Epoch::to_string,
                        ));
                    });
                }
            }

            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    show_tree(
                        root,
                        &query_hierarchy,
                        |i| i != 0,
                        ui,
                        |ui, state, (entity, name, children), depth| {
                            let has_children = children.is_some_and(|c| !c.is_empty());
                            let has_trajectory = query_trajectory.get(entity).is_ok();
                            if has_trajectory {
                                ui.add_visible_ui(has_children, |ui| {
                                    let selected_count = children
                                        .into_iter()
                                        .flatten()
                                        .filter_map(|entity| bodies.get(entity))
                                        .count();

                                    if children.is_some_and(|c| c.len() != selected_count) {
                                        let color = &mut ui.visuals_mut().selection.bg_fill;
                                        *color = color.linear_multiply(0.4);
                                    }

                                    let any_selected = selected_count > 0;
                                    if ui.selectable_label(any_selected, "A").clicked() {
                                        for entity in children.into_iter().flatten() {
                                            match any_selected {
                                                true => bodies.remove(entity),
                                                false => bodies.insert(*entity),
                                            };
                                        }
                                    }
                                });
                            }

                            if has_trajectory {
                                let checked = bodies.contains(&entity);
                                if ui.selectable_label(checked, name.as_str()).clicked() {
                                    match bodies.entry(entity) {
                                        bevy::utils::hashbrown::hash_set::Entry::Occupied(
                                            entry,
                                        ) => {
                                            entry.remove();
                                        }
                                        bevy::utils::hashbrown::hash_set::Entry::Vacant(entry) => {
                                            entry.insert();
                                        }
                                    }
                                }
                            } else {
                                ui.label(name.as_str());
                            }

                            if depth == 0 {
                                state.show_toggle_button(ui, reversed_paint_default_icon);
                            }

                            ui.add_space(10.0);
                        },
                    );
                });
        });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}

pub fn reversed_paint_default_icon(ui: &mut egui::Ui, openness: f32, response: &egui::Response) {
    let visuals = ui.style().interact(response);

    let rect = response.rect;

    // Draw a pointy triangle arrow:
    let rect = egui::Rect::from_center_size(
        rect.center(),
        egui::vec2(rect.width(), rect.height()) * 0.75,
    );
    let rect = rect.expand(visuals.expansion);
    let mut points = vec![rect.left_top(), rect.right_top(), rect.center_bottom()];
    use std::f32::consts::TAU;
    let rotation = egui::emath::Rot2::from_angle(egui::remap(openness, 0.0..=1.0, TAU / 4.0..=0.0));
    for p in &mut points {
        *p = rect.center() + rotation * (*p - rect.center());
    }

    ui.painter().add(egui::Shape::convex_polygon(
        points,
        visuals.fg_stroke.color,
        egui::Stroke::NONE,
    ));
}

fn load_solar_system_state(
    asset_server: Res<AssetServer>,
    mut ev_picked: EventReader<DialogDirectoryPicked<SolarSystemDir>>,
    mut events: EventWriter<LoadSolarSystemEvent>,
) {
    for picked in ev_picked.read() {
        match LoadSolarSystemEvent::try_from_dir(&picked.path, &asset_server) {
            Ok(event) => {
                events.send(event);
            }
            Err(err) => {
                bevy::log::error!(
                    "Failed to load solar system at {}: {}",
                    picked.path.display(),
                    err
                );
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum ExportType {
    State { epoch: hifitime::Epoch },
}

impl std::fmt::Display for ExportType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportType::State { .. } => {
                write!(f, "Export state")
            }
        }
    }
}

#[derive(Event, PartialEq, PartialOrd)]
pub struct ExportSolarSystemEvent {
    pub export: ExportType,
    pub bodies: Vec<Entity>,
}

struct ExportFile;

fn export_solar_system(
    mut commands: Commands,
    mut event: EventReader<ExportSolarSystemEvent>,
    query: Query<(&Name, &Mu, &Trajectory)>,
) {
    if let Some(event) = event.read().next() {
        let bytes = match event.export {
            ExportType::State { epoch } => {
                #[derive(serde::Serialize)]
                struct BodyJson {
                    name: String,
                    mu: f64,
                    position: bevy::math::DVec3,
                    velocity: bevy::math::DVec3,
                }
                #[derive(serde::Serialize)]
                struct StateJson {
                    epoch: hifitime::Epoch,
                    bodies: Vec<BodyJson>,
                }

                let bodies = query
                    .iter_many(&event.bodies)
                    .map(|(name, mu, trajectory)| {
                        trajectory.state_vector(epoch).map(|state_vector| BodyJson {
                            name: name.to_string(),
                            mu: **mu,
                            position: state_vector.position,
                            velocity: state_vector.velocity,
                        })
                    })
                    .collect::<Option<Vec<_>>>();

                match bodies {
                    None => {
                        bevy::log::error!("Something went wrong when exporting bodies");
                        return;
                    }
                    Some(bodies) => {
                        serde_json::to_vec_pretty(&StateJson { epoch, bodies }).unwrap()
                    }
                }
            }
        };

        commands
            .dialog()
            .add_filter("JSON", &["json"])
            .save_file::<ExportFile>(bytes);
    }
}

fn export_solar_system_result(mut ev_saved: EventReader<DialogFileSaved<ExportFile>>) {
    for ev in ev_saved.read() {
        match ev.result {
            Ok(_) => bevy::log::info!("File {} successfully saved", ev.file_name),
            Err(ref err) => bevy::log::error!("Failed to save {}: {}", ev.file_name, err),
        }
    }
}
