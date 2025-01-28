use crate::{
    flight_plan::FlightPlan,
    hierarchy,
    load::SystemRoot,
    prediction::{Mu, Trajectory, TrajectoryData},
    time::SimulationTime,
    ui::{epoch_clamped_parser, show_tree, FixedUiSet, ParsedTextEdit},
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::Epoch;

pub struct ExportPlugin;

impl Plugin for ExportPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ExportSolarSystemEvent>().add_systems(
            Update,
            (ExportWindow::show.after(FixedUiSet), export_solar_system)
                .run_if(in_state(MainState::Running)),
        );
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

#[derive(Default, Resource)]
pub struct ExportWindow;

impl ExportWindow {
    #[expect(clippy::type_complexity)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        mut event: EventWriter<ExportSolarSystemEvent>,
        window: Option<Res<Self>>,
        sim_time: Res<SimulationTime>,
        query_trajectory: Query<Entity, With<Trajectory>>,
        mut query_hierarchy: Query<
            (Entity, &Name, Option<&hierarchy::Children>),
            Or<(With<SystemRoot>, Without<FlightPlan>)>,
        >,
        root: Single<Entity, With<SystemRoot>>,
        mut bodies: Local<Option<bevy::ecs::entity::EntityHashSet>>,
        mut cached_exports: Local<Option<[ExportType; 1]>>,
        mut current: Local<usize>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

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
                egui::ComboBox::from_id_salt("export type").show_index(
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
                    ui.spacing_mut().item_spacing = [1.0, 3.0].into();
                    show_tree(
                        *root,
                        &query_hierarchy.transmute_lens().query(),
                        |i| i != 0,
                        ui,
                        |ui, state, (entity, name, children), depth| {
                            let has_children = children.is_some_and(|c| !c.is_empty());
                            let has_trajectory = query_trajectory.contains(entity);
                            if has_trajectory {
                                let mut ui_builder = egui::UiBuilder::new();
                                if !has_children {
                                    ui_builder = ui_builder.invisible();
                                }
                                ui.scope_builder(ui_builder, |ui| {
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

#[derive(Event, PartialEq, PartialOrd)]
pub struct ExportSolarSystemEvent {
    pub export: ExportType,
    pub bodies: Vec<Entity>,
}

pub struct ExportSolarSystemFile;

fn export_solar_system(
    mut commands: Commands,
    mut event: EventReader<ExportSolarSystemEvent>,
    query: Query<(&Name, &Mu, &Trajectory)>,
) {
    if let Some(event) = event.read().next() {
        let bytes = match event.export {
            ExportType::State { epoch } => serde_json::to_vec_pretty(&serde_json::json!({
                "epoch": epoch,
                "bodies": query
                    .iter_many(&event.bodies)
                    .map(|(name, mu, trajectory)| {
                        trajectory.state_vector(epoch).map(|state_vector|
                            serde_json::json!({
                                "name": name.to_string(),
                                "mu": **mu,
                                "position": state_vector.position,
                                "velocity": state_vector.velocity,
                            })
                        )
                    })
                    .collect::<Option<Vec<_>>>()
            }))
            .unwrap(),
        };

        commands
            .dialog()
            .add_filter("JSON", &["json"])
            .save_file::<ExportSolarSystemFile>(bytes);
    }
}
