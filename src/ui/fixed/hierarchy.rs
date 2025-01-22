use crate::{
    camera::Followed, hierarchy, load::SystemRoot, plot::TrajectoryPlot, selection::Selected,
    ui::show_tree,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

pub fn solar_system_hierarchy(
    mut contexts: EguiContexts,
    mut followed: ResMut<Followed>,
    mut selected: ResMut<Selected>,
    mut query: Query<&mut TrajectoryPlot>,
    query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
    root: Query<Entity, With<SystemRoot>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    let root = root.single();

    egui::SidePanel::left("Hierachy")
        .resizable(false)
        .min_width(250.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing = [1.0, 3.0].into();
                    show_tree(
                        root,
                        &query_hierarchy,
                        |_| true,
                        ui,
                        |ui, state, (entity, name, children), _| {
                            let has_children = children.is_some_and(|c| !c.is_empty());

                            let mut ui_builder = egui::UiBuilder::new();
                            if !has_children {
                                ui_builder = ui_builder.invisible();
                            }
                            ui.scope_builder(ui_builder, |ui| {
                                state.show_toggle_button(
                                    ui,
                                    egui::collapsing_header::paint_default_icon,
                                );
                            });

                            // Camera follow toggle
                            ui.scope(|ui| {
                                let is_followed = followed.0 == Some(entity);
                                if ui.selectable_label(is_followed, "⌖").clicked() {
                                    followed.replace(entity);
                                }
                            });
                            if let Ok(mut plot) = query.get_mut(entity) {
                                // Plot toggle
                                ui.scope(|ui| {
                                    let [r, g, b, _a] = plot.color.to_srgba().to_u8_array();
                                    ui.visuals_mut().selection.bg_fill =
                                        egui::Color32::from_gray(80);
                                    ui.visuals_mut().selection.stroke =
                                        egui::Stroke::new(1.0, egui::Color32::from_rgb(r, g, b));

                                    ui.toggle_value(&mut plot.enabled, "○");
                                });

                                let mut ui_builder = egui::UiBuilder::new();
                                if !has_children {
                                    ui_builder = ui_builder.invisible();
                                }
                                // Children plot toggle
                                ui.scope_builder(ui_builder, |ui| {
                                    ui.visuals_mut().selection.bg_fill =
                                        egui::Color32::from_gray(80);

                                    let plotted_count = children
                                        .into_iter()
                                        .flatten()
                                        .filter_map(|entity| query.get(*entity).ok())
                                        .filter(|p| p.enabled)
                                        .count();

                                    if children.is_some_and(|c| c.len() != plotted_count) {
                                        let color = &mut ui.visuals_mut().selection.bg_fill;
                                        *color = color.linear_multiply(0.4);
                                    }

                                    let any_plotted = plotted_count > 0;
                                    if ui.selectable_label(any_plotted, "A").clicked() {
                                        for child in children.into_iter().flatten() {
                                            if let Ok(mut plot) = query.get_mut(*child) {
                                                plot.enabled = !any_plotted;
                                            }
                                        }
                                    }
                                });

                                let checked = **selected == Some(entity);
                                if ui.selectable_label(checked, name.as_str()).clicked() {
                                    **selected = Some(entity);
                                }
                            } else {
                                ui.label(name.as_str());
                            }
                        },
                    );
                })
        });
}
