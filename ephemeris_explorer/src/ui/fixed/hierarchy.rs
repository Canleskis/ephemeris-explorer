use crate::{
    analysis::{OrbitPlotConfig, Satellites},
    camera::Followed,
    load::SystemRoot,
    ouf_of_bounds::OutOfBounds,
    selection::{Selectable, Selected},
    ui::show_tree,
};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

pub fn solar_system_hierarchy(
    mut contexts: EguiContexts,
    mut followed: ResMut<Followed>,
    mut selected: ResMut<Selected>,
    query: Query<(Entity, &Name, Option<&Satellites>), Without<OutOfBounds>>,
    query_inactive: Query<(Entity, &Name), With<OutOfBounds>>,
    mut query_plot: Query<&mut OrbitPlotConfig>,
    query_selectable: Query<&Selectable>,
    root: Single<Entity, With<SystemRoot>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::SidePanel::left("Hierarchy")
        .resizable(false)
        .min_width(250.0)
        .show(ctx, |ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);

            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing = [4.0, 4.0].into();

                    show_tree(
                        ui,
                        query.get(*root).unwrap(),
                        |&(e, name, _)| (e, name),
                        |_, _| true,
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

                            // Plot toggle
                            ui.scope(|ui| {
                                let Ok(mut plot_config) = query_plot.get_mut(entity) else {
                                    return;
                                };
                                let [r, g, b, _a] = plot_config.color.to_srgba().to_u8_array();
                                ui.visuals_mut().selection.bg_fill = egui::Color32::from_gray(80);
                                ui.visuals_mut().selection.stroke.color =
                                    egui::Color32::from_rgb(r, g, b);

                                if ui.selectable_label(plot_config.enabled, "○").clicked() {
                                    plot_config.enabled = !plot_config.enabled;
                                }
                            });

                            if query_selectable.contains(entity) {
                                let is_selected = **selected == Some(entity);
                                if ui.selectable_label(is_selected, name.as_str()).clicked() {
                                    **selected = Some(entity);
                                }
                            } else {
                                ui.style_mut().interaction.selectable_labels = false;
                                ui.add_space(ui.spacing().button_padding.x);
                                ui.label(name.as_str());
                            }

                            children.map(|c| c.into_iter().flat_map(|e| query.get(*e).ok()))
                        },
                    );

                    if query_inactive.is_empty() {
                        return;
                    }
                    let mut inactive = Some(query_inactive.iter());
                    show_tree(
                        ui,
                        (Entity::PLACEHOLDER, &Name::new("Outside of time bounds")),
                        |&(e, name)| (e, name),
                        |_, _| true,
                        |ui, state, (entity, name), i| {
                            let has_children = inactive.is_some();

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

                            if i > 0 {
                                // Camera follow toggle
                                ui.add_enabled_ui(false, |ui| ui.selectable_label(false, "⌖"));
                            }

                            // Plot toggle
                            ui.scope(|ui| {
                                let Ok(mut plot_config) = query_plot.get_mut(entity) else {
                                    return;
                                };
                                let [r, g, b, _a] = plot_config.color.to_srgba().to_u8_array();
                                ui.visuals_mut().selection.bg_fill = egui::Color32::from_gray(80);
                                ui.visuals_mut().selection.stroke.color =
                                    egui::Color32::from_rgb(r, g, b);

                                if ui.selectable_label(plot_config.enabled, "○").clicked() {
                                    plot_config.enabled = !plot_config.enabled;
                                }
                            });

                            if query_selectable.contains(entity) {
                                let is_selected = **selected == Some(entity);
                                if ui.selectable_label(is_selected, name.as_str()).clicked() {
                                    **selected = Some(entity);
                                }
                            } else {
                                ui.style_mut().interaction.selectable_labels = false;
                                ui.add_space(ui.spacing().button_padding.x);
                                ui.label(name.as_str());
                            }

                            inactive.take()
                        },
                    );
                })
        });
}
