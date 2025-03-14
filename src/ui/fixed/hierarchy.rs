use crate::{
    camera::Followed,
    hierarchy::OrbitedBy,
    load::SystemRoot,
    selection::{Selectable, Selected},
    ui::{show_tree, SourceOf, TrajectoryPlot},
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

pub fn solar_system_hierarchy(
    mut contexts: EguiContexts,
    mut followed: ResMut<Followed>,
    mut selected: ResMut<Selected>,
    query_hierarchy: Query<(Entity, &Name, Option<&OrbitedBy>)>,
    mut query_plot: Query<&mut TrajectoryPlot>,
    query_source_of: Query<&SourceOf>,
    query_selectable: Query<&Selectable>,
    root: Single<Entity, With<SystemRoot>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::SidePanel::left("Hierarchy")
        .resizable(false)
        .min_width(250.0)
        .show(ctx, |ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);

            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing = [1.0, 3.0].into();
                    show_tree(
                        ui,
                        query_hierarchy.get(*root).unwrap(),
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
                                if let Ok(source_of) = query_source_of.get(entity) {
                                    if let Some(first) =
                                        source_of.first().and_then(|e| query_plot.get(*e).ok())
                                    {
                                        let [r, g, b, _a] = first.color.to_srgba().to_u8_array();
                                        ui.visuals_mut().selection.bg_fill =
                                            egui::Color32::from_gray(80);
                                        ui.visuals_mut().selection.stroke = egui::Stroke::new(
                                            1.0,
                                            egui::Color32::from_rgb(r, g, b),
                                        );
                                    }

                                    let enabled =
                                        query_plot.iter_many(source_of).any(|p| p.enabled);

                                    if ui.selectable_label(enabled, "○").clicked() {
                                        let mut iter = query_plot.iter_many_mut(source_of);
                                        while let Some(mut plot) = iter.fetch_next() {
                                            plot.enabled = !enabled;
                                        }
                                    }
                                }
                            });

                            if query_selectable.contains(entity) {
                                let checked = **selected == Some(entity);
                                if ui.selectable_label(checked, name.as_str()).clicked() {
                                    **selected = Some(entity);
                                }
                            } else {
                                ui.style_mut().interaction.selectable_labels = false;
                                ui.add_space(ui.spacing().button_padding.x);
                                ui.label(name.as_str());
                            }

                            children
                                .map(|c| c.into_iter().flat_map(|e| query_hierarchy.get(*e).ok()))
                        },
                    );
                })
        });
}
