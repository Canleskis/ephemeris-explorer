use crate::{
    MainState,
    analysis::{OrbitPlotConfig, OrbitPlotReference, Primary, Satellites},
    dynamics::Trajectory,
    load::SystemRoot,
    selection::Selected,
    simulation::SimulationTime,
    ui::{
        CalendarDragValue, IdentedInfo, Length, PlotBound, PlotBoundMode, Velocity, WindowsUiSet,
        epoch_placeholder, get_name, show_tree,
    },
};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use ephemeris::{EvaluateTrajectory, RelativeTrajectory};
use ftime::Epoch;

pub struct BodyInfoPlugin;

impl Plugin for BodyInfoPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            EguiPrimaryContextPass,
            BodyInfoWindow::show
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Component)]
pub struct BodyInfoWindow;

impl BodyInfoWindow {
    pub fn show(
        mut contexts: EguiContexts,
        mut selected: ResMut<Selected>,
        sim_time: Res<SimulationTime>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&Satellites>)>,
        query_trajectory: Query<&Trajectory>,
        mut query: Query<(&Trajectory, &Primary, &mut OrbitPlotConfig)>,
        root: Single<Entity, With<SystemRoot>>,
    ) {
        let mut open = selected.is_some();
        if let Some(entity) = **selected {
            let Ok(ctx) = contexts.ctx_mut() else { return };

            let Ok((_, name, _)) = query_hierarchy.get(entity) else {
                return;
            };
            let name = name.to_string();

            egui::Window::new(name.as_str())
                .id("Selected".into())
                .open(&mut open)
                .pivot(egui::Align2::RIGHT_TOP)
                .default_pos([ctx.content_rect().size().x - 11.0, 33.0])
                .resizable([false, true])
                .max_width(350.0)
                .show(ctx, |ui| {
                    let Ok((trajectory, parent, mut plot)) = query.get_mut(entity) else {
                        return;
                    };

                    let reference = match plot.reference {
                        OrbitPlotReference::Entity(reference) => reference,
                        OrbitPlotReference::Primary => **parent,
                    };
                    let relative =
                        RelativeTrajectory::new(trajectory, query_trajectory.get(reference).ok());

                    let plot_reference_name =
                        get_name(reference, &query_hierarchy.transmute_lens().query());

                    let mut plot_config = *plot;

                    ui.horizontal(|ui| {
                        let mut is_auto =
                            matches!(plot_config.reference, OrbitPlotReference::Primary);
                        if ui.checkbox(&mut is_auto, "Auto").changed() {
                            plot_config.reference = match is_auto {
                                true => OrbitPlotReference::Primary,
                                false => OrbitPlotReference::Entity(**parent),
                            };
                        }
                        ui.add_enabled_ui(!is_auto, |ui| {
                            ui.label("Reference:");
                            egui::ComboBox::from_id_salt("Reference")
                                .wrap_mode(egui::TextWrapMode::Extend)
                                .selected_text(&plot_reference_name)
                                .show_ui(ui, |ui| {
                                    show_tree(
                                        ui,
                                        query_hierarchy.get(*root).unwrap(),
                                        |&(e, name, _)| (e, name),
                                        |_, _| true,
                                        |ui, _, (ref_entity, ref_name, children), _| {
                                            ui.horizontal(|ui| {
                                                ui.add_enabled_ui(entity != ref_entity, |ui| {
                                                    ui.selectable_value(
                                                        &mut plot_config.reference,
                                                        OrbitPlotReference::Entity(ref_entity),
                                                        ref_name.as_str(),
                                                    )
                                                    .on_disabled_hover_text(
                                                        "Cannot use itself as a reference",
                                                    );
                                                });
                                                ui.add_space(10.0);
                                            });

                                            children.map(|c| query_hierarchy.iter_many(c))
                                        },
                                    );
                                });
                        });
                    });

                    ui.separator();

                    ui.heading("Info");

                    ui.add_space(5.0);

                    let sv = relative.state_vector(sim_time.current());

                    egui::Grid::new("info").min_col_width(200.0).show(ui, |ui| {
                        IdentedInfo::new("Distance", sv.map(|sv| sv.position.length()))
                            .hover_text(format!("Distance relative to {plot_reference_name}"))
                            .show(ui, |ui, distance| {
                                if let Some(distance) = distance {
                                    ui.label(format!("{:.2}", Length::km(distance)));
                                } else {
                                    ui.label("N/A");
                                }
                            });
                        IdentedInfo::new("Speed", sv.map(|sv| sv.velocity.length()))
                            .hover_text(format!("Speed relative to {plot_reference_name}"))
                            .show(ui, |ui, speed| {
                                if let Some(speed) = speed {
                                    ui.label(format!("{:.2}", Velocity::kps(speed)));
                                } else {
                                    ui.label("N/A");
                                }
                            });
                    });

                    ui.add_space(5.0);

                    egui::CollapsingHeader::new("Details").show_unindented(ui, |ui| {
                        egui::Grid::new("info details")
                            .min_col_width(200.0)
                            .show(ui, |ui| {
                                IdentedInfo::new("Position", sv.map(|sv| sv.position))
                                    .hover_text(format!(
                                        "Position relative to {plot_reference_name}"
                                    ))
                                    .show(ui, |ui, position| {
                                        if let Some(pos) = position {
                                            ui.label(format!("x: {:.6}", Length::km(pos.x)));
                                            ui.label(format!("y: {:.6}", Length::km(pos.y)));
                                            ui.label(format!("z: {:.6}", Length::km(pos.z)));
                                        } else {
                                            ui.label("x: N/A");
                                            ui.label("y: N/A");
                                            ui.label("z: N/A");
                                        }
                                    });
                                IdentedInfo::new("Velocity", sv.map(|sv| sv.velocity))
                                    .hover_text(format!(
                                        "Velocity relative to {plot_reference_name}"
                                    ))
                                    .show(ui, |ui, velocity| {
                                        if let Some(vel) = velocity {
                                            ui.label(format!("x: {:.3}", Velocity::kps(vel.x)));
                                            ui.label(format!("y: {:.3}", Velocity::kps(vel.y)));
                                            ui.label(format!("z: {:.3}", Velocity::kps(vel.z)));
                                        } else {
                                            ui.label("x: N/A");
                                            ui.label("y: N/A");
                                            ui.label("z: N/A");
                                        }
                                    });
                            });
                    });

                    ui.add_space(5.0);
                    ui.separator();
                    ui.heading("Plotting");
                    ui.add_space(4.0);

                    ui.checkbox(&mut plot_config.enabled, "Enabled");

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Max points:");
                        ui.add(egui::Slider::new(
                            &mut plot_config.max_points_per_segment,
                            1..=100_000,
                        ));
                    });

                    ui.add_space(5.0);

                    plot_bound(
                        ui,
                        "Start:",
                        &mut plot_config.start,
                        sim_time.current(),
                        "-∞",
                    );
                    ui.add_space(5.0);
                    plot_bound(ui, "End:  ", &mut plot_config.end, sim_time.current(), "∞");

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Resolution:");
                        ui.add(
                            egui::Slider::new(&mut plot_config.resolution, 0.5..=10.0)
                                .logarithmic(true),
                        );

                        ui.label("Color:");
                        let mut color = plot_config.color.to_linear().to_f32_array();
                        ui.color_edit_button_rgba_unmultiplied(&mut color);
                        plot_config.color = Color::LinearRgba(LinearRgba::from_f32_array(color));
                    });

                    // Prevent change detection from triggering every frame
                    if plot_config != *plot {
                        *plot = plot_config;
                    }
                });
        }

        if selected.is_some() && !open {
            **selected = None;
        }
    }
}

fn plot_bound(
    ui: &mut egui::Ui,
    label: &str,
    bound: &mut PlotBound,
    mut current: Epoch,
    unbounded: &str,
) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.spacing_mut().interact_size.x = 200.0;
        match bound.mode {
            PlotBoundMode::Epoch => ui.add(CalendarDragValue::new(&mut bound.epoch)),
            PlotBoundMode::Current => ui
                .add_enabled(false, CalendarDragValue::new(&mut current))
                .on_disabled_hover_text("Follows the current time"),
            PlotBoundMode::Unbounded => {
                epoch_placeholder(ui, unbounded).on_disabled_hover_text("Unbounded")
            }
        };
        mode_toggle(ui, &mut bound.mode, PlotBoundMode::Current, "Now", 0.0);
        mode_toggle(
            ui,
            &mut bound.mode,
            PlotBoundMode::Unbounded,
            unbounded,
            24.0,
        );
    });
}

fn mode_toggle(
    ui: &mut egui::Ui,
    current: &mut PlotBoundMode,
    mode: PlotBoundMode,
    text: &str,
    min_width: f32,
) -> egui::Response {
    let selected = *current == mode;
    let button = egui::Button::selectable(selected, text).min_size(egui::vec2(min_width, 0.0));
    let response = ui.add(button);
    if response.clicked() {
        *current = if selected { PlotBoundMode::Epoch } else { mode };
    }
    response
}
