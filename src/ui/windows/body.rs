use crate::{
    camera::{Followed, SetFollowed},
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    hierarchy,
    plot::TrajectoryPlot,
    prediction::{DiscreteStates, DiscreteStatesBuilder, Trajectory, TrajectoryData},
    selection::Selected,
    time::SimulationTime,
    ui::{get_name, nformat, relative_state_vector, show_tree, IdentedInfo, ParsedTextEdit},
    MainState, SystemRoot,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use hifitime::{Duration, Epoch};
use std::str::FromStr;

pub struct BodyInfoPlugin;

impl Plugin for BodyInfoPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            BodyInfoWindow::init.run_if(in_state(MainState::Running)),
        )
        .add_systems(
            Update,
            (BodyInfoWindow::show, BodyInfoWindow::persisting)
                .chain()
                .run_if(in_state(MainState::Running)),
        );
    }
}

fn ui_coordinate(frame: BurnFrame, ui: &mut egui::Ui, coord: &str) {
    match frame {
        BurnFrame::Frenet => match coord {
            "x" => {
                ui.label("prograde:");
            }
            "y" => {
                ui.label("radial:");
                ui.add_space(24.0);
            }
            "z" => {
                ui.label("normal:");
                ui.add_space(14.0);
            }
            _ => {}
        },
        BurnFrame::Cartesian => match coord {
            "x" => {
                ui.label("x:");
            }
            "y" => {
                ui.label("y:");
            }
            "z" => {
                ui.label("z:");
            }
            _ => {}
        },
    }
}

#[derive(Debug, Clone, Copy)]
enum OverwriteKind {
    Current,
}

#[derive(Debug, Clone, Copy)]
struct PlotOverwrite {
    previous: Epoch,
    kind: OverwriteKind,
}

impl PlotOverwrite {
    fn with(&self, sim_time: &SimulationTime, epoch: &mut Epoch) {
        match self.kind {
            OverwriteKind::Current => *epoch = sim_time.current(),
        }
    }
}

#[derive(Component)]
struct BodyInfoWindow {
    auto_plot_reference: bool,
    plot_start_overwrite: Option<PlotOverwrite>,
    plot_end_overwrite: Option<PlotOverwrite>,
    delete_body: Option<bool>,
}

#[inline]
fn precision() -> Duration {
    Duration::from_milliseconds(100.0)
}

impl BodyInfoWindow {
    fn init(
        mut commands: Commands,
        query: Query<(Entity, &TrajectoryPlot, Option<&DiscreteStatesBuilder>), Without<Self>>,
    ) {
        for (entity, plot, builder) in &query {
            commands.entity(entity).insert(Self {
                auto_plot_reference: true,
                plot_start_overwrite: Some(PlotOverwrite {
                    previous: plot.start,
                    kind: OverwriteKind::Current,
                }),
                plot_end_overwrite: None,
                delete_body: builder.map(|_| false),
            });
        }
    }

    fn persisting(
        sim_time: Res<SimulationTime>,
        selected: Res<Selected>,
        mut query: Query<(Entity, &hierarchy::Parent, &mut TrajectoryPlot, &mut Self)>,
    ) {
        for (entity, parent, mut plot, mut info) in &mut query {
            if info.auto_plot_reference {
                plot.reference = Some(**parent);
            }

            if let Some(overwrite) = &info.plot_start_overwrite {
                overwrite.with(&sim_time, &mut plot.start);
            }
            if let Some(ovewrite) = &info.plot_end_overwrite {
                ovewrite.with(&sim_time, &mut plot.end);
            }

            if info.delete_body.is_some_and(std::convert::identity) && **selected != Some(entity) {
                info.delete_body = Some(false);
            }
        }
    }

    #[expect(clippy::too_many_arguments)]
    fn show(
        mut commands: Commands,
        mut contexts: EguiContexts,
        mut selected: ResMut<Selected>,
        followed: Res<Followed>,
        sim_time: Res<SimulationTime>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        mut query_trajectory: Query<&Trajectory>,
        mut query: Query<(
            &hierarchy::Parent,
            Option<&mut FlightPlan>,
            &mut TrajectoryPlot,
            &mut Self,
        )>,
        root: Query<Entity, With<SystemRoot>>,
        mut delete: Local<bevy::utils::HashSet<uuid::Uuid>>,
    ) {
        let mut open = selected.is_some();
        if let Some(entity) = **selected {
            let Some(ctx) = contexts.try_ctx_mut() else {
                return;
            };

            let root = root.single();

            let Ok((_, name, _)) = query_hierarchy.get(entity) else {
                return;
            };
            let name = name.to_string();

            egui::Window::new(name.as_str())
                .id("Selected".into())
                .open(&mut open)
                .pivot(egui::Align2::RIGHT_TOP)
                .default_pos([ctx.screen_rect().size().x - 11.0, 33.0])
                .resizable([false, true])
                .show(ctx, |ui| {
                    let Ok((parent, mut flight_plan, mut plot, mut info)) = query.get_mut(entity)
                    else {
                        return;
                    };

                    if let Some(delete) = &mut info.delete_body {
                        if ui.button("Delete ship").clicked() {
                            *delete = true;
                        }

                        if *delete {
                            egui::Window::new("Confirm deletion")
                                .resizable(false)
                                .collapsible(false)
                                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                                .show(ui.ctx(), |ui| {
                                    ui.label(format!(
                                        r#"Are you sure you want to delete "{}"?"#,
                                        name
                                    ));
                                    ui.label("All unsaved flight plan data will be lost.");
                                    ui.add_space(10.0);
                                    ui.horizontal(|ui| {
                                        if ui.button("Confirm").clicked() {
                                            if followed.is_some_and(|f| f == entity) {
                                                commands.add(SetFollowed(Some(**parent)));
                                            }
                                            commands.entity(entity).despawn_recursive();
                                        }
                                        if ui.button("Cancel").clicked() {
                                            *delete = false;
                                        }
                                    });
                                });
                        }

                        ui.separator();
                    }

                    let plot_reference_name = &plot
                        .reference
                        .map(|r| get_name(r, query_hierarchy.transmute_lens()))
                        .unwrap_or_else(|| "None".to_string());

                    ui.checkbox(&mut info.auto_plot_reference, "Auto");

                    ui.add_enabled_ui(!info.auto_plot_reference, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Reference:");
                            egui::ComboBox::from_id_source("Reference")
                                .wrap_mode(egui::TextWrapMode::Extend)
                                .selected_text(plot_reference_name)
                                .show_ui(ui, |ui| {
                                    show_tree(
                                        root,
                                        &query_hierarchy,
                                        |_| true,
                                        ui,
                                        |ui, _, (ref_entity, ref_name, _), _| {
                                            ui.horizontal(|ui| {
                                                ui.add_enabled_ui(entity != ref_entity, |ui| {
                                                    ui.selectable_value(
                                                        &mut plot.reference,
                                                        Some(ref_entity),
                                                        ref_name.as_str(),
                                                    )
                                                    .on_disabled_hover_text(
                                                        "Cannot use itself as a reference",
                                                    );
                                                });
                                                ui.add_space(10.0);
                                            });
                                        },
                                    );
                                });
                        });
                    });

                    ui.separator();
                    ui.heading("Info");
                    ui.add_space(5.0);
                    egui::Grid::new("Prediction info")
                        .min_col_width(200.0)
                        .show(ui, |ui| {
                            let sv = relative_state_vector(
                                entity,
                                plot.reference,
                                sim_time.current(),
                                query_trajectory.transmute_lens(),
                            );

                            IdentedInfo::new("Position", sv.map(|sv| sv.position))
                                .hover_text(format!("Position relative to {}", plot_reference_name))
                                .show(ui, |ui, position| {
                                    if let Some(position) = position {
                                        ui.label(nformat!("x: {:.2} km", position.x));
                                        ui.label(nformat!("y: {:.2} km", position.y));
                                        ui.label(nformat!("z: {:.2} km", position.z));
                                    } else {
                                        ui.label("x: N/A");
                                        ui.label("y: N/A");
                                        ui.label("z: N/A");
                                    }
                                });
                            IdentedInfo::new("Velocity", sv.map(|sv| sv.velocity))
                                .hover_text(format!("Velocity relative to {}", plot_reference_name))
                                .show(ui, |ui, velocity| {
                                    if let Some(velocity) = velocity {
                                        ui.label(nformat!("x: {:.2} km/s", velocity.x));
                                        ui.label(nformat!("y: {:.2} km/s", velocity.y));
                                        ui.label(nformat!("z: {:.2} km/s", velocity.z));
                                    } else {
                                        ui.label("x: N/A");
                                        ui.label("y: N/A");
                                        ui.label("z: N/A");
                                    }
                                });
                            ui.end_row();
                            IdentedInfo::new("Distance", sv.map(|sv| sv.position.length()))
                                .hover_text(format!("Distance relative to {}", plot_reference_name))
                                .show(ui, |ui, distance| {
                                    if let Some(distance) = distance {
                                        ui.label(nformat!("{:.2} km", distance));
                                    } else {
                                        ui.label("N/A");
                                    }
                                });
                            IdentedInfo::new("Speed", sv.map(|sv| sv.velocity.length()))
                                .hover_text(format!("Speed relative to {}", plot_reference_name))
                                .show(ui, |ui, speed| {
                                    if let Some(speed) = speed {
                                        ui.label(nformat!("{:.2} km/s", speed));
                                    } else {
                                        ui.label("N/A");
                                    }
                                });
                        });

                    ui.add_space(5.0);
                    ui.separator();
                    ui.heading("Plotting");
                    ui.add_space(5.0);

                    ui.checkbox(&mut plot.enabled, "Enabled");

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Max points:");
                        ui.add(egui::Slider::new(&mut plot.max_points, 1..=100_000));
                    });

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        let label = ui
                            .selectable_label(info.plot_start_overwrite.is_some(), "Start:")
                            .on_hover_text("Click to toggle overwrite with current time");
                        if label.clicked() {
                            match info.plot_start_overwrite.take() {
                                Some(PlotOverwrite { previous, .. }) => {
                                    plot.start = previous;
                                }
                                None => {
                                    info.plot_start_overwrite = Some(PlotOverwrite {
                                        previous: plot.start,
                                        kind: OverwriteKind::Current,
                                    });
                                }
                            }
                        }
                        ui.add_enabled(
                            info.plot_start_overwrite.is_none(),
                            ParsedTextEdit::singleline(
                                &mut plot.start,
                                |buf| Epoch::from_str(buf).ok(),
                                Epoch::to_string,
                            ),
                        )
                        .on_disabled_hover_text("Overwritten with current time");
                    });
                    ui.horizontal(|ui| {
                        let label = ui
                            .selectable_label(info.plot_end_overwrite.is_some(), "End:  ")
                            .on_hover_text("Click to toggle overwrite with current time");
                        if label.clicked() {
                            match info.plot_end_overwrite.take() {
                                Some(PlotOverwrite { previous, .. }) => {
                                    plot.end = previous;
                                }
                                None => {
                                    info.plot_end_overwrite = Some(PlotOverwrite {
                                        previous: plot.end,
                                        kind: OverwriteKind::Current,
                                    });
                                }
                            }
                        }
                        ui.add_enabled(
                            info.plot_end_overwrite.is_none(),
                            ParsedTextEdit::singleline(
                                &mut plot.end,
                                |buf| Epoch::from_str(buf).ok(),
                                Epoch::to_string,
                            ),
                        )
                        .on_disabled_hover_text("Overwritten with current time");
                    });

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Resolution:");
                        ui.add(
                            egui::Slider::new(&mut plot.threshold, 0.5..=10.0).logarithmic(true),
                        );

                        ui.label("Color:");
                        let mut color = plot.color.to_linear().to_f32_array();
                        ui.color_edit_button_rgba_unmultiplied(&mut color);
                        plot.color = Color::LinearRgba(LinearRgba::from_f32_array(color));
                    });

                    let Some(flight_plan) = flight_plan.as_mut() else {
                        return;
                    };

                    ui.add_space(5.0);
                    ui.separator();
                    ui.heading("Flight planning");
                    ui.add_space(2.0);
                    ui.spacing_mut().text_edit_width = 240.0;

                    let Ok(trajectory) = query_trajectory.get_mut(entity) else {
                        return;
                    };
                    let trajectory = trajectory.downcast_ref::<DiscreteStates>();

                    let min_time = trajectory.start();
                    let max_time = Epoch::from_tai_duration(Duration::MAX);

                    let mut changed = false;
                    ui.horizontal(|ui| {
                        ui.label("End:").changed();
                        if ui
                            .add(ParsedTextEdit::singleline(
                                &mut flight_plan.end,
                                |buf| {
                                    Epoch::from_str(buf)
                                        .ok()
                                        .map(|t| t.clamp(min_time, max_time).round(precision()))
                                },
                                Epoch::to_string,
                            ))
                            .changed()
                        {
                            changed = true;
                        }
                    });
                    ui.label(format!("({})", flight_plan.end - min_time));

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Max iterations:");
                        if ui
                            .add(
                                egui::Slider::new(&mut flight_plan.max_iterations, 0..=1_000_000)
                                    .logarithmic(true),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });

                    ui.add_space(10.0);

                    ui.horizontal(|ui| {
                        let new_button = ui.button("New burn");
                        if new_button.clicked() {
                            let last_burn = flight_plan.burns.last();
                            let start = last_burn.map(|b| b.end()).unwrap_or(min_time);
                            flight_plan
                                .burns
                                .push(Burn::new(start, plot.reference.unwrap_or(root)));
                            changed = true;
                        }

                        let any_to_delete = !delete.is_empty();
                        let delete_button =
                            ui.add_enabled(any_to_delete, egui::Button::new("Delete selected"));
                        if delete_button.clicked() {
                            flight_plan.burns.retain(|burn| !delete.remove(&burn.id));
                            // Clear in case a burn was already deleted in another way
                            delete.clear();
                            changed = true;
                        }

                        if ui.button("Sort").clicked() {
                            flight_plan.burns.sort_by_key(|burn| burn.start);
                        }
                    });

                    ui.add_space(5.0);

                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            flight_plan
                                .burns
                                .iter_mut()
                                .enumerate()
                                .for_each(|(idx, burn)| {
                                    ui.scope(|ui| {
                                        let id = burn.id;
                                        let burn_changed = Self::show_burn_ui(
                                            ui,
                                            burn,
                                            idx,
                                            root,
                                            entity,
                                            &mut query_hierarchy,
                                            min_time,
                                            || delete.insert(id),
                                        );

                                        if burn_changed {
                                            changed = true;
                                        }
                                    });
                                })
                        });

                    if changed {
                        commands.trigger_targets(FlightPlanChanged, entity);
                    }
                });
        }

        if selected.is_some() && !open {
            **selected = None;
        }
    }

    #[expect(clippy::too_many_arguments)]
    fn show_burn_ui(
        ui: &mut egui::Ui,
        burn: &mut Burn,
        idx: usize,
        root: Entity,
        selected: Entity,
        query_hierarchy: &mut Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        min_time: Epoch,
        mut delete_cb: impl FnMut() -> bool,
    ) -> bool {
        ui.add_space(5.0);

        let mut changed = false;

        let id = burn.id.to_string().into();
        egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
            .show_header(ui, |ui| {
                ui.scope(|ui| {
                    let delete = ui.data(|data| data.get_temp::<bool>(id)).unwrap_or(false);
                    if delete {
                        ui.visuals_mut().selection.bg_fill = egui::Color32::DARK_RED;
                    }
                    if ui.selectable_label(delete, "🗑").clicked() {
                        ui.data_mut(|data| data.insert_temp(id, delete_cb()));
                    }
                });

                if burn.overlaps {
                    ui.visuals_mut().selection.bg_fill = egui::Color32::DARK_RED;
                }

                let mut text = format!("Burn #{}", idx + 1);
                if burn.overlaps {
                    text.push_str(" ⚠ overlaps with another burn");
                }
                if ui.toggle_value(&mut burn.enabled, text).changed() {
                    changed = true;
                }
            })
            .body(|ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size = [240.0, 18.0].into();
                    ui.label("Start time:");
                    ui.add_space(5.0);

                    let mut start = burn.start.to_tai_seconds();
                    if ui
                        .add(
                            egui::DragValue::new(&mut start)
                                .custom_formatter(|value, _| {
                                    Epoch::from_tai_seconds(value).to_string()
                                })
                                .custom_parser(|text| {
                                    Epoch::from_str(text)
                                        .ok()
                                        .filter(|t| *t >= min_time)
                                        .map(|t| t.to_tai_seconds())
                                }),
                        )
                        .changed()
                    {
                        burn.start = Epoch::from_tai_seconds(start).round(precision());
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size = [150.0, 18.0].into();

                    let speed =
                        (burn.duration.max(precision()) / 100).min(Duration::from_seconds(60.0));

                    let mut duration = burn.duration.to_seconds();
                    ui.label("Length:");
                    ui.add_space(22.0);
                    if ui
                        .add(
                            egui::DragValue::new(&mut duration)
                                .speed(speed.to_seconds())
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
                        changed = true;
                    }
                    burn.duration = Duration::from_seconds(duration).round(precision());
                });

                ui.horizontal(|ui| {
                    ui.label("Frame:");
                    ui.add_space(27.0);

                    egui::ComboBox::from_id_source("Frame")
                        .selected_text(burn.frame.to_string())
                        .show_ui(ui, |ui| {
                            for value in BurnFrame::values() {
                                if ui
                                    .selectable_value(&mut burn.frame, value, value.to_string())
                                    .changed()
                                {
                                    changed = true;
                                };
                            }
                        });

                    if matches!(burn.frame, BurnFrame::Frenet) {
                        egui::ComboBox::from_id_source("Reference")
                            .wrap_mode(egui::TextWrapMode::Extend)
                            .selected_text(get_name(
                                burn.reference,
                                query_hierarchy.transmute_lens(),
                            ))
                            .show_ui(ui, |ui| {
                                show_tree(
                                    root,
                                    query_hierarchy,
                                    |_| true,
                                    ui,
                                    |ui, _, (ref_entity, ref_name, _), _| {
                                        ui.horizontal(|ui| {
                                            ui.add_enabled_ui(selected != ref_entity, |ui| {
                                                if ui
                                                    .selectable_value(
                                                        &mut burn.reference,
                                                        ref_entity,
                                                        ref_name.as_str(),
                                                    )
                                                    .on_disabled_hover_text(
                                                        "Cannot use itself as a reference",
                                                    )
                                                    .changed()
                                                {
                                                    changed = true;
                                                }
                                            });
                                            ui.add_space(10.0);
                                        });
                                    },
                                );
                            });
                    }
                });

                ui.add_space(2.0);

                IdentedInfo::new("Acceleration:", &mut burn.acceleration).show(ui, |ui, acc| {
                    ui.spacing_mut().interact_size = [60.0, 18.0].into();
                    let delta_v = |a| a / 1e3 * burn.duration.to_seconds();

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "x");
                        if ui
                            .add(egui::DragValue::new(&mut acc.x).speed(0.01))
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("m/s²");
                        ui.label(format!("({:.3} km/s)", delta_v(acc.x)));
                    });

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "y");
                        if ui
                            .add(egui::DragValue::new(&mut acc.y).speed(0.01))
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("m/s²");
                        ui.label(format!("({:.3} km/s)", delta_v(acc.y)));
                    });

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "z");
                        if ui
                            .add(egui::DragValue::new(&mut acc.z).speed(0.01))
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("m/s²");
                        ui.label(format!("({:.3} km/s)", delta_v(acc.z)));
                    });
                });
            });

        changed
    }
}
