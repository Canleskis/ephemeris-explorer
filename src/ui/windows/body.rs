use crate::{
    camera::{Followed, SetFollowed},
    dynamics::{Bodies, SpacecraftPropagator},
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    hierarchy::{HierarchyQueryExt, OrbitedBy, Orbiting},
    load::SystemRoot,
    prediction::{PredictionContext, Trajectory},
    selection::Selected,
    simulation::SimulationTime,
    ui::{
        get_name, nformat, remove_separation, show_tree, IdentedInfo, PlotConfig, PlotSeparation, PlotSource, PlotSourceOf, WindowsUiSet
    },
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use bevy_file_dialog::prelude::*;
use ephemeris::{BoundedTrajectory, EvaluateTrajectory};
use ftime::{Duration, Epoch};
use std::str::FromStr;

pub struct BodyInfoPlugin;

impl Plugin for BodyInfoPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            BodyInfoWindow::init
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        )
        .add_systems(
            EguiPrimaryContextPass,
            (BodyInfoWindow::show, BodyInfoWindow::persisting)
                .chain()
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

fn ui_coordinate(frame: BurnFrame, ui: &mut egui::Ui, coord: &str) {
    match frame {
        BurnFrame::Relative => match coord {
            "x" => {
                ui.label("Prograde:");
            }
            "y" => {
                ui.label("Radial:");
                ui.add_space(20.0);
            }
            "z" => {
                ui.label("Normal:");
                ui.add_space(12.0);
            }
            _ => {}
        },
        BurnFrame::Inertial => match coord {
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

pub struct ExportShipFile;

#[derive(Component)]
pub struct BodyInfoWindow {
    auto_plot_reference: bool,
    plot_start_overwrite: Option<PlotOverwrite>,
    plot_end_overwrite: Option<PlotOverwrite>,
    delete_body: Option<bool>,
}

impl BodyInfoWindow {
    fn init(
        mut commands: Commands,
        query: Query<(Entity, &PlotSourceOf, Option<&FlightPlan>), Without<Self>>,
        query_plot: Query<&PlotConfig>,
    ) {
        for (entity, source_of, flight_plan) in &query {
            let Some(plot) = source_of.iter().find_map(|e| query_plot.get(e).ok()) else {
                continue;
            };
            commands.entity(entity).insert(Self {
                auto_plot_reference: true,
                plot_start_overwrite: Some(PlotOverwrite {
                    previous: plot.start,
                    kind: OverwriteKind::Current,
                }),
                plot_end_overwrite: None,
                delete_body: flight_plan.map(|_| false),
            });
        }
    }

    fn persisting(
        sim_time: Res<SimulationTime>,
        selected: Res<Selected>,
        mut query: Query<(Entity, &Orbiting, &PlotSourceOf, &mut Self)>,
        mut query_plot: Query<&mut PlotConfig>,
    ) {
        for (entity, parent, source_of, mut info) in &mut query {
            let Some(plot_entity) = source_of.iter().find(|e| query_plot.contains(*e)) else {
                continue;
            };
            let mut plot = query_plot.get_mut(plot_entity).unwrap();

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

    #[expect(clippy::type_complexity)]
    pub fn show(
        mut commands: Commands,
        mut contexts: EguiContexts,
        mut selected: ResMut<Selected>,
        followed: Res<Followed>,
        sim_time: Res<SimulationTime>,
        mut query_hierarchy: Query<(Entity, &Name, Option<&OrbitedBy>)>,
        query_orbited_by: Query<&OrbitedBy>,
        query_trajectory: Query<&Trajectory>,
        mut query: Query<(
            Option<&Orbiting>,
            &PlotSourceOf,
            Option<(&mut FlightPlan, &PredictionContext<SpacecraftPropagator>)>,
            &mut Self,
        )>,
        mut query_plot: Query<(&mut PlotConfig, &PlotSource)>,
        mut query_separation: Query<&mut PlotSeparation>,
        #[expect(unused)] query_source_of: Query<&PlotSourceOf>,
        root: Single<Entity, With<SystemRoot>>,
        mut delete: Local<bevy::platform::collections::hash_set::HashSet<uuid::Uuid>>,
    ) {
        let mut open = selected.is_some();
        if let Some(entity) = **selected {
            let Ok(ctx) = contexts.ctx_mut() else {
                return;
            };

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
                    let Ok((parent, source_of, mut data, mut info)) = query.get_mut(entity) else {
                        return;
                    };

                    let Some(plot_entity) = source_of.iter().find(|e| query_plot.contains(*e))
                    else {
                        return;
                    };
                    let (mut plot, source) = query_plot.get_mut(plot_entity).unwrap();

                    let Some(relative) =
                        plot.get_relative_trajectory(source.entity(), &query_trajectory)
                    else {
                        return;
                    };

                    if let Some(((flight_plan, _), delete)) =
                        data.as_mut().zip(info.delete_body.as_mut())
                    {
                        ui.horizontal(|ui| {
                            if ui.button("Export").clicked() {
                                let mut names = query_hierarchy.transmute_lens();
                                let bytes =
                                    query_trajectory.get(entity).ok().and_then(|trajectory| {
                                        let start = trajectory.start();
                                        let sv = trajectory.state_vector(start)?;
                                        let burns = flight_plan
                                            .burns
                                            .iter()
                                            .map(|burn| burn_to_value(burn, names.query()))
                                            .collect::<Vec<_>>();
                                        Some(
                                            serde_json::to_vec_pretty(&serde_json::json!({
                                                "name": name,
                                                "start": start,
                                                "end": flight_plan.end,
                                                "position": sv.position,
                                                "velocity": sv.velocity,
                                                "burns": burns,
                                            }))
                                            .unwrap(),
                                        )
                                    });

                                if let Some(bytes) = bytes {
                                    commands
                                        .dialog()
                                        .add_filter("JSON", &["json"])
                                        .save_file::<ExportShipFile>(bytes);
                                }
                            }

                            if ui.button("Delete ship").clicked() {
                                *delete = true;
                            }
                        });
                    }

                    if let Some(delete) = &mut info.delete_body {
                        if *delete {
                            egui::Window::new("Confirm deletion")
                                .resizable(false)
                                .collapsible(false)
                                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                                .show(ui.ctx(), |ui| {
                                    ui.label(format!(
                                        r#"Are you sure you want to delete "{name}"?"#
                                    ));
                                    ui.label("All unsaved flight plan data will be lost.");
                                    ui.add_space(10.0);
                                    ui.horizontal(|ui| {
                                        if ui.button("Confirm").clicked() {
                                            if followed.is_some_and(|f| f == entity) {
                                                commands.queue(SetFollowed(parent.map(|p| **p)));
                                            }
                                            commands.entity(entity).despawn();
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
                        .map(|entity| get_name(entity, &query_hierarchy.transmute_lens().query()))
                        .unwrap_or_else(|| "None".to_string());

                    ui.horizontal(|ui| {
                        ui.checkbox(&mut info.auto_plot_reference, "Auto");
                        ui.add_enabled_ui(!info.auto_plot_reference, |ui| {
                            ui.label("Reference:");
                            egui::ComboBox::from_id_salt("Reference")
                                .wrap_mode(egui::TextWrapMode::Extend)
                                .selected_text(plot_reference_name)
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

                                            children.map(|c| query_hierarchy.iter_many(c))
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
                            let sv = relative.state_vector(sim_time.current());

                            IdentedInfo::new("Position", sv.map(|sv| sv.position))
                                .hover_text(format!("Position relative to {plot_reference_name}"))
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
                                .hover_text(format!("Velocity relative to {plot_reference_name}"))
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
                                .hover_text(format!("Distance relative to {plot_reference_name}"))
                                .show(ui, |ui, distance| {
                                    if let Some(distance) = distance {
                                        ui.label(nformat!("{:.2} km", distance));
                                    } else {
                                        ui.label("N/A");
                                    }
                                });
                            IdentedInfo::new("Speed", sv.map(|sv| sv.velocity.length()))
                                .hover_text(format!("Speed relative to {plot_reference_name}"))
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
                    ui.add_space(4.0);

                    ui.checkbox(&mut plot.enabled, "Enabled");

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Max points:");
                        ui.add(egui::Slider::new(&mut plot.max_points, 1..=100_000));
                    });

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

                        ui.add_enabled_ui(info.plot_start_overwrite.is_none(), |ui| {
                            ui.add(
                                egui::DragValue::new(plot.start.mut_offset().mut_seconds())
                                    .speed(Duration::from_hours(1.0).as_seconds())
                                    .custom_formatter(epoch_formatter)
                                    .custom_parser(epoch_parser),
                            )
                        })
                        .response
                        .on_disabled_hover_text("Overwritten with current time");
                    });
                    ui.add_space(5.0);
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
                        ui.add_enabled_ui(info.plot_end_overwrite.is_none(), |ui| {
                            ui.add(
                                egui::DragValue::new(plot.end.mut_offset().mut_seconds())
                                    .speed(Duration::from_hours(1.0).as_seconds())
                                    .custom_formatter(epoch_formatter)
                                    .custom_parser(epoch_parser),
                            )
                        })
                        .response
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

                    ui.add_space(5.0);

                    // Reborrow to drop previous mutable borrow of the query.
                    let (plot, _) = query_plot.get(plot_entity).unwrap();

                    if let Ok(target) = query_separation.get_mut(plot_entity).as_deref_mut() {
                        let unknown = Name::new("Unknown");
                        let name = |e| {
                            query_hierarchy
                                .get(e)
                                .map(|(_, name, _)| name)
                                .unwrap_or(&unknown)
                        };

                        ui.horizontal(|ui| {
                            ui.label("Separation marker:");
                            egui::ComboBox::from_id_salt("Separation")
                                .wrap_mode(egui::TextWrapMode::Extend)
                                .selected_text(name(target.entity()).as_str())
                                .show_ui(ui, |ui| {
                                    show_tree(
                                        ui,
                                        PlotSeparation::Trajectory(*root),
                                        |&item| item,
                                        |_, _| true,
                                        |ui, _, separation, _| {
                                            let ref_entity = separation.entity();
                                            let color = query_plot
                                                .get(ref_entity)
                                                .ok()
                                                .map(|(plot, _)| {
                                                    let [r, g, b, a] =
                                                        plot.color.to_srgba().to_u8_array();
                                                    egui::Rgba::from_srgba_premultiplied(r, g, b, a)
                                                        .into()
                                                })
                                                .unwrap_or(
                                                    ui.visuals()
                                                        .widgets
                                                        .noninteractive
                                                        .fg_stroke
                                                        .color,
                                                );

                                            let name = name(ref_entity);
                                            let name = name
                                                .find("Plot")
                                                .map(|i| {
                                                    ui.spacing_mut().interact_size.y = 5.0;
                                                    egui::RichText::new(&name[i..])
                                                        .size(10.5)
                                                        .color(color)
                                                })
                                                .unwrap_or_else(|| egui::RichText::new(name));

                                            let enabled = entity != ref_entity
                                                && !source_of.contains(&ref_entity);
                                            ui.add_enabled_ui(enabled, |ui| {
                                                ui.vertical(|ui| {
                                                    ui.selectable_value(target, separation, name);
                                                });

                                                ui.add_space(20.0);
                                            })
                                            .response
                                            .on_disabled_hover_text(
                                                "Cannot use itself as a target",
                                            );

                                            // Eventually, when we can have multiple plotted trajectories for a body

                                            // let plots =
                                            //     query_source_of.get(ref_entity).ok().map(|c| {
                                            //         c.iter()
                                            //             .filter(|e| query_plot.contains(**e))
                                            //             .map(|e| SeparationPlot::Plot(*e))
                                            //     });

                                            // let children = query_children.get(ref_entity).ok().map(|c| {
                                            //     c.iter().map(|e| SeparationPlot::Trajectory(*e))
                                            // });

                                            // Some(
                                            //     plots
                                            //         .into_iter()
                                            //         .flatten()
                                            //         .chain(children.into_iter().flatten()),
                                            // )

                                            query_orbited_by.get(ref_entity).ok().map(|c| {
                                                c.iter().map(|e| PlotSeparation::Trajectory(*e))
                                            })
                                        },
                                    );
                                });

                            if ui.button("❌").clicked() {
                                commands.entity(plot_entity).queue(remove_separation);
                            }
                        });
                    } else if ui.button("Set separation marker").clicked() {
                        commands
                            .entity(plot_entity)
                            .insert(PlotSeparation::Trajectory(plot.reference.unwrap_or(*root)));
                    }

                    let Some((flight_plan, prediction)) = data.as_mut() else {
                        return;
                    };

                    ui.add_space(5.0);
                    ui.separator();
                    ui.heading("Flight planning");
                    ui.add_space(2.0);

                    ui.add_space(2.0);

                    let mut changed = false;

                    let Ok(trajectory) = query_trajectory.get(entity) else {
                        return;
                    };
                    let min_time = trajectory.start();
                    let max_time = Epoch::from_offset(Duration::MAX);

                    ui.horizontal(|ui| {
                        ui.label("Max iterations:");
                        let speed = 1e-1f64.max(flight_plan.max_iterations as f64 * 1e-1);
                        if ui
                            .add(
                                egui::DragValue::new(&mut flight_plan.max_iterations)
                                    .speed(speed)
                                    .range(0..=1_000_000),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });

                    ui.add_space(2.0);

                    ui.horizontal(|ui| {
                        ui.spacing_mut().interact_size.x = 240.0;
                        ui.label("End:");

                        if ui
                            .add(
                                egui::DragValue::new(flight_plan.end.mut_offset().mut_seconds())
                                    .speed(Duration::from_hours(1.0).as_seconds())
                                    .range(
                                        min_time.as_offset_seconds()..=max_time.as_offset_seconds(),
                                    )
                                    .custom_formatter(epoch_formatter)
                                    .custom_parser(epoch_parser),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });
                    ui.label(format!("({})", flight_plan.end - min_time));

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        let new_button = ui.button("New burn");
                        if new_button.clicked() {
                            let last_burn = flight_plan.burns.last();
                            let start = last_burn.map(|b| b.end()).unwrap_or(min_time);
                            flight_plan.burns.push(Burn::new(
                                start,
                                match plot.reference {
                                    Some(e) if e != *root => BurnFrame::Relative,
                                    _ => BurnFrame::Inertial,
                                },
                                plot.reference.unwrap_or(*root),
                            ));
                            changed = true;
                        }

                        let any_to_delete = !delete.is_empty();
                        let delete_button =
                            ui.add_enabled(any_to_delete, egui::Button::new("Delete selected"));
                        if delete_button.clicked() {
                            flight_plan.burns.retain(|burn| !delete.remove(&burn.id));
                            delete.clear();
                            changed = true;
                        }

                        if ui.button("Sort").clicked() {
                            flight_plan.burns.sort_by_key(|burn| burn.start);
                        }
                    });

                    ui.add_space(5.0);

                    ui.label(format!(
                        "Total delta-v: {:.3} km/s",
                        flight_plan.total_delta_v() / 1e3
                    ));

                    ui.add_space(2.0);

                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .min_scrolled_height(214.0)
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
                                            *root,
                                            entity,
                                            prediction.propagator.context(),
                                            &mut query_hierarchy,
                                            &query_orbited_by,
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

    fn show_burn_ui(
        ui: &mut egui::Ui,
        burn: &mut Burn,
        idx: usize,
        root: Entity,
        selected: Entity,
        context: &Bodies,
        query_hierarchy: &mut Query<(Entity, &Name, Option<&OrbitedBy>)>,
        query_orbited_by: &Query<&OrbitedBy>,
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
                if ui.selectable_label(burn.enabled, text).clicked() {
                    burn.enabled = !burn.enabled;
                    changed = true;
                }
            })
            .body(|ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size.x = 240.0;
                    ui.label("Start time:");
                    ui.add_space(5.0);

                    if ui
                        .add(
                            egui::DragValue::new(burn.start.mut_offset().mut_seconds())
                                .range(min_time.as_offset_seconds()..=f64::INFINITY)
                                .custom_formatter(epoch_formatter)
                                .custom_parser(epoch_parser),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.spacing_mut().interact_size.x = 150.0;

                    let speed = (burn.duration.max(Duration::from_seconds(0.1)) * 1e-3)
                        .min(Duration::from_seconds(60.0));

                    ui.label("Length:");
                    ui.add_space(22.0);
                    if ui
                        .add(
                            egui::DragValue::new(burn.duration.mut_seconds())
                                .speed(speed.as_seconds())
                                .range(0.0..=f64::INFINITY)
                                .custom_formatter(duration_formatter)
                                .custom_parser(duration_parser),
                        )
                        .changed()
                    {
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 3.0;
                    ui.label("Frame:");
                    ui.add_space(32.0);

                    egui::ComboBox::from_id_salt("Frame")
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

                    if matches!(burn.frame, BurnFrame::Relative) {
                        egui::ComboBox::from_id_salt("Reference")
                            .wrap_mode(egui::TextWrapMode::Extend)
                            .selected_text(get_name(
                                burn.reference,
                                &query_hierarchy.transmute_lens().query(),
                            ))
                            .show_ui(ui, |ui| {
                                let first_in_context = query_orbited_by
                                    .iter_orbiting_descendants(root)
                                    .find(|e| context.0.contains_key(e));

                                let Some(first_in_context) = first_in_context else {
                                    return;
                                };

                                show_tree(
                                    ui,
                                    query_hierarchy.get(first_in_context).unwrap(),
                                    |&(e, name, _)| (e, name),
                                    |_, _| true,
                                    |ui, _, (ref_entity, ref_name, children), _| {
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

                                        children.map(|c| {
                                            query_hierarchy
                                                .iter_many(c)
                                                .filter(|(e, ..)| context.0.contains_key(e))
                                        })
                                    },
                                );
                            });
                    }
                });

                ui.add_space(2.0);

                IdentedInfo::new("Acceleration:", &mut burn.acceleration).show(ui, |ui, acc| {
                    ui.spacing_mut().interact_size.x = 60.0;

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "x");
                        if ui
                            .add(egui::DragValue::new(&mut acc.x).speed(0.01))
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("m/s²");
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
                    });
                });

                ui.add_space(2.0);

                ui.label(format!("Delta-v: {:.3} km/s", burn.delta_v() / 1e3));
            });

        changed
    }
}

fn burn_to_value(burn: &Burn, query: Query<&Name>) -> serde_json::Value {
    let mut json = serde_json::json!({
        "start": burn.start,
        "duration": burn.duration,
        "acceleration": burn.acceleration,
    });

    if let BurnFrame::Relative = burn.frame {
        json["reference"] = serde_json::json!(query.get(burn.reference).unwrap().to_string());
    }

    json
}

fn duration_formatter(value: f64, _: std::ops::RangeInclusive<usize>) -> String {
    Duration::from_seconds(value).to_string()
}

fn duration_parser(text: &str) -> Option<f64> {
    Duration::from_str(text).ok().map(|d| d.as_seconds())
}

fn epoch_formatter(value: f64, _: std::ops::RangeInclusive<usize>) -> String {
    Epoch::from_offset(Duration::from_seconds(value)).to_string()
}

fn epoch_parser(text: &str) -> Option<f64> {
    Epoch::from_str(text).ok().map(|t| t.as_offset_seconds())
}
