use crate::{
    MainState,
    analysis::{OrbitTarget, Primary, Satellites},
    camera::{Followed, SetFollowed},
    dynamics::{Bodies, SpacecraftTrajectory, Trajectory},
    flight_plan::{
        Burn, BurnFrame, FlightPlan, FlightPlanChanged, GlobalBodies, IntegrationMethod,
    },
    load::{BurnDe, Ship, SystemRoot},
    prediction::PredictionPropagator,
    ui::{
        IdentedInfo, Velocity, WindowsUiSet, acceleration_formatter, acceleration_parser,
        duration_formatter, duration_parser, epoch_formatter, epoch_parser, get_name,
        length_formatter, length_parser, show_tree, try_get_name,
    },
};

use bevy::{platform::collections::hash_map::Entry, prelude::*};
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use bevy_file_dialog::prelude::*;
use ephemeris::{BoundedTrajectory, EvaluateTrajectory};
use ftime::{Duration, Epoch};

pub struct FlightPlanWindowPlugin;

impl Plugin for FlightPlanWindowPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(add_fp_window_state).add_systems(
            EguiPrimaryContextPass,
            FlightPlanWindow::show
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

pub struct ExportShipFile;

#[derive(Clone, Default, Component)]
pub struct FlightPlanWindow {
    pub open: bool,
    pub context_open: bool,
    pub context_all: bool,
    pub delete: bool,
    pub delete_burns: bevy::platform::collections::hash_set::HashSet<uuid::Uuid>,
}

fn add_fp_window_state(trigger: On<Add, FlightPlan>, mut commands: Commands) {
    commands
        .entity(trigger.entity)
        .insert(FlightPlanWindow::default());
}

impl FlightPlanWindow {
    #[expect(clippy::type_complexity)]
    pub fn show(
        mut commands: Commands,
        mut contexts: EguiContexts,
        followed: Res<Followed>,
        query_hierarchy: Query<(Entity, &Name, Option<&Satellites>)>,
        query_satellites: Query<&Satellites>,
        mut query: Query<(
            Entity,
            &Trajectory,
            &Primary,
            &mut FlightPlan,
            &PredictionPropagator<SpacecraftTrajectory>,
            &mut OrbitTarget,
            &mut Self,
        )>,
        global: Res<GlobalBodies>,
        root: Single<Entity, With<SystemRoot>>,
    ) {
        for (entity, trajectory, parent, mut fp, propagator, mut target, mut window) in &mut query {
            let Ok(ctx) = contexts.ctx_mut() else { return };

            let Ok((_, name, _)) = query_hierarchy.get(entity) else {
                return;
            };

            let mut window_open = window.open;
            egui::Window::new(format!("{name} flight plan"))
                .open(&mut window_open)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Export").clicked() {
                            let start = trajectory.start();
                            let sv = trajectory.state_vector(start).unwrap();
                            let bytes = Some(
                                serde_json::to_vec_pretty(&Ship::new(
                                    name.to_string(),
                                    fp.method,
                                    fp.parameters.tol.position,
                                    start,
                                    fp.end,
                                    sv.position,
                                    sv.velocity,
                                    fp.burns
                                        .values()
                                        .map(|burn| BurnDe {
                                            start: burn.start,
                                            duration: burn.duration,
                                            acceleration: burn.acceleration,
                                            reference: try_get_name(
                                                burn.reference,
                                                &query_hierarchy,
                                            ),
                                        })
                                        .collect(),
                                    if global.bodies().keys().any(|e| !fp.context.contains(*e)) {
                                        Some(
                                            fp.context
                                                .keys()
                                                .map(|e| get_name(*e, &query_hierarchy))
                                                .collect(),
                                        )
                                    } else {
                                        None
                                    },
                                ))
                                .unwrap(),
                            );

                            if let Some(bytes) = bytes {
                                commands
                                    .dialog()
                                    .add_filter("JSON", &["json"])
                                    .save_file::<ExportShipFile>(bytes);
                            }
                        }

                        if ui.button("Delete ship").clicked() {
                            window.delete = true;
                        }
                    });

                    egui::Window::new("Confirm deletion")
                        .id(format!("{name} deletion").into())
                        .resizable(false)
                        .collapsible(false)
                        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                        .open(&mut window.delete)
                        .show(ui.ctx(), |ui| {
                            ui.label(format!(r#"Are you sure you want to delete "{name}"?"#));
                            ui.label("All unsaved flight plan data will be lost.");
                            ui.add_space(10.0);
                            ui.horizontal(|ui| {
                                if ui.button("Confirm").clicked() {
                                    if followed.is_some_and(|f| f == entity) {
                                        commands.queue(SetFollowed(Some(**parent)));
                                    }
                                    commands.entity(entity).despawn();
                                }
                                if ui.button("Cancel").clicked() {
                                    ui.close_kind(egui::UiKind::Window);
                                }
                            });
                        });

                    ui.separator();

                    let mut plot_target = target.entity();
                    ui.horizontal(|ui| {
                        if let Some(target) = &mut plot_target {
                            let name = query_hierarchy
                                .get(*target)
                                .map(|(_, name, _)| name.as_str())
                                .unwrap_or("Unknown");

                            ui.label("Target:");
                            egui::ComboBox::from_id_salt("Target")
                                .wrap_mode(egui::TextWrapMode::Extend)
                                .selected_text(name)
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
                                                        target,
                                                        ref_entity,
                                                        ref_name.as_str(),
                                                    )
                                                    .on_disabled_hover_text("Cannot target itself");
                                                });
                                                ui.add_space(10.0);
                                            });

                                            children.map(|c| query_hierarchy.iter_many(c))
                                        },
                                    );
                                });

                            if ui.button("❌").clicked() {
                                plot_target = None;
                            }
                        } else if ui.button("Set target").clicked() {
                            plot_target = Some(**parent);
                        }

                        if plot_target != target.entity() {
                            *target = OrbitTarget(plot_target);
                        }
                    });

                    ui.separator();

                    ui.heading("Flight planning");
                    ui.add_space(4.0);

                    ui.horizontal(|ui| {
                        ui.label("Flight plan context:");
                        if ui
                            .button(if window.context_open { "Hide" } else { "Show" })
                            .clicked()
                        {
                            window.context_open = true;
                        }
                    });

                    let mut fp_changed = false;

                    let mut window_context_open = window.context_open;
                    egui::Window::new(format!("{name} flight plan context"))
                        .open(&mut window_context_open)
                        .show(ctx, |ui| {
                            if ui.checkbox(&mut window.context_all, "All").changed()
                                && window.context_all
                            {
                                fp.context = global.bodies().clone();
                            }

                            ui.separator();

                            ui.add_enabled_ui(!window.context_all, |ui| {
                                egui::ScrollArea::vertical()
                                    .auto_shrink([false, true])
                                    .show(ui, |ui| {
                                        ui.spacing_mut().item_spacing = [1.0, 3.0].into();
                                        show_tree(
                                            ui,
                                            query_hierarchy.get(*root).unwrap(),
                                            |&(e, name, _)| (e, name.clone()),
                                            |i, _| i < 2,
                                            |ui, state, (entity, name, children), _| {
                                                let children = || {
                                                    children.map(|c| {
                                                        c.iter().filter(|e| {
                                                            global.bodies().contains(*e)
                                                        })
                                                    })
                                                };

                                                let has_children =
                                                    children().is_some_and(|c| c.count() != 0);

                                                let mut ui_builder = egui::UiBuilder::new();
                                                if !has_children {
                                                    ui_builder = ui_builder.invisible();
                                                }

                                                ui.scope_builder(ui_builder, |ui| {
                                                    state.show_toggle_button(
                                                        ui,
                                                        egui::collapsing_header::paint_default_icon,
                                                    );

                                                    let selected_count = children()
                                                        .into_iter()
                                                        .flatten()
                                                        .filter(|entity| {
                                                            fp.context.contains(*entity)
                                                        })
                                                        .count();

                                                    let all_selected =
                                                        children().is_some_and(|c| {
                                                            c.count() != selected_count
                                                        });
                                                    let any_selected = selected_count > 0;

                                                    if all_selected {
                                                        let color =
                                                            &mut ui.visuals_mut().selection.bg_fill;
                                                        *color = color.linear_multiply(0.4);
                                                    }

                                                    if ui
                                                        .selectable_label(any_selected, "A")
                                                        .clicked()
                                                    {
                                                        for entity in
                                                            children().into_iter().flatten()
                                                        {
                                                            if any_selected {
                                                                fp.context.remove(entity);
                                                            } else {
                                                                fp.context.insert(
                                                                    entity,
                                                                    global
                                                                        .bodies()
                                                                        .get(entity)
                                                                        .unwrap()
                                                                        .clone(),
                                                                );
                                                            }
                                                        }

                                                        fp_changed = true
                                                    }
                                                });

                                                if global.bodies().contains(entity) {
                                                    let checked = fp.context.contains(entity);
                                                    if ui
                                                        .selectable_label(checked, name.as_str())
                                                        .clicked()
                                                    {
                                                        match fp.context.0.entry(entity) {
                                                            Entry::Occupied(entry) => {
                                                                entry.remove();
                                                            }
                                                            Entry::Vacant(entry) => {
                                                                entry.insert(
                                                                    global
                                                                        .bodies()
                                                                        .get(entity)
                                                                        .unwrap()
                                                                        .clone(),
                                                                );
                                                            }
                                                        }

                                                        fp_changed = true
                                                    }
                                                } else {
                                                    ui.style_mut().interaction.selectable_labels =
                                                        false;
                                                    ui.add_space(ui.spacing().button_padding.x);
                                                    ui.label(name.as_str());
                                                }

                                                ui.add_space(10.0);

                                                children().map(|c| query_hierarchy.iter_many(c))
                                            },
                                        );
                                    });
                            });
                        });
                    window.context_open = window_context_open;

                    let min_time = trajectory.start();
                    let max_time = Epoch::MAX;

                    ui.horizontal(|ui| {
                        ui.label("Integrator:");

                        egui::ComboBox::from_id_salt("Integrator")
                            .selected_text(fp.method.to_string())
                            .show_ui(ui, |ui| {
                                for &value in IntegrationMethod::values() {
                                    if ui
                                        .selectable_value(&mut fp.method, value, value.to_string())
                                        .changed()
                                    {
                                        fp_changed = true;
                                    };
                                }
                            });
                    });

                    ui.horizontal(|ui| {
                        ui.label("Max iterations:");
                        let speed = 1e-1f64.max(fp.parameters.n_max as f64 * 1e-1);
                        if ui
                            .add(
                                egui::DragValue::new(&mut fp.parameters.n_max)
                                    .speed(speed)
                                    .range(0..=1_000_000),
                            )
                            .changed()
                        {
                            fp_changed = true;
                        }

                        ui.label("Tolerance:");
                        let speed = 1e-12f64.max(fp.parameters.tol.position * 1e-2);

                        if ui
                            .add(
                                egui::DragValue::new(&mut fp.parameters.tol.position)
                                    .speed(speed)
                                    .range(1e-9..=1e3)
                                    .custom_formatter(length_formatter)
                                    .custom_parser(length_parser),
                            )
                            .changed()
                        {
                            fp.parameters.tol.velocity = fp.parameters.tol.position;
                            fp_changed = true;
                        }
                    });

                    ui.add_space(2.0);

                    ui.horizontal(|ui| {
                        ui.spacing_mut().interact_size.x = 220.0;
                        ui.label("End:");
                        if ui
                            .add(
                                egui::DragValue::new(fp.end.mut_offset().mut_seconds())
                                    .speed(Duration::from_hours(1.0).as_seconds())
                                    .range(
                                        min_time.as_offset_seconds()..=max_time.as_offset_seconds(),
                                    )
                                    .custom_formatter(epoch_formatter)
                                    .custom_parser(epoch_parser),
                            )
                            .changed()
                        {
                            fp_changed = true;
                        }
                    });
                    ui.label(format!("({})", fp.end - min_time));

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        let new_button = ui.button("New burn");
                        if new_button.clicked() {
                            let last_burn = fp.burns.last();
                            let start = last_burn.map(|(_, b)| b.end()).unwrap_or(min_time);
                            fp.burns.insert(
                                uuid::Uuid::new_v4(),
                                Burn::new(
                                    start,
                                    match **parent {
                                        parent if parent != *root => BurnFrame::Relative,
                                        _ => BurnFrame::Inertial,
                                    },
                                    **parent,
                                ),
                            );
                            fp_changed = true;
                        }

                        let any_to_delete = !window.delete_burns.is_empty();
                        let delete_button =
                            ui.add_enabled(any_to_delete, egui::Button::new("Delete selected"));
                        if delete_button.clicked() {
                            fp.burns.retain(|id, _| !window.delete_burns.remove(id));
                            window.delete_burns.clear();
                            fp_changed = true;
                        }

                        if ui.button("Sort").clicked() {
                            fp.burns.sort_by_key(|_, burn| burn.start);
                        }
                    });

                    ui.add_space(5.0);

                    ui.label(format!(
                        "Total delta-v: {:.3}",
                        Velocity::kps(fp.total_delta_v())
                    ));

                    ui.add_space(2.0);

                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .min_scrolled_height(214.0)
                        .show(ui, |ui| {
                            fp.burns
                                .iter_mut()
                                .enumerate()
                                .for_each(|(idx, (&id, burn))| {
                                    ui.scope(|ui| {
                                        let burn_changed = Self::show_burn_ui(
                                            ui,
                                            id,
                                            burn,
                                            idx,
                                            *root,
                                            entity,
                                            propagator.context(),
                                            &query_hierarchy,
                                            &query_satellites,
                                            min_time,
                                            || window.delete_burns.insert(id),
                                        );

                                        if burn_changed {
                                            fp_changed = true;
                                        }
                                    });
                                })
                        });

                    if fp_changed {
                        commands.trigger(FlightPlanChanged(entity));
                    }
                });

            window.open = window_open;
        }
    }

    fn show_burn_ui(
        ui: &mut egui::Ui,
        id: uuid::Uuid,
        burn: &mut Burn,
        idx: usize,
        root: Entity,
        selected: Entity,
        context: &Bodies,
        query_hierarchy: &Query<(Entity, &Name, Option<&Satellites>)>,
        query_orbited_by: &Query<&Satellites>,
        min_time: Epoch,
        mut delete_cb: impl FnMut() -> bool,
    ) -> bool {
        ui.add_space(5.0);

        let mut changed = false;

        let id = id.to_string().into();
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
                    ui.spacing_mut().interact_size.x = 220.0;
                    ui.label("Start time:");
                    ui.add_space(3.0);

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
                    ui.add_space(31.0);

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
                            .selected_text(get_name(burn.reference, query_hierarchy))
                            .show_ui(ui, |ui| {
                                let first_in_context = query_orbited_by
                                    .iter_descendants(root)
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

                IdentedInfo::new("Acceleration:", &mut burn.acceleration).show(ui, |ui, acc| {
                    ui.spacing_mut().interact_size.x = 100.0;

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "x");
                        if ui
                            .add(
                                egui::DragValue::new(&mut acc.x)
                                    .speed(1e-4)
                                    .custom_formatter(acceleration_formatter)
                                    .custom_parser(acceleration_parser),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "y");
                        if ui
                            .add(
                                egui::DragValue::new(&mut acc.y)
                                    .speed(1e-4)
                                    .custom_formatter(acceleration_formatter)
                                    .custom_parser(acceleration_parser),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });

                    ui.horizontal(|ui| {
                        ui_coordinate(burn.frame, ui, "z");
                        if ui
                            .add(
                                egui::DragValue::new(&mut acc.z)
                                    .speed(1e-4)
                                    .custom_formatter(acceleration_formatter)
                                    .custom_parser(acceleration_parser),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });
                });

                ui.add_space(2.0);

                ui.label(format!("Delta-v: {:.3}", Velocity::kps(burn.delta_v())));
            });

        changed
    }
}
