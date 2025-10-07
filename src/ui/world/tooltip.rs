use crate::{
    dynamics::SpacecraftPropagator,
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    load::SystemRoot,
    prediction::{Predicting, Trajectory},
    simulation::SimulationTime,
    ui::{
        nformat, ManoeuvreHitPoint, MarkerGizmoConfigGroup, PlotPoints, SourceOf,
        TrajectoryHitPoint, TrajectoryPlot, WorldUiInteraction, WorldUiSet, MANOEUVRE_SIZE,
        PICK_THRESHOLD,
    },
    MainState,
};

use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use ephemeris::{EvaluateTrajectory, RelativeTrajectory};
use ftime::{Duration, Epoch};

#[derive(Clone, Copy, Debug, Default, Resource)]
pub struct ManoeuvreDraggingOptions {
    pub enabled: bool,
}

pub struct TooltipPlugin;

impl Plugin for TooltipPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ManoeuvreDraggingOptions>()
            .init_resource::<FocusedManoeuvreHitPoint>()
            .add_systems(
                PostUpdate,
                (
                    show_separation.after(crate::ui::compute_plot_points_parallel),
                    (
                        focus_manoeuvre_hit,
                        manoeuvre_dragging,
                        show_cursor_crossings,
                    )
                        .chain()
                        .before(bevy_egui::EguiPostUpdateSet::EndPass)
                        .after(crate::ui::trajectory_picking),
                )
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

#[derive(Resource, Clone, Copy, Default)]
pub struct FocusedManoeuvreHitPoint {
    pub point: Option<ManoeuvreHitPoint>,
    pub locked: bool,
}

fn focus_manoeuvre_hit(
    mut manoeuvre_events: EventReader<ManoeuvreHitPoint>,
    mut focused: ResMut<FocusedManoeuvreHitPoint>,
) {
    if !focused.locked {
        focused.point = manoeuvre_events
            .read()
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
            .copied();
    }
}

// A bit janky when there are many crossings, but still useful.
fn manoeuvre_dragging(
    mut commands: Commands,
    drag_opts: Res<ManoeuvreDraggingOptions>,
    ray_map: Res<RayMap>,
    perspective: Single<&PerspectiveProjection, With<Camera>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut world_ui: ResMut<WorldUiInteraction>,
    mut focused_manoeuvre_hit: ResMut<FocusedManoeuvreHitPoint>,
    query_plot: Query<&PlotPoints>,
    mut query_flight_plan: Query<&mut FlightPlan>,
) {
    if !drag_opts.enabled {
        return;
    }

    if focused_manoeuvre_hit.point.is_some() && mouse.just_pressed(MouseButton::Left) {
        focused_manoeuvre_hit.locked = true;
        world_ui.using_pointer = true;
    }

    if !focused_manoeuvre_hit.locked {
        return;
    }

    if mouse.just_released(MouseButton::Left) {
        focused_manoeuvre_hit.locked = false;
        world_ui.using_pointer = false;
    }

    if let Some(hit) = focused_manoeuvre_hit.point {
        let Ok(points) = query_plot.get(hit.plot_entity) else {
            return;
        };
        let Some((_, ray)) = ray_map.iter().next() else {
            return;
        };
        let ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

        if let Some((mut time, _, _)) = points
            .ray_distances(&ray)
            .filter(|hit| hit.1 < hit.2 * perspective.fov * MANOEUVRE_SIZE)
            .min_by(|a, b| a.0.cmp(&b.0))
        {
            let Ok(mut flight_plan) = query_flight_plan.get_mut(hit.flight_plan_entity) else {
                return;
            };

            if let Some(burn) = flight_plan.burns.iter_mut().find(|b| b.id == hit.id) {
                time = time.round(Duration::from_seconds(1.0));
                if burn.start != time {
                    burn.start = time;
                    commands.trigger_targets(FlightPlanChanged, hit.flight_plan_entity);
                }
            }
        }
    };
}

fn show_cursor_crossings(
    mut contexts: EguiContexts,
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
    mut commands: Commands,
    input: Res<ButtonInput<MouseButton>>,
    mut sim_time: ResMut<SimulationTime>,
    drag_opts: Res<ManoeuvreDraggingOptions>,
    mut traj_events: EventReader<TrajectoryHitPoint>,
    focused_manoeuvre_hit: Res<FocusedManoeuvreHitPoint>,
    query_trajectory: Query<&Trajectory>,
    query_points: Query<(&Name, &PlotPoints, &TrajectoryPlot)>,
    mut query_flight_plan: Query<&mut FlightPlan>,
    camera: Single<(&GlobalTransform, &Camera, &PerspectiveProjection)>,
    mut buffer: Local<Vec<TrajectoryHitPoint>>,
    mut persisted: Local<Vec<TrajectoryHitPoint>>,
    root: Single<Entity, With<SystemRoot>>,
) {
    let (camera_transform, camera, perspective) = *camera;
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    buffer.clear();

    if let Some(hit) = focused_manoeuvre_hit.point {
        (|| {
            let flight_plan = query_flight_plan.get_mut(hit.flight_plan_entity).ok()?;
            let (burn_idx, burn) = flight_plan
                .burns
                .iter()
                .enumerate()
                .find(|(_, burn)| burn.id == hit.id)?;
            let (name, points, plot) = query_points.get(hit.plot_entity).ok()?;

            let position = points.evaluate(burn.start)?;
            let vp_position = camera.world_to_viewport(camera_transform, position).ok()?;
            let window_position = vp_position + Vec2::new(20.0, -20.0);

            egui::Window::new("Manoeuvre")
                .collapsible(false)
                .resizable(false)
                .fade_in(false)
                .fade_out(false)
                .title_bar(false)
                .constrain(false)
                .fixed_size([250.0, 200.0])
                .fixed_pos(window_position.to_array())
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                            ui.style_mut().interaction.selectable_labels = false;
                            ui.label(format!("{} — Burn #{}", name.as_str(), burn_idx + 1));
                        });
                    });
                    ui.separator();
                    ui.label(burn.start.to_string());
                });

            if drag_opts.enabled {
                let direction = camera_transform.translation() - position;
                let size = direction.length() * perspective.fov * PICK_THRESHOLD;
                gizmos.circle(
                    Transform::from_translation(position)
                        .looking_to(direction, Vec3::Y)
                        .to_isometry(),
                    size,
                    plot.color,
                );
            }

            Some(())
        })();
    } else {
        // Only hits close to the cursor are used (small separation).
        buffer.extend(
            traj_events
                .read()
                .filter(|hit| hit.separation < hit.distance * perspective.fov * PICK_THRESHOLD)
                .cloned(),
        );
    }

    if let Some(min_hit) = buffer
        .iter()
        .min_by(|a, b| a.separation.total_cmp(&b.separation))
        .cloned()
    {
        let threshold = 0.01;
        let min_distance = min_hit.distance * (1.0 - threshold);
        let max_distance = min_hit.distance * (1.0 + threshold);
        buffer.retain(|hit| {
            // Filter out hits from other entities than the closest hit and keep hits that are
            //relatively close to it.
            hit.entity == min_hit.entity && (min_distance..=max_distance).contains(&hit.distance)
        });
        if input.just_pressed(MouseButton::Left) && !ctx.is_pointer_over_area() {
            persisted.clear();
            persisted.extend(buffer.iter().cloned());
        }
    }

    for (i, hits) in [&mut *buffer, &mut *persisted].into_iter().enumerate() {
        if hits.is_empty() {
            continue;
        }
        let is_pinned = i == 1;
        // All hits are on the same entity.
        let entity = hits[0].entity;
        let Ok((name, points, plot)) = query_points.get(entity) else {
            continue;
        };

        let Some(relative) = plot.get_relative_trajectory(&query_trajectory) else {
            continue;
        };

        let window_data = hits
            .iter()
            .filter_map(|hit| {
                let position = points.evaluate(hit.time)?;
                let vp_position = camera.world_to_viewport(camera_transform, position).ok()?;
                Some((vp_position, position, hit.time))
            })
            .collect::<Vec<_>>();
        if window_data.is_empty() {
            continue;
        }
        let window_position = window_data
            .iter()
            .max_by(|a, b| a.0.x.total_cmp(&b.0.x))
            .unwrap()
            .0
            + Vec2::new(20.0, -20.0);

        egui::Window::new(name.as_str())
            .collapsible(false)
            .resizable(false)
            .fade_in(false)
            .fade_out(false)
            .title_bar(false)
            .constrain(false)
            .fixed_size([250.0, 200.0])
            .id(egui::Id::new(i.to_string() + "#intersections"))
            .fixed_pos(window_position.to_array())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.style_mut().interaction.selectable_labels = false;
                        let suf = if window_data.len() == 1 { "" } else { "s" };
                        ui.label(format!(
                            "{} — {} crossing{}",
                            name.as_str(),
                            window_data.len(),
                            suf
                        ));

                        if !is_pinned {
                            return;
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if close_button(ui).clicked() {
                                hits.clear();
                            }
                        });
                    });
                });
                ui.separator();

                egui::ScrollArea::vertical()
                    .auto_shrink([false, true])
                    .show_rows(
                        ui,
                        ui.text_style_height(&egui::TextStyle::Body),
                        window_data.len(),
                        |ui, range| {
                            for (j, (_, position, time)) in window_data.iter().enumerate() {
                                let hovered = if range.contains(&j) {
                                    ui.scope(|ui| {
                                        let time = *time;
                                        let is_current = time == sim_time.current();
                                        let mut text = egui::RichText::new(time.to_string());
                                        if is_current {
                                            text = text.italics();
                                        }
                                        egui::CollapsingHeader::new(text)
                                            .id_salt(format!("{name}#{time}#{j}"))
                                            .show(ui, |ui| {
                                                if let Some(position) = relative.position(time) {
                                                    ui.label(nformat!(
                                                        "Distance: {:.2} km",
                                                        position.length()
                                                    ));
                                                }
                                                ui.horizontal(|ui| {
                                                    if ui.button("Go to").clicked() {
                                                        sim_time.set_current_clamped(time);
                                                    }

                                                    if let Ok(mut flight_plan) =
                                                        query_flight_plan.get_mut(plot.source)
                                                    {
                                                        if ui.button("Add manoeuvre").clicked() {
                                                            flight_plan.burns.push(Burn::new(
                                                                time,
                                                                match plot.reference {
                                                                    Some(e) if e != *root => {
                                                                        BurnFrame::Relative
                                                                    }
                                                                    _ => BurnFrame::Inertial,
                                                                },
                                                                plot.reference.unwrap_or(*root),
                                                            ));
                                                            commands.trigger_targets(
                                                                FlightPlanChanged,
                                                                entity,
                                                            );
                                                        }
                                                    }
                                                })
                                            });
                                    })
                                    .response
                                    .contains_pointer()
                                } else {
                                    false
                                };

                                let direction = camera_transform.translation() - *position;
                                let size = direction.length() * perspective.fov * PICK_THRESHOLD;

                                gizmos.circle(
                                    Transform::from_translation(*position)
                                        .looking_to(direction, Vec3::Y)
                                        .to_isometry(),
                                    size,
                                    plot.color.with_alpha((hovered as usize as f32 + 1.0) / 2.0),
                                );
                            }
                        },
                    );
            });
    }
}

fn close_button(ui: &mut egui::Ui) -> egui::Response {
    let s_rect = ui.max_rect();
    let size = egui::Vec2::splat(ui.spacing().icon_width);
    let pad = (s_rect.height() - size.y) / 2.0;
    let min = egui::pos2(
        s_rect.right() - pad - size.x,
        s_rect.center().y - 0.5 * size.y,
    );
    let rect = egui::Rect::from_min_size(min, size);

    let close_id = ui.auto_id_with("window_close_button");
    let response = ui.interact(rect, close_id, egui::Sense::click());
    ui.expand_to_include_rect(response.rect);

    let visuals = ui.style().interact(&response);
    let rect = rect.shrink(2.0).expand(visuals.expansion);
    let stroke = visuals.fg_stroke;
    ui.painter() // paints \
        .line_segment([rect.left_top(), rect.right_bottom()], stroke);
    ui.painter() // paints /
        .line_segment([rect.right_top(), rect.left_bottom()], stroke);
    response
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Component, Hash)]
pub enum SeparationPlot {
    Trajectory(Entity),
    Plot(Entity),
}

impl SeparationPlot {
    pub fn entity(&self) -> Entity {
        match self {
            SeparationPlot::Trajectory(entity) => *entity,
            SeparationPlot::Plot(entity) => *entity,
        }
    }
}

fn show_separation(
    mut contexts: EguiContexts,
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
    camera: Single<(&GlobalTransform, &Camera, &PerspectiveProjection)>,
    query_data: Query<(&Name, &PlotPoints, &TrajectoryPlot, Option<&SeparationPlot>)>,
    // Don't display the separation if the prediction is still being computed.
    query_trajectory: Query<&Trajectory, Without<Predicting<SpacecraftPropagator>>>,
    query_source_of: Query<&SourceOf>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    let (camera_transform, camera, perspective) = *camera;

    let color = LinearRgba::gray(0.4);

    for (name, points, plot, target) in query_data.iter() {
        (|| {
            let target = target?;
            let (target_trajectory, target_data) = match target {
                SeparationPlot::Trajectory(entity) => (query_trajectory.get(*entity).ok()?, None),
                SeparationPlot::Plot(entity) => {
                    let (_, target_points, target_plot, _) = query_data.get(*entity).ok()?;
                    let target_trajectory = query_trajectory.get(target_plot.source).ok()?;

                    (target_trajectory, Some((target_points, target_plot)))
                }
            };
            const EPOCH_MIN: Epoch = Epoch::from_offset(Duration::MIN);
            let target_start = target_data.map_or(EPOCH_MIN, |(_, plot)| plot.start);
            const EPOCH_MAX: Epoch = Epoch::from_offset(Duration::MAX);
            let target_end = target_data.map_or(EPOCH_MAX, |(_, plot)| plot.end);

            let trajectory = query_trajectory.get(plot.source).ok()?;
            let relative = RelativeTrajectory::new(trajectory, Some(target_trajectory));
            let at = relative.closest_separation_between(
                plot.start.max(target_start),
                plot.end.min(target_end),
                0.001,
                1000,
                |t1, t2, at| {
                    t1.position(at)
                        .unwrap()
                        .distance_squared(t2.position(at).unwrap())
                },
            )?;

            let position = points.evaluate(at)?;
            let direction = camera_transform.translation() - position;
            let size = direction.length() * 6e-3 * perspective.fov;
            gizmos.rect(
                Transform::from_translation(position)
                    .looking_to(direction, camera_transform.up())
                    .to_isometry(),
                Vec2::splat(size),
                color,
            );

            // If there is no target data, that means the target is a trajectory.
            // Find the first plotted trajectory that contains the time of closest separation.
            let target_data = target_data.or_else(|| {
                let source_of = query_source_of.get(target.entity()).ok()?;
                source_of.iter().find_map(|plot_entity| {
                    query_data
                        .get(*plot_entity)
                        .ok()
                        .map(|(_, points, plot, _)| (points, plot))
                        .filter(|(_, plot)| plot.start <= at && plot.end >= at)
                })
            });

            let target_position = target_data.and_then(|(points, _)| points.evaluate(at));
            if let Some(target_position) = target_position {
                let direction = camera_transform.translation() - target_position;
                let size = direction.length() * 6e-3 * perspective.fov;
                gizmos.rect(
                    Transform::from_translation(target_position)
                        .looking_to(direction, camera_transform.up())
                        .to_isometry(),
                    Vec2::splat(size),
                    color,
                );
            }

            let window_position = match target_position {
                // Only show the separation line if the two plots have the same reference.
                // In that case, we also offset the window position to the middle of that line.
                Some(target_position)
                    if target_data.is_some_and(|(_, target_plot)| {
                        target_plot.reference == plot.reference
                    }) =>
                {
                    gizmos.line(position, target_position, color);
                    (position + target_position) / 2.0
                }
                _ => position,
            };
            let window_position = camera
                .world_to_viewport(camera_transform, window_position)
                .ok()?
                + Vec2::new(10.0, -15.0);

            let distance = relative.position(at)?.length();
            egui::Window::new(name.as_str())
                .id(egui::Id::new(name.to_string() + "#separation"))
                .fixed_pos(window_position.to_array())
                .fixed_size([200.0, 200.0])
                .collapsible(false)
                .resizable(false)
                .fade_in(false)
                .fade_out(false)
                .title_bar(false)
                .constrain(false)
                .show(ctx, |ui| {
                    ui.label(nformat!("Separation: {:.2} km", distance));
                });

            Some(())
        })();
    }
}
