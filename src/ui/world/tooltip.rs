use crate::{
    MainState,
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    load::SystemRoot,
    prediction::{Predicting, Trajectory},
    simulation::SimulationTime,
    ui::{
        HitData, MANOEUVRE_SIZE, MarkerGizmoConfigGroup, PICK_THRESHOLD, PickingSet, PlotConfig,
        PlotPoints, PlotSource, PlotSourceOf, PointerHit, PointerHover, WorldInteraction,
        WorldUiSet, nformat,
    },
};

use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use ephemeris::{EvaluateTrajectory, RelativeTrajectory};
use ftime::{Duration, Epoch};

#[derive(Clone, Copy, Debug, Default, Resource)]
pub struct ManoeuvreDraggingOptions {
    pub enabled: bool,
}

pub struct TooltipPlugin;

impl Plugin for TooltipPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TrajectoryTooltips>()
            .init_resource::<ManoeuvreTooltip>()
            .init_resource::<ManoeuvreDraggingOptions>()
            .add_systems(
                PostUpdate,
                (
                    // Picking runs after egui has written whether it wants input, which is done
                    // after the `EguiPrimaryContextPass` schedule, meaning tooltips will be updated
                    // after they have been rendered (one frame delay).
                    (
                        update_manoeuvre_tooltip,
                        update_trajectory_tooltips,
                        update_separation_tooltip,
                    )
                        .before(bevy_egui::EguiPostUpdateSet::EndPass)
                        .chain(),
                    (
                        manoeuvre_tooltip_gizmo,
                        trajectory_tooltips_gizmo.after(bevy_egui::EguiPostUpdateSet::EndPass),
                        manoeuvre_dragging,
                        separation_tooltip_gizmo,
                    ),
                )
                    .after(PickingSet)
                    .chain(),
            )
            .add_systems(
                EguiPrimaryContextPass,
                (
                    trajectory_tooltips_window,
                    manoeuvre_tooltip_window,
                    separation_tooltip_window,
                )
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

#[derive(Clone, Copy, Default)]
pub struct ManoeuvreTooltipData {
    pub id: uuid::Uuid,
    pub position: Option<Vec3>,
    pub dragging: bool,
}

#[derive(Clone, Copy, Default, Resource)]
pub struct ManoeuvreTooltip(pub Option<(Entity, ManoeuvreTooltipData)>);

impl ManoeuvreTooltip {
    pub fn clear(&mut self) {
        self.0 = None;
    }
}

fn update_manoeuvre_tooltip(
    query_plot: Query<(&PlotPoints, &PlotSource)>,
    query_flight_plan: Query<&FlightPlan>,
    hover: Res<PointerHover>,
    mut tooltip: ResMut<ManoeuvreTooltip>,
) {
    if !tooltip.0.is_some_and(|(_, data)| data.dragging) {
        tooltip.clear();
        if let Some(PointerHit(entity, HitData::Manoeuvre(data))) = &hover.0 {
            tooltip.0 = Some((
                *entity,
                ManoeuvreTooltipData {
                    id: data.id,
                    position: None,
                    dragging: false,
                },
            ));
        }
    }

    if let Some((tooltip_entity, tooltip_data)) = &mut tooltip.0
        && let Ok((points, source)) = query_plot.get(*tooltip_entity)
        && let Some(burn) = query_flight_plan
            .get(source.entity())
            .ok()
            .and_then(|flight_plan| flight_plan.get_burn(tooltip_data.id))
    {
        tooltip_data.position = points.evaluate(burn.start);
    }
}

fn manoeuvre_tooltip_gizmo(
    tooltip: Res<ManoeuvreTooltip>,
    camera: Single<(&GlobalTransform, &Projection)>,
    query_plot: Query<&PlotConfig>,
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
) {
    let Some((tooltip_entity, tooltip_data)) = tooltip.0 else {
        return;
    };

    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    let Ok(plot) = query_plot.get(tooltip_entity) else {
        return;
    };

    if let Some(position) = tooltip_data.position {
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
}

fn manoeuvre_tooltip_window(
    tooltip: Res<ManoeuvreTooltip>,
    camera: Single<(&GlobalTransform, &Camera)>,
    query_plot: Query<(&PlotPoints, &PlotSource)>,
    query_flight_plan: Query<(&Name, &FlightPlan)>,
    mut contexts: EguiContexts,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let Some((tooltip_entity, tooltip_data)) = tooltip.0 else {
        return;
    };

    let (camera_transform, camera) = *camera;

    let Ok((points, source)) = query_plot.get(tooltip_entity) else {
        return;
    };
    let Ok((name, flight_plan)) = query_flight_plan.get(source.entity()) else {
        return;
    };
    let Some((burn_idx, burn)) = flight_plan
        .burns
        .iter()
        .enumerate()
        .find(|(_, burn)| burn.id == tooltip_data.id)
    else {
        return;
    };

    let Some(position) = points.evaluate(burn.start) else {
        return;
    };
    let Ok(vp_position) = camera.world_to_viewport(camera_transform, position) else {
        return;
    };
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
}

// A bit janky by nature but still useful.
fn manoeuvre_dragging(
    mut commands: Commands,
    drag_opts: Res<ManoeuvreDraggingOptions>,
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut tooltip: ResMut<ManoeuvreTooltip>,
    mut world_ui: ResMut<WorldInteraction>,
    query_plot: Query<(&PlotPoints, &PlotSource)>,
    mut query_flight_plan: Query<&mut FlightPlan>,
) {
    if !drag_opts.enabled {
        return;
    }

    let Some((tooltip_entity, tooltip_data)) = &mut tooltip.0 else {
        return;
    };

    if mouse.just_pressed(MouseButton::Left) {
        tooltip_data.dragging = true;
        world_ui.using_pointer = true;
    }

    if !tooltip_data.dragging {
        return;
    }

    if mouse.just_released(MouseButton::Left) {
        tooltip_data.dragging = false;
        world_ui.using_pointer = false;
    }

    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Ok((points, source)) = query_plot.get(*tooltip_entity) else {
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
        let Ok(mut flight_plan) = query_flight_plan.get_mut(source.entity()) else {
            return;
        };

        if let Some(burn) = flight_plan.get_burn_mut(tooltip_data.id) {
            time = time.round(Duration::from_seconds(1.0));
            if burn.start != time {
                burn.start = time;
                commands.trigger_targets(FlightPlanChanged, source.entity());
            }
        }
    }
}

#[derive(Clone)]
pub struct TrajectoryTooltipData {
    pub time: Epoch,
    pub position: Option<Vec3>,
    pub highlight: bool,
}

#[derive(Clone, Default)]
pub struct TrajectoryTooltip(pub Option<(Entity, Vec<TrajectoryTooltipData>)>);

impl TrajectoryTooltip {
    pub fn clear(&mut self) {
        self.0 = None;
    }
}

#[derive(Resource, Default)]
pub struct TrajectoryTooltips {
    pub hovered: TrajectoryTooltip,
    pub pinned: TrajectoryTooltip,
}

impl TrajectoryTooltips {
    pub fn pin_hovered(&mut self) {
        self.pinned.clone_from(&self.hovered);
    }

    pub fn iter(&self) -> impl Iterator<Item = &TrajectoryTooltip> {
        [&self.hovered, &self.pinned].into_iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut TrajectoryTooltip> {
        [&mut self.hovered, &mut self.pinned].into_iter()
    }
}

fn update_trajectory_tooltips(
    input: Res<ButtonInput<MouseButton>>,
    query_plot: Query<&PlotPoints>,
    hover: Res<PointerHover>,
    mut tooltips: ResMut<TrajectoryTooltips>,
) {
    tooltips.hovered.clear();

    if let Some(PointerHit(entity, HitData::Trajectory(data))) = &hover.0 {
        tooltips.hovered.0 = Some((
            *entity,
            data.hits
                .iter()
                .map(|hit| TrajectoryTooltipData {
                    time: hit.time,
                    position: None,
                    highlight: false,
                })
                .collect(),
        ));

        if input.just_pressed(MouseButton::Left) {
            tooltips.pin_hovered();
        }
    }

    for tooltip in tooltips.iter_mut() {
        let Some((tooltip_entity, tooltip_data)) = &mut tooltip.0 else {
            continue;
        };
        let Ok(points) = query_plot.get(*tooltip_entity) else {
            continue;
        };
        for data in tooltip_data.iter_mut() {
            data.position = points.evaluate(data.time);
        }
    }
}

fn trajectory_tooltips_gizmo(
    tooltips: Res<TrajectoryTooltips>,
    camera: Single<(&GlobalTransform, &Projection)>,
    query_plot: Query<&PlotConfig>,
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    for tooltip in tooltips.iter() {
        let Some((tooltip_entity, tooltip_data)) = &tooltip.0 else {
            continue;
        };
        let Some(plot) = query_plot.get(*tooltip_entity).ok() else {
            continue;
        };
        for data in tooltip_data.iter() {
            let Some(world_position) = data.position else {
                continue;
            };
            let direction = camera_transform.translation() - world_position;
            let size = direction.length() * perspective.fov * PICK_THRESHOLD;

            gizmos.circle(
                Transform::from_translation(world_position)
                    .looking_to(direction, Vec3::Y)
                    .to_isometry(),
                size,
                plot.color
                    .with_alpha((data.highlight as usize as f32 + 1.0) / 2.0),
            );
        }
    }
}

fn trajectory_tooltips_window(
    root: Single<Entity, With<SystemRoot>>,
    camera: Single<(&GlobalTransform, &Camera)>,
    mut tooltips: ResMut<TrajectoryTooltips>,
    query_plot: Query<(&Name, &PlotConfig, &PlotSource)>,
    query_trajectory: Query<&Trajectory>,
    mut commands: Commands,
    mut contexts: EguiContexts,
    mut sim_time: ResMut<SimulationTime>,
    mut query_flight_plan: Query<&mut FlightPlan>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let (camera_transform, camera) = *camera;

    for (i, tooltip) in tooltips.iter_mut().enumerate() {
        let Some((tooltip_entity, tooltip_data)) = &mut tooltip.0 else {
            continue;
        };
        let is_pinned = i == 1;
        let Ok((name, plot, source)) = query_plot.get(*tooltip_entity) else {
            continue;
        };

        let Some(window_position) = tooltip_data
            .iter()
            .flat_map(|d| camera.world_to_viewport(camera_transform, d.position?).ok())
            .max_by(|a, b| a.x.total_cmp(&b.x))
            .map(|p| p + Vec2::new(20.0, -20.0))
        else {
            continue;
        };

        let Some(relative) = plot.get_relative_trajectory(source.entity(), &query_trajectory)
        else {
            continue;
        };
        let window = egui::Window::new(name.as_str())
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
                        let suf = if tooltip_data.len() == 1 { "" } else { "s" };
                        ui.label(format!(
                            "{} — {} crossing{}",
                            name.as_str(),
                            tooltip_data.len(),
                            suf
                        ));

                        if is_pinned {
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if close_button(ui).clicked() {
                                        ui.close_kind(egui::UiKind::Window);
                                    }
                                },
                            );
                        }
                    });
                });
                ui.separator();

                egui::ScrollArea::vertical()
                    .auto_shrink([false, true])
                    .show_rows(
                        ui,
                        ui.text_style_height(&egui::TextStyle::Body),
                        tooltip_data.len(),
                        |ui, range| {
                            for (j, data) in tooltip_data.iter_mut().enumerate() {
                                data.highlight = if range.contains(&j) {
                                    ui.scope(|ui| {
                                        let time = data.time;
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
                                                        query_flight_plan.get_mut(source.entity())
                                                        && ui.button("Add manoeuvre").clicked()
                                                    {
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
                                                            source.entity(),
                                                        );
                                                    }
                                                })
                                            });
                                    })
                                    .response
                                    .contains_pointer()
                                } else {
                                    false
                                };
                            }
                        },
                    );
            });

        if window.is_some_and(|inner| inner.response.should_close()) {
            tooltip.clear();
        }
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
pub enum PlotSeparation {
    Trajectory(Entity),
    Plot(Entity),
}

impl PlotSeparation {
    pub fn entity(&self) -> Entity {
        match self {
            PlotSeparation::Trajectory(entity) => *entity,
            PlotSeparation::Plot(entity) => *entity,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct SeparationTooltipData {
    pub time: Epoch,
    pub distance: f64,
    pub position: Option<Vec3>,
    pub target_position: Option<Vec3>,
}

#[derive(Clone, Component)]
pub struct SeparationTooltip {
    pub entity: Entity,
    pub target: Entity,
    pub data: SeparationTooltipData,
    pub show_separation_line: bool,
}

fn update_separation_tooltip(
    mut commands: Commands,
    query_plot: Query<(
        Entity,
        &PlotConfig,
        &PlotPoints,
        &PlotSource,
        Option<&PlotSeparation>,
    )>,
    query_trajectory: Query<&Trajectory, Without<Predicting>>,
    query_source_of: Query<&PlotSourceOf>,
) {
    for (entity, plot, points, source, separation) in query_plot.iter() {
        let Some(separation) = separation else {
            continue;
        };

        let Ok(trajectory) = query_trajectory.get(source.entity()) else {
            continue;
        };

        let (target_trajectory, target_data) = match separation {
            PlotSeparation::Trajectory(entity) => (query_trajectory.get(*entity), None),
            PlotSeparation::Plot(entity) => {
                let Ok((_, target_plot, target_points, target_source, _)) = query_plot.get(*entity)
                else {
                    continue;
                };
                (
                    query_trajectory.get(target_source.entity()),
                    Some((target_plot, target_points)),
                )
            }
        };
        let Ok(target_trajectory) = target_trajectory else {
            continue;
        };

        const EPOCH_MIN: Epoch = Epoch::from_offset(Duration::MIN);
        let target_start = target_data.map_or(EPOCH_MIN, |(plot, _)| plot.start);
        const EPOCH_MAX: Epoch = Epoch::from_offset(Duration::MAX);
        let target_end = target_data.map_or(EPOCH_MAX, |(plot, _)| plot.end);

        let relative = RelativeTrajectory::new(trajectory, Some(target_trajectory));
        if let Some(at) = relative.closest_separation_between(
            plot.start.max(target_start),
            plot.end.min(target_end),
            0.001,
            1000,
            |t1, t2, at| {
                t1.position(at)
                    .unwrap()
                    .distance_squared(t2.position(at).unwrap())
            },
        ) {
            // If there is no target data, that means the target is a trajectory.
            // Find the first plotted trajectory that contains the time of closest separation.
            let target_data = target_data.or_else(|| {
                let source_of = query_source_of.get(separation.entity()).ok()?;
                source_of.iter().find_map(|plot_entity| {
                    query_plot
                        .get(plot_entity)
                        .ok()
                        .map(|(_, plot, points, ..)| (plot, points))
                        .filter(|(plot, _)| plot.start <= at && plot.end >= at)
                })
            });

            commands.entity(entity).insert(SeparationTooltip {
                entity,
                target: separation.entity(),
                data: SeparationTooltipData {
                    time: at,
                    distance: relative.position(at).unwrap().length(),
                    position: points.evaluate(at),
                    target_position: target_data.and_then(|(_, points)| points.evaluate(at)),
                },
                show_separation_line: target_data
                    .is_some_and(|(target_plot, _)| target_plot.reference == plot.reference),
            });
        }
    }
}

fn separation_tooltip_gizmo(
    query: Query<&SeparationTooltip>,
    camera: Single<(&GlobalTransform, &Projection)>,
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    let color = LinearRgba::gray(0.4);

    for tooltip in query.iter() {
        let Some(position) = tooltip.data.position else {
            return;
        };
        let direction = camera_transform.translation() - position;
        let size = direction.length() * 6e-3 * perspective.fov;
        gizmos.rect(
            Transform::from_translation(position)
                .looking_to(direction, camera_transform.up())
                .to_isometry(),
            Vec2::splat(size),
            color,
        );

        if let Some(target_position) = tooltip.data.target_position {
            let direction = camera_transform.translation() - target_position;
            let size = direction.length() * 6e-3 * perspective.fov;
            gizmos.rect(
                Transform::from_translation(target_position)
                    .looking_to(direction, camera_transform.up())
                    .to_isometry(),
                Vec2::splat(size),
                color,
            );

            if tooltip.show_separation_line {
                gizmos.line(position, target_position, color);
            }
        }
    }
}

fn separation_tooltip_window(
    camera: Single<(&GlobalTransform, &Camera)>,
    query: Query<(&Name, &SeparationTooltip)>,
    mut contexts: EguiContexts,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let (camera_transform, camera) = *camera;

    for (name, tooltip) in query.iter() {
        let Some(position) = tooltip.data.position else {
            return;
        };

        let Ok(window_position) = camera
            .world_to_viewport(
                camera_transform,
                match (tooltip.data.target_position, tooltip.show_separation_line) {
                    (Some(target_position), true) => (position + target_position) / 2.0,
                    _ => position,
                },
            )
            .map(|p| p + Vec2::new(10.0, -15.0))
        else {
            return;
        };

        egui::Window::new(name.as_str())
            .id(egui::Id::new(name.to_string() + "#separation"))
            .collapsible(false)
            .resizable(false)
            .fade_in(false)
            .fade_out(false)
            .title_bar(false)
            .constrain(false)
            .fixed_pos(window_position.to_array())
            .fixed_size([200.0, 200.0])
            .show(ctx, |ui| {
                ui.label(nformat!("Separation: {:.2} km", tooltip.data.distance));
            });
    }
}

pub fn remove_separation(mut entity: EntityWorldMut) {
    entity.remove::<SeparationTooltip>();
    entity.remove::<PlotSeparation>();
}
