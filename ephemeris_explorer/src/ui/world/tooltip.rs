use crate::{
    MainState,
    analysis::{BurnPlotSegment, PlotSegment},
    camera::CameraProximityIgnore,
    dynamics::{SoiTransitions, Trajectory},
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    floating_origin::{BigSpace, Grid, GridExt},
    load::SystemRoot,
    simulation::SimulationTime,
    ui::{
        HitData, MANOEUVRE_SIZE, MarkerGizmoConfigGroup, PICK_THRESHOLD, PickingSet, PlotConfig,
        PlotPoints, PlotSource, PlotSourceOf, PointerHit, PointerHover, Position, WorldInteraction,
        WorldUiSet,
    },
};

use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use ephemeris::{EvaluateTrajectory, Frame};
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
            .add_systems(Startup, prepare_tooltip_caches)
            .add_systems(
                PostUpdate,
                (
                    (plot_manoeuvres_markers, plot_transitions_markers),
                    (
                        (
                            manoeuvre_dragging,
                            update_trajectory_tooltips,
                            update_manoeuvre_tooltip,
                        )
                            .chain(),
                        update_separation_tooltip,
                    )
                        .before(bevy_egui::EguiPostUpdateSet::EndPass),
                    (
                        trajectory_tooltips_world.after(bevy_egui::EguiPostUpdateSet::EndPass),
                        separation_tooltip_gizmo,
                    ),
                )
                    .chain()
                    .after(PickingSet::Hover)
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running)),
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

fn plot_manoeuvres_markers(
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
    query: Query<(&Trajectory, &FlightPlan, &PlotSourceOf)>,
    query_trajectory: Query<&Trajectory>,
    query_plot: Query<&PlotPoints, With<BurnPlotSegment>>,
    root: Single<&Grid, With<BigSpace>>,
    camera: Single<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    for (trajectory, flight_plan, source_of) in query.iter() {
        for burn in flight_plan.burns.values() {
            if !burn.is_active() {
                continue;
            }
            for points in query_plot.iter_many(source_of.iter()) {
                let Some(world_pos) = points.evaluate(burn.start) else {
                    continue;
                };

                if let Some(transform) = trajectory.state_vector(burn.start).and_then(|sv| {
                    burn.try_reference_frame(&query_trajectory).ok()?.transform(
                        burn.start,
                        &sv,
                        &(),
                    )
                }) {
                    let prograde = root.to_global_vector(transform.0.x_axis).as_vec3();
                    let radial = root.to_global_vector(transform.0.y_axis).as_vec3();
                    let normal = root.to_global_vector(transform.0.z_axis).as_vec3();

                    let size = camera_transform.translation().distance(world_pos)
                        * perspective.fov
                        * MANOEUVRE_SIZE;

                    gizmos.arrow(world_pos, world_pos + prograde * size, LinearRgba::GREEN);
                    gizmos.arrow(world_pos, world_pos + normal * size, LinearRgba::BLUE);
                    gizmos.arrow(world_pos, world_pos + radial * size, LinearRgba::RED);
                }
            }
        }
    }
}

fn plot_transitions_markers(
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
    query: Query<(&SoiTransitions, &PlotSourceOf)>,
    query_plot: Query<(&PlotPoints, &PlotConfig)>,
    camera: Single<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    for (transitions, source_of) in query.iter() {
        for &(time, _entity) in transitions.iter() {
            for (points, plot) in query_plot.iter_many(source_of.iter()) {
                let Some(world_pos) = points.evaluate(time) else {
                    continue;
                };

                let direction = camera_transform.translation() - world_pos;
                let size = direction.length() * perspective.fov * 0.004;

                gizmos.circle(
                    Transform::from_translation(world_pos)
                        .looking_to(direction, camera_transform.up())
                        .to_isometry(),
                    size,
                    plot.color,
                );
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ManoeuvreTooltipData {
    pub id: uuid::Uuid,
    time: Epoch,
    pub position: Option<Vec3>,
    pub dragging: bool,
}

#[derive(Clone, Copy, Default, Resource)]
pub struct ManoeuvreTooltip(pub Option<(Entity, Entity, ManoeuvreTooltipData)>);

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
    if !tooltip.0.is_some_and(|(.., data)| data.dragging) {
        tooltip.clear();
        if let Some(PointerHit(entity, HitData::Manoeuvre(data))) = &hover.0 {
            let Ok((_, source)) = query_plot.get(*entity) else {
                return;
            };
            tooltip.0 = Some((
                *entity,
                source.entity,
                ManoeuvreTooltipData {
                    id: data.id,
                    time: Epoch::default(), // placeholder value
                    position: None,
                    dragging: false,
                },
            ));
        }
    }

    if let Some((tooltip_entity, _, tooltip_data)) = &mut tooltip.0
        && let Ok((points, source)) = query_plot.get(*tooltip_entity)
        && let Some(burn) = query_flight_plan
            .get(source.entity)
            .ok()
            .and_then(|flight_plan| flight_plan.burns.get(&tooltip_data.id))
    {
        tooltip_data.time = burn.start;
        tooltip_data.position = points.evaluate(burn.start);
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

    let Some((tooltip_entity, _, tooltip_data)) = tooltip.0 else {
        return;
    };

    let (camera_transform, camera) = *camera;

    let Ok((points, source)) = query_plot.get(tooltip_entity) else {
        return;
    };
    let Ok((name, flight_plan)) = query_flight_plan.get(source.entity) else {
        return;
    };
    let Some((burn_idx, _, burn)) = flight_plan.burns.get_full(&tooltip_data.id) else {
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
// The fact that we drag a manoeuvre on a changing trajectory causes multiple problems:
// - Moving the manoeuvre to a future position retroactively changes the trajectory, leading to a
//   different position that what might have been intended.
// - When a trajectory is updated, its plot segment entities may change, so we can't reliably track
//   on which entity the tooltip should be.
// Fixing this requires a design change: have spacecraft trajectories split into two after each
// manoeuvre, one representing the unapplied changes that we use to edit the manoeuvre, and one
// with the changes applied. A manoeuvre would need to be applied once accepted by the user (or
// automatically once the user input stops?).
fn manoeuvre_dragging(
    mut commands: Commands,
    drag_opts: Res<ManoeuvreDraggingOptions>,
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut tooltip: ResMut<ManoeuvreTooltip>,
    mut world_ui: ResMut<WorldInteraction>,
    query_points: Query<&PlotPoints>,
    mut query_flight_plan: Query<(&mut FlightPlan, &PlotSourceOf)>,
) {
    if !drag_opts.enabled {
        return;
    }

    let Some((tooltip_entity, source_entity, tooltip_data)) = &mut tooltip.0 else {
        world_ui
            .map_unchanged(|ui| &mut ui.using_pointer)
            .set_if_neq(false);
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

    let Ok((mut flight_plan, source_of)) = query_flight_plan.get_mut(*source_entity) else {
        return;
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((new_entity, (mut time, _, _))) = source_of
        .iter()
        .zip(query_points.iter_many(source_of.iter()))
        .flat_map(|(entity, points)| {
            points
                .ray_distances(&ray)
                .filter(|hit| hit.1 < hit.2 * perspective.fov * MANOEUVRE_SIZE)
                .min_by(|a, b| a.0.cmp(&b.0))
                .map(|hit| (entity, hit))
        })
        .min_by(|(_, a), (_, b)| a.2.total_cmp(&b.2))
        && let Some(burn) = flight_plan.burns.get_mut(&tooltip_data.id)
    {
        time = time.round(Duration::from_seconds(1.0));
        if burn.start != time {
            burn.start = time;
            commands.trigger(FlightPlanChanged(*source_entity));
        }
        *tooltip_entity = new_entity;
    }
}

#[derive(Clone)]
pub struct TrajectoryTooltipData {
    pub time: Epoch,
    pub position: Option<Vec3>,
    pub selected: bool,
}

#[derive(Clone, Default)]
pub struct TrajectoryTooltip {
    data: Option<(Entity, Vec<TrajectoryTooltipData>)>,
    show_window: bool,
}

impl TrajectoryTooltip {
    pub fn clear(&mut self) {
        self.data = None;
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
        self.pinned.show_window = true;
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
        tooltips.hovered.data = Some((
            *entity,
            data.hits
                .iter()
                .map(|hit| TrajectoryTooltipData {
                    time: hit.time,
                    position: None,
                    selected: false,
                })
                .collect(),
        ));
        tooltips.hovered.show_window = true;

        if input.just_pressed(MouseButton::Left) {
            tooltips.pin_hovered();
        }
    }

    for tooltip in tooltips.iter_mut() {
        let Some((tooltip_entity, tooltip_data)) = &mut tooltip.data else {
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

#[derive(Clone, Copy, Debug, Component)]
struct TooltipMesh;

pub struct TrajectoryTooltipCache {
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    entities: Vec<Entity>,
}

#[derive(Component)]
#[require(Transform, Visibility)]
pub struct TrajectoryTooltipCaches {
    hovered: TrajectoryTooltipCache,
    pinned: TrajectoryTooltipCache,
}

impl TrajectoryTooltipCaches {
    #[inline]
    fn iter_mut(&mut self) -> impl Iterator<Item = &mut TrajectoryTooltipCache> {
        [&mut self.hovered, &mut self.pinned].into_iter()
    }
}

fn prepare_tooltip_caches(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(TrajectoryTooltipCaches {
        hovered: TrajectoryTooltipCache {
            mesh: meshes.add(Sphere::new(1.0).mesh().uv(16, 8)),
            material: materials.add(StandardMaterial {
                unlit: true,
                alpha_mode: AlphaMode::Blend,
                ..default()
            }),
            entities: Vec::new(),
        },
        pinned: TrajectoryTooltipCache {
            mesh: meshes.add(Sphere::new(1.0).mesh().uv(16, 8)),
            material: materials.add(StandardMaterial {
                unlit: true,
                alpha_mode: AlphaMode::Blend,
                ..default()
            }),
            entities: Vec::new(),
        },
    });
}

// This uses a pool of entities to display (the shared) sphere meshes for the two tooltips.
fn trajectory_tooltips_world(
    tooltips: Res<TrajectoryTooltips>,
    camera: Single<(&GlobalTransform, &Projection), Without<TooltipMesh>>,
    query_plot: Query<&PlotConfig>,
    mut query_sphere: Query<(&mut GlobalTransform, &mut Visibility), With<TooltipMesh>>,
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut caches: Single<(Entity, &mut TrajectoryTooltipCaches)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };
    let (cache_entity, caches) = &mut *caches;
    for (tooltip, cache) in tooltips.iter().zip(caches.iter_mut()) {
        for entity in cache.entities.iter() {
            if let Ok((_, mut visibility)) = query_sphere.get_mut(*entity) {
                *visibility = Visibility::Hidden;
            }
        }

        let Some((tooltip_entity, tooltip_data)) = &tooltip.data else {
            continue;
        };

        let Ok(plot) = query_plot.get(*tooltip_entity) else {
            error!("{tooltip_entity:?} with a tooltip does not exist or is not a plot segment");
            continue;
        };
        materials.get_mut(&cache.material).unwrap().base_color = plot.color;

        for (i, data) in tooltip_data.iter().enumerate() {
            let Some(world_position) = data.position else {
                continue;
            };

            let direction = camera_transform.translation() - world_position;
            let radius = if data.selected { 1.3 } else { 1.0 };
            let size = direction.length() * perspective.fov * PICK_THRESHOLD * radius;
            let transform = GlobalTransform::from(
                Transform::from_translation(world_position).with_scale(Vec3::splat(size)),
            );

            let Some(&entity) = cache.entities.get(i) else {
                let entity = commands
                    .spawn((
                        transform,
                        Visibility::Visible,
                        Mesh3d(cache.mesh.clone()),
                        MeshMaterial3d(cache.material.clone()),
                        bevy::light::NotShadowReceiver,
                        bevy::light::NotShadowCaster,
                        CameraProximityIgnore,
                        TooltipMesh,
                        ChildOf(*cache_entity),
                    ))
                    .id();
                cache.entities.push(entity);
                continue;
            };

            if let Ok((mut tooltip_transform, mut visibility)) = query_sphere.get_mut(entity) {
                *tooltip_transform = transform;
                *visibility = Visibility::Visible;
            }
        }
    }
}

fn trajectory_tooltips_window(
    root: Single<Entity, With<SystemRoot>>,
    camera: Single<(&GlobalTransform, &Camera)>,
    mut tooltips: ResMut<TrajectoryTooltips>,
    query_plot: Query<(&PlotSource, &Name)>,
    query_trajectory: Query<&Trajectory>,
    query_name: Query<&Name>,
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
        if !tooltip.show_window {
            continue;
        }
        let Some((tooltip_entity, tooltip_data)) = &mut tooltip.data else {
            continue;
        };
        let is_pinned = i == 1;
        let Ok((source, segment)) = query_plot.get(*tooltip_entity) else {
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

        let Some(relative) = source.get_relative_trajectory(&query_trajectory) else {
            continue;
        };
        let Ok(name) = query_name.get(source.entity) else {
            continue;
        };
        let window = egui::Window::new(name.as_str())
            .collapsible(false)
            .resizable(false)
            .fade_in(false)
            .fade_out(false)
            .title_bar(false)
            .constrain(false)
            .fixed_size([350.0, 200.0])
            .id(egui::Id::new(i.to_string() + "#intersections"))
            .fixed_pos(window_position.to_array())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.style_mut().interaction.selectable_labels = false;
                        let suffix = if tooltip_data.len() == 1 { "" } else { "s" };
                        ui.label(format!(
                            "{} — {} crossing{} ({})",
                            name.as_str(),
                            tooltip_data.len(),
                            suffix,
                            segment
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
                                data.selected = if range.contains(&j) {
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
                                                    ui.label(format!(
                                                        "Distance: {}",
                                                        Position::km(position.length())
                                                    ));
                                                }
                                                ui.horizontal(|ui| {
                                                    if ui.button("Go to").clicked() {
                                                        sim_time.set_current_clamped(time);
                                                    }

                                                    if let Ok(mut flight_plan) =
                                                        query_flight_plan.get_mut(source.entity)
                                                        && ui.button("Add manoeuvre").clicked()
                                                    {
                                                        flight_plan.burns.insert(
                                                            uuid::Uuid::new_v4(),
                                                            Burn::new(
                                                                time,
                                                                match source.reference {
                                                                    Some(e) if e != *root => {
                                                                        BurnFrame::Relative
                                                                    }
                                                                    _ => BurnFrame::Inertial,
                                                                },
                                                                source.reference.unwrap_or(*root),
                                                            ),
                                                        );
                                                        commands.trigger(FlightPlanChanged(
                                                            source.entity,
                                                        ));
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

#[derive(Clone, Copy, Debug, PartialEq, Component)]
pub struct PlotSeparation {
    pub entity: Entity,
    pub time: Epoch,
    pub distance: f64,
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
    pub show_window: bool,
}

fn update_separation_tooltip(
    mut commands: Commands,
    query_plot: Query<(
        Entity,
        &PlotSource,
        &PlotPoints,
        &PlotSegment,
        Option<&PlotSeparation>,
    )>,
    mut query_tooltip: Query<&mut SeparationTooltip>,
) {
    for (entity, source, points, segment, separation) in query_plot.iter() {
        let Some(separation) = separation else {
            continue;
        };

        let Ok((_, target_source, target_points, ..)) = query_plot.get(separation.entity) else {
            continue;
        };

        let updated_tooltip = SeparationTooltip {
            entity,
            target: separation.entity,
            data: SeparationTooltipData {
                time: separation.time,
                distance: separation.distance,
                position: points.evaluate(separation.time),
                target_position: target_points.evaluate(separation.time),
            },
            show_separation_line: source.reference == target_source.reference,
            show_window: source.reference == target_source.reference
                || !matches!(segment, PlotSegment::Flyby),
        };

        let Ok(mut tooltip) = query_tooltip.get_mut(entity) else {
            commands.entity(entity).insert(updated_tooltip);
            continue;
        };
        *tooltip = updated_tooltip;
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
        if !tooltip.show_window {
            continue;
        }

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
            .id(egui::Id::new(
                tooltip.entity.to_bits() + tooltip.target.to_bits(),
            ))
            .collapsible(false)
            .resizable(false)
            .fade_in(false)
            .fade_out(false)
            .title_bar(false)
            .constrain(false)
            .fixed_pos(window_position.to_array())
            .fixed_size([200.0, 200.0])
            .show(ctx, |ui| {
                ui.label(format!(
                    "Separation: {}",
                    Position::km(tooltip.data.distance)
                ));
            });
    }
}

pub fn remove_tooltip(mut entity: EntityWorldMut) {
    entity.remove::<SeparationTooltip>();
    entity.remove::<PlotSeparation>();
}
