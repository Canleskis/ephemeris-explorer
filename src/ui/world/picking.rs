// The backend provided by bevy_picking is not really suitable for our use case, since the HitData
// struct is missing some data we need to send. We also don't need the complexity of bevy_picking so
// we can reimplement what we need.

use crate::{
    flight_plan::FlightPlan,
    selection::Selectable,
    ui::{PlotPoints, PlotSourceOf},
};

use bevy::{
    math::bounding::{BoundingSphere, RayCast3d},
    picking::{backend::ray::RayMap, pointer::PointerLocation},
    prelude::*,
    render::camera::NormalizedRenderTarget,
};
use bevy_egui::{EguiContext, EguiContextSettings, input::WindowToEguiContextMap};
use ftime::Epoch;

#[derive(Clone, Copy, Debug)]
pub struct TrajectoryHit {
    pub time: Epoch,
    pub separation: f32,
    pub depth: f32,
}

#[derive(Clone, Debug)]
pub struct TrajectoryHits {
    pub hits: Vec<TrajectoryHit>,
}

impl TrajectoryHits {
    pub fn is_empty(&self) -> bool {
        self.hits.is_empty()
    }

    pub fn min_distance(&self) -> Option<f32> {
        self.hits
            .iter()
            .map(|hit| hit.depth)
            .min_by(|a, b| a.total_cmp(b))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ManoeuvreHit {
    pub id: uuid::Uuid,
    pub depth: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyHit {
    pub depth: f32,
}

#[derive(Clone, Debug)]
pub enum HitData {
    Trajectory(TrajectoryHits),
    Manoeuvre(ManoeuvreHit),
    Body(BodyHit),
    Ui,
}

impl HitData {
    pub fn depth(&self) -> f32 {
        match self {
            HitData::Trajectory(hits) => hits.min_distance().unwrap_or(f32::MAX),
            HitData::Manoeuvre(hit) => hit.depth,
            HitData::Body(hit) => hit.depth,
            HitData::Ui => -1_000_000.0,
        }
    }
}

#[derive(Clone, Debug, Event)]
pub struct PointerHit(pub Entity, pub HitData);

#[derive(Resource, Default)]
pub struct PointerHover(pub Option<PointerHit>);

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct PickingSet;

pub struct CustomPickingPlugin;

impl Plugin for CustomPickingPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<PointerHit>()
            .init_resource::<PointerHover>()
            .configure_sets(
                PostUpdate,
                PickingSet.run_if(not(crate::camera::using_pointer)),
            )
            .add_systems(
                PostUpdate,
                (
                    (
                        body_picking,
                        (manoeuvre_picking, trajectory_picking)
                            .after(crate::ui::compute_plot_points_parallel),
                        // Some inputs are not always properly updated yet when this runs since it's
                        // not guaranteed to run after egui's end pass, but this is better than
                        // forcing all systems that need to run after picking to run after egui's
                        // end pass.
                        egui_picking,
                    ),
                    update_pointer_hover,
                )
                    .chain()
                    .in_set(PickingSet),
            );
    }
}

pub const BODY_RADIUS: f32 = 1e-2;

fn body_picking(
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    query_can_select: Query<(Entity, &GlobalTransform, &Selectable)>,
    mut events: EventWriter<PointerHit>,
) {
    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    if let Some((entity, depth)) = query_can_select
        .iter()
        .filter_map(|(entity, transform, clickable)| {
            let distance = transform.translation().distance(Vec3::from(ray.origin));
            let toi = ray.sphere_intersection_at(&bevy::math::bounding::BoundingSphere::new(
                transform.translation(),
                clickable.radius + distance * perspective.fov * BODY_RADIUS,
            ))?;

            Some(((entity, toi), clickable.index as f32 * toi))
        })
        .min_by(|(_, i1), (_, i2)| i1.total_cmp(i2))
        .map(|(d, _)| d)
    {
        events.write(PointerHit(entity, HitData::Body(BodyHit { depth })));
    }
}

pub const PICK_THRESHOLD: f32 = 6e-3;

pub fn trajectory_picking(
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    query_plot: Query<(Entity, &PlotPoints)>,
    mut events: EventWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((min_distance, (entity, mut picks))) = query_plot
        .iter()
        .map(|(entity, points)| {
            (
                entity,
                TrajectoryHits {
                    hits: points
                        .ray_distances(&ray)
                        .map(|(time, separation, distance)| TrajectoryHit {
                            time,
                            separation,
                            depth: distance,
                        })
                        .filter(|hit| hit.separation < hit.depth * perspective.fov * PICK_THRESHOLD)
                        .collect::<Vec<_>>(),
                },
            )
        })
        .filter_map(|(entity, hits)| Some((hits.min_distance()?, (entity, hits))))
        .min_by(|(a, _), (b, _)| a.total_cmp(b))
    {
        let max = min_distance * (1.0 + 0.01);
        picks.hits.retain(|hit| hit.depth <= max);

        events.write(PointerHit(entity, HitData::Trajectory(picks)));
    }
}

pub const MANOEUVRE_SIZE: f32 = 2e-2;

// We don't have a `ManoeuvrePlotPoint` component like we have for trajectories, so we recompute the
// manoeuvre positions here.
pub fn manoeuvre_picking(
    ray_map: Res<RayMap>,
    query: Query<(&FlightPlan, &PlotSourceOf)>,
    query_plot: Query<(Entity, &PlotPoints)>,
    perspective: Single<&Projection, With<Camera>>,
    mut events: EventWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((entity, data)) = query
        .iter()
        .flat_map(|(flight_plan, source_of)| {
            query_plot
                .iter_many(source_of.iter())
                .flat_map(|(entity, points)| {
                    let ray = &ray;
                    flight_plan.burns.iter().filter_map(move |burn| {
                        if !burn.enabled || burn.overlaps {
                            return None;
                        }

                        let world_pos = points.evaluate(burn.start)?;
                        let r = world_pos.distance(Vec3::from(ray.origin));
                        ray.sphere_intersection_at(&BoundingSphere::new(
                            world_pos,
                            r * perspective.fov * MANOEUVRE_SIZE / 2.0,
                        ))
                        .map(|depth| (entity, ManoeuvreHit { id: burn.id, depth }))
                    })
                })
        })
        .min_by(|(_, a), (_, b)| a.depth.total_cmp(&b.depth))
    {
        events.write(PointerHit(entity, HitData::Manoeuvre(data)));
    }
}

pub fn egui_picking(
    window_to_egui_context_map: Res<WindowToEguiContextMap>,
    pointers: Query<&PointerLocation>,
    mut egui_context: Query<(Entity, &mut EguiContext, &EguiContextSettings, &Camera)>,
    mut output: EventWriter<PointerHit>,
) {
    for location in pointers.iter().filter_map(|p| p.location.as_ref()) {
        if let NormalizedRenderTarget::Window(window) = location.target {
            for window_context_entity in window_to_egui_context_map
                .window_to_contexts
                .get(&window.entity())
                .cloned()
                .unwrap_or_default()
            {
                let Ok((entity, mut ctx, settings, camera)) =
                    egui_context.get_mut(window_context_entity)
                else {
                    continue;
                };
                if !camera
                    .physical_viewport_rect()
                    .is_some_and(|rect| rect.as_rect().contains(location.position))
                {
                    continue;
                }

                if settings.capture_pointer_input
                    && (ctx.get_mut().is_pointer_over_area() || ctx.get_mut().is_using_pointer())
                {
                    output.write(PointerHit(entity, HitData::Ui));
                }
            }
        }
    }
}

fn _show_body_picking_zone(
    mut gizmos: Gizmos,
    query_camera: Query<&GlobalTransform, With<Camera>>,
    query_can_select: Query<(&GlobalTransform, &Selectable)>,
) {
    let Ok(camera_transform) = query_camera.single() else {
        return;
    };

    for (transform, can_select) in &query_can_select {
        let direction = transform.translation() - camera_transform.translation();
        let radius = can_select.radius + direction.length_squared() / 100.0;
        gizmos.circle(
            transform
                .compute_transform()
                .looking_to(direction, Vec3::Y)
                .to_isometry(),
            radius,
            Color::WHITE,
        );
    }
}

fn update_pointer_hover(mut hover: ResMut<PointerHover>, mut events: EventReader<PointerHit>) {
    hover.0 = events
        .read()
        .min_by(|PointerHit(_, a), PointerHit(_, b)| a.depth().total_cmp(&b.depth()))
        .cloned();
}
