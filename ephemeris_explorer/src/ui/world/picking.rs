// The backend provided by bevy_picking is not really suitable for our use case, since the HitData
// struct is missing some data we need to send. We also don't need the complexity of bevy_picking so
// we can reimplement what we need.

// TODO: There is a bunch of duplicated code between marker picking and marker plotting; figure out
// a way to reduce it.

use crate::{
    analysis::BurnPlotSegment,
    dynamics::{Apsides, Apsis, Trajectory},
    flight_plan::FlightPlan,
    selection::Selectable,
    ui::{PlotPoints, PlotSource, PlotSourceOf},
};

use bevy::{
    camera::NormalizedRenderTarget,
    math::bounding::{BoundingSphere, RayCast3d},
    picking::{backend::ray::RayMap, pointer::PointerLocation},
    prelude::*,
};
use bevy_egui::{EguiContext, EguiContextSettings, input::WindowToEguiContextMap};
use ephemeris::{BoundedTrajectory, EvaluateTrajectory};
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

    pub fn min_depth(&self) -> Option<f32> {
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
pub struct ApsisHit {
    pub apsis: Apsis,
    pub depth: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyHit {
    pub depth: f32,
}

#[derive(Clone, Copy, Debug)]
pub enum BoundHit {
    Start { depth: f32 },
    End { depth: f32 },
}

impl BoundHit {
    #[inline]
    pub fn depth(&self) -> f32 {
        match self {
            BoundHit::Start { depth } | BoundHit::End { depth } => *depth,
        }
    }
}

#[derive(Clone, Debug)]
pub enum HitData {
    Trajectory(TrajectoryHits),
    Manoeuvre(ManoeuvreHit),
    Apsis(ApsisHit),
    Bound(BoundHit),
    Body(BodyHit),
    Ui,
}

impl HitData {
    #[inline]
    pub fn depth(&self) -> f32 {
        match self {
            HitData::Trajectory(hits) => hits.min_depth().unwrap_or(f32::MAX),
            HitData::Manoeuvre(hit) => hit.depth,
            HitData::Apsis(hit) => hit.depth,
            HitData::Bound(hit) => hit.depth(),
            HitData::Body(hit) => hit.depth,
            HitData::Ui => -1_000_000.0,
        }
    }
}

#[derive(Clone, Debug, Message)]
pub struct PointerHit(pub Entity, pub HitData);

#[derive(Resource, Default)]
pub struct PointerHover(pub Option<PointerHit>);

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum PickingSet {
    Backend,
    Hover,
}

pub struct CustomPickingPlugin;

impl Plugin for CustomPickingPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<PointerHit>()
            .init_resource::<PointerHover>()
            .configure_sets(
                PostUpdate,
                (
                    PickingSet::Backend
                        .after(crate::ui::PlotSystems::ComputePoints)
                        .run_if(not(
                            crate::camera::using_pointer.or(crate::ui::world_ui_using_pointer)
                        )),
                    PickingSet::Hover,
                )
                    .chain(),
            )
            .add_systems(
                PostUpdate,
                (
                    (
                        body_picking,
                        (
                            manoeuvre_marker_picking,
                            apsis_marker_picking,
                            bound_marker_picking,
                            trajectory_picking,
                        ),
                        // Some inputs are not always properly updated yet when this runs since it's
                        // not guaranteed to run after egui's end pass, but this is better than
                        // forcing all systems that need to run after picking to run after egui's
                        // end pass.
                        egui_picking,
                    )
                        .in_set(PickingSet::Backend),
                    update_pointer_hover.in_set(PickingSet::Hover),
                ),
            );
    }
}

pub const BODY_RADIUS: f32 = 1e-2;

fn body_picking(
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    query_can_select: Query<(Entity, &GlobalTransform, &Selectable)>,
    mut events: MessageWriter<PointerHit>,
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

pub const TRAJECTORY_LINE_SIZE: f32 = 6e-3;

pub fn trajectory_picking(
    ray_map: Res<RayMap>,
    perspective: Single<&Projection, With<Camera>>,
    query_plot: Query<(Entity, &PlotPoints)>,
    mut events: MessageWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((min_depth, (entity, mut picks))) = query_plot
        .iter()
        .map(|(entity, points)| {
            (
                entity,
                TrajectoryHits {
                    hits: points
                        .ray_distances(&ray)
                        .map(|(time, separation, depth)| TrajectoryHit {
                            time,
                            separation,
                            depth,
                        })
                        .filter(|hit| {
                            hit.separation < hit.depth * perspective.fov * TRAJECTORY_LINE_SIZE
                        })
                        .collect::<Vec<_>>(),
                },
            )
        })
        .filter_map(|(entity, hits)| Some((hits.min_depth()?, (entity, hits))))
        .min_by(|(a, _), (b, _)| a.total_cmp(b))
    {
        let max = min_depth * (1.0 + 0.01);
        picks.hits.retain(|hit| hit.depth <= max);

        events.write(PointerHit(entity, HitData::Trajectory(picks)));
    }
}

pub const MANOEUVRE_SIZE: f32 = 2e-2;

// We don't have a component for world positions like we have for trajectories with `PlotPoints`, so
// we recompute the manoeuvre positions here.
pub fn manoeuvre_marker_picking(
    ray_map: Res<RayMap>,
    query: Query<(&FlightPlan, &PlotSourceOf)>,
    query_plot: Query<(Entity, &PlotSource, &PlotPoints), With<BurnPlotSegment>>,
    query_trajectory: Query<&Trajectory>,
    perspective: Single<&Projection, With<Camera>>,
    mut events: MessageWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = &bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((entity, data)) = query
        .iter()
        .flat_map(|(flight_plan, source_of)| {
            query_plot
                .iter_many(source_of.iter())
                .flat_map(|(plot_entity, source, points)| {
                    flight_plan.burns.iter().filter_map(move |(&id, burn)| {
                        if !burn.is_active() {
                            return None;
                        }

                        let world_pos = points.evaluate(burn.start)?;
                        let relative = source.get_relative_trajectory(&query_trajectory).ok()?;
                        let distance = relative.position(burn.start)?.length();
                        let size = world_pos.distance(Vec3::from(ray.origin))
                            * perspective.fov
                            * MANOEUVRE_SIZE
                            / 2.0;
                        let size = size.min(distance as f32);
                        ray.sphere_intersection_at(&BoundingSphere::new(world_pos, size))
                            .map(|depth| (plot_entity, ManoeuvreHit { id, depth }))
                    })
                })
        })
        .min_by(|(_, a), (_, b)| a.depth.total_cmp(&b.depth))
    {
        events.write(PointerHit(entity, HitData::Manoeuvre(data)));
    }
}

pub const PERIAPSIS_SIZE: f32 = 1e-2;

pub fn apsis_marker_picking(
    ray_map: Res<RayMap>,
    query: Query<(&Apsides, &PlotSourceOf)>,
    query_plot: Query<(Entity, &PlotSource, &PlotPoints)>,
    query_trajectory: Query<&Trajectory>,
    perspective: Single<&Projection, With<Camera>>,
    mut events: MessageWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = &bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((entity, data)) = query
        .iter()
        .flat_map(|(apsides, source_of)| {
            query_plot
                .iter_many(source_of.iter())
                .flat_map(|(plot_entity, source, points)| {
                    apsides.iter().flat_map(move |&(apsis, _)| {
                        let world_pos = points.evaluate(apsis.time())?;
                        let relative = source.get_relative_trajectory(&query_trajectory).ok()?;
                        let distance = relative.position(apsis.time())?.length();
                        let size = world_pos.distance(Vec3::from(ray.origin))
                            * perspective.fov
                            * PERIAPSIS_SIZE;
                        let size = size.min(distance as f32);
                        ray.sphere_intersection_at(&BoundingSphere::new(world_pos, size))
                            .map(|depth| (plot_entity, ApsisHit { apsis, depth }))
                    })
                })
        })
        .min_by(|(_, a), (_, b)| a.depth.total_cmp(&b.depth))
    {
        events.write(PointerHit(entity, HitData::Apsis(data)));
    }
}

pub const START_SIZE: f32 = 6e-3;
pub const END_SIZE: f32 = 4e-3;

pub fn bound_marker_picking(
    ray_map: Res<RayMap>,
    query: Query<(&Trajectory, &PlotSourceOf)>,
    query_plot: Query<(Entity, &PlotSource, &PlotPoints)>,
    query_trajectory: Query<&Trajectory>,
    perspective: Single<&Projection, With<Camera>>,
    mut events: MessageWriter<PointerHit>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = &bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    if let Some((entity, data)) = query
        .iter()
        .flat_map(|(traj, source_of)| {
            query_plot
                .iter_many(source_of.iter())
                .flat_map(|(plot_entity, source, points)| {
                    [
                        (traj.start(), BoundHit::Start { depth: 0.0 }, START_SIZE),
                        (traj.end(), BoundHit::End { depth: 0.0 }, END_SIZE),
                    ]
                    .into_iter()
                    .filter_map(move |(time, mut hit, size)| {
                        let world_pos = points.evaluate(time)?;
                        let relative = source.get_relative_trajectory(&query_trajectory).ok()?;
                        let distance = relative.position(time)?.length();
                        let size =
                            world_pos.distance(Vec3::from(ray.origin)) * perspective.fov * size;
                        let size = size.min(distance as f32);

                        match &mut hit {
                            BoundHit::Start { depth: d } | BoundHit::End { depth: d } => {
                                *d = ray.sphere_intersection_at(&BoundingSphere::new(
                                    world_pos, size,
                                ))?;
                            }
                        }

                        Some((plot_entity, hit))
                    })
                })
        })
        .min_by(|(_, a), (_, b)| a.depth().total_cmp(&b.depth()))
    {
        events.write(PointerHit(entity, HitData::Bound(data)));
    }
}

pub fn egui_picking(
    window_to_egui_context_map: Res<WindowToEguiContextMap>,
    pointers: Query<&PointerLocation>,
    mut egui_context: Query<(Entity, &mut EguiContext, &EguiContextSettings, &Camera)>,
    mut output: MessageWriter<PointerHit>,
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
    perspective: Single<(&GlobalTransform, &Projection), With<Camera>>,
    query_can_select: Query<(&GlobalTransform, &Selectable)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *perspective else {
        unreachable!("Camera is not perspective");
    };

    for (transform, clickable) in &query_can_select {
        let direction = transform.translation() - camera_transform.translation();
        let radius = clickable.radius + direction.length() * perspective.fov * BODY_RADIUS;
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

fn update_pointer_hover(mut hover: ResMut<PointerHover>, mut events: MessageReader<PointerHit>) {
    hover.0 = events
        .read()
        .min_by(|PointerHit(_, a), PointerHit(_, b)| a.depth().total_cmp(&b.depth()))
        .cloned();
}
