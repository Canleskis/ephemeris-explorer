use crate::{
    compound_trajectory::{CompoundTrajectoryData, TrajectoryReferenceTranslated},
    flight_plan::FlightPlan,
    floating_origin::{BigSpace, ReferenceFrame},
    prediction::{DiscreteStatesBuilder, PredictionCtx, StateVector, Trajectory, TrajectoryData},
    selection::Clickable,
    MainState,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::Epoch;

#[derive(Component, Default)]
pub struct TrajectoryPlot {
    pub enabled: bool,
    pub color: Color,
    pub start: Epoch,
    pub end: Epoch,
    pub threshold: f32,
    pub max_points: usize,
}

/// Global space position of the points of a trajectory.
#[derive(Default, Debug, Component, Deref, DerefMut)]
pub struct PlotPoints(pub Vec<(Epoch, Vec3)>);

impl PlotPoints {
    // Adapted from Principia's PlotMethod3.
    // (https://github.com/mockingbirdnest/Principia/blob/2024080411-Klein/ksp_plugin/planetarium_body.hpp)
    pub fn new(
        eval: impl Fn(Epoch) -> StateVector<DVec3>,
        min: Epoch,
        max: Epoch,
        camera_transform: &GlobalTransform,
        tan2_angular_resolution: f64,
        max_points: usize,
    ) -> Self {
        let tan2_angular_resolution = tan2_angular_resolution * tan2_angular_resolution;
        let final_time = max;
        let mut previous_time = min;

        let mut previous = eval(previous_time);
        let mut delta = (final_time - previous_time).to_seconds();

        let mut points = Vec::new();
        points.push((previous_time, previous.position.as_vec3()));

        let mut t: Epoch;
        let mut estimated_tan2_error = None::<f64>;
        let mut current: StateVector<DVec3>;

        while points.len() < max_points && previous_time < final_time {
            loop {
                if let Some(estimated_tan2_error) = estimated_tan2_error {
                    delta *= 0.9
                        * (tan2_angular_resolution / estimated_tan2_error)
                            .sqrt()
                            .sqrt();
                }

                t = previous_time + delta;

                if t > final_time {
                    t = final_time;
                    delta = (t - previous_time).to_seconds();
                }

                let extrapolated_position = previous.position + previous.velocity * delta;
                current = eval(t);

                // Prevent catastrophic retries
                if estimated_tan2_error.is_some_and(|error| error == 0.0) {
                    break;
                }

                estimated_tan2_error = Some(
                    angular_distance(camera_transform, extrapolated_position, current.position)
                        / 16.0,
                );

                if estimated_tan2_error.is_some_and(|error| error <= tan2_angular_resolution) {
                    break;
                }
            }

            previous_time = t;
            previous = current;
            points.push((t, current.position.as_vec3()));
        }

        Self(points)
    }

    pub fn evaluate(&self, at: Epoch) -> Option<Vec3> {
        match self.binary_search_by(|(t, _)| t.cmp(&at)) {
            Ok(i) => Some(self[i].1),
            Err(i) => {
                let &(t1, p1) = self.get(i.checked_sub(1)?)?;
                let &(t2, p2) = self.get(i)?;
                let t = ((at - t1).to_seconds() / (t2 - t1).to_seconds()) as f32;

                Some(p1.lerp(p2, t))
            }
        }
    }
}

#[derive(Clone, Event)]
pub struct TrajectoryHitPoint {
    pub entity: Entity,
    pub time: Epoch,
    pub separation: f32,
    pub distance: f32,
}

#[derive(Default)]
pub struct TrajectoryPlotPlugin;

impl Plugin for TrajectoryPlotPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<TrajectoryHitPoint>()
            .add_systems(Startup, setup_gizmos)
            .add_systems(
                PostUpdate,
                (
                    compute_plot_points::<TrajectoryReferenceTranslated>,
                    (
                        plot_trajectories,
                        plot_burns,
                        trajectory_picking
                            .run_if(not(crate::ui::is_using_pointer
                                .or_else(crate::camera::is_using_pointer))),
                    ),
                )
                    .chain()
                    .run_if(in_state(MainState::Running))
                    .after(bevy::transform::TransformSystem::TransformPropagate),
            );
    }
}

fn setup_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    for (_, config, _) in config_store.iter_mut() {
        config.line_width = 1.0;
    }
}

pub fn transform_vector3(v: DVec3, root: &ReferenceFrame) -> Vec3 {
    root.local_floating_origin()
        .reference_frame_transform()
        .transform_vector3(v)
        .as_vec3()
}

pub fn to_global_pos(pos: DVec3, root: &ReferenceFrame) -> Vec3 {
    let (cell, translation) = root.translation_to_grid(pos);
    root.global_transform(&cell, &Transform::from_translation(translation))
        .translation()
}

pub fn to_global_sv(sv: StateVector<DVec3>, root: &ReferenceFrame) -> StateVector<DVec3> {
    StateVector {
        velocity: transform_vector3(sv.velocity, root).as_dvec3(),
        position: to_global_pos(sv.position, root).as_dvec3(),
    }
}

pub fn compute_plot_points<C>(
    query_trajectory: Query<C::QueryData<'_>>,
    mut query: Query<(&mut PlotPoints, &C, &TrajectoryPlot)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
    camera: Query<(&GlobalTransform, &Projection)>,
) where
    C: CompoundTrajectoryData,
{
    let Ok((camera_transform, proj)) = camera.get_single() else {
        return;
    };
    let root = root.single();

    for (mut points, data, plot) in query.iter_mut() {
        points.clear();

        let Some(trajectory) = data.fetch(&query_trajectory) else {
            continue;
        };

        if !plot.enabled || trajectory.is_empty() || plot.start >= plot.end {
            continue;
        }

        let threshold = match proj {
            Projection::Perspective(p) => {
                const ARC_MINUTE: f32 = 0.000290888;
                plot.threshold * ARC_MINUTE * p.fov
            }
            _ => unreachable!(),
        } as f64;

        let start = trajectory.start();
        let end = trajectory.end();

        let min = plot.start.clamp(start, end);
        let max = plot.end.clamp(start, end);

        let eval = |at| to_global_sv(trajectory.state_vector(at).unwrap(), root);

        *points = PlotPoints::new(eval, min, max, camera_transform, threshold, plot.max_points)
    }
}

pub const PICK_THRESHOLD: f32 = 6e-3;

pub fn trajectory_picking(
    query_window: Query<&Window>,
    query_clickable: Query<(&GlobalTransform, &Clickable)>,
    query_camera: Query<(&GlobalTransform, &Camera, &Projection)>,
    query_points: Query<(Entity, &PlotPoints)>,
    mut events: EventWriter<TrajectoryHitPoint>,
) {
    let Ok(window) = query_window.get_single() else {
        return;
    };
    let Ok((camera_transform, camera, proj)) = query_camera.get_single() else {
        return;
    };

    let fov = match proj {
        Projection::Perspective(p) => p.fov,
        _ => return,
    };

    let ray = window.cursor_position().and_then(|position| {
        Some(bevy::math::bounding::RayCast3d::from_ray(
            camera.viewport_to_world(camera_transform, position)?,
            f32::MAX,
        ))
    });

    let Some(mut ray) = ray else {
        return;
    };

    ray.max = query_clickable
        .iter()
        .filter_map(|(transform, clickable)| {
            let distance = transform.translation().distance(Vec3::from(ray.origin));
            ray.sphere_intersection_at(&bevy::math::bounding::BoundingSphere::new(
                transform.translation(),
                clickable.radius + distance * fov * 1e-2,
            ))
        })
        .fold(f32::MAX, f32::min);

    for (entity, points) in query_points.iter() {
        for ((t1, p1), (t2, p2)) in points.iter().zip(points.iter().skip(1)) {
            let p1 = bevy::math::Vec3A::from(*p1);
            let p2 = bevy::math::Vec3A::from(*p2);

            let seg_direction = p2 - p1;
            let w0 = ray.origin - p1;

            let a = ray.direction.dot(*ray.direction);
            let b = ray.direction.dot(seg_direction);
            let c = seg_direction.dot(seg_direction);
            let d = ray.direction.dot(w0);
            let e = seg_direction.dot(w0);

            let denom = a * c - b * b;

            let t_ray = if denom.abs() > f32::EPSILON {
                (b * e - c * d) / denom
            } else {
                0.0
            }
            .max(0.0);
            let t_seg = (a * e - b * d) / denom;

            if t_ray > ray.max {
                continue;
            }

            if !(0.0..=1.0).contains(&t_seg) {
                continue;
            }

            let closest_point_seg = p1 + seg_direction * t_seg;
            let closest_point_ray = ray.origin + ray.direction * t_ray;
            let separation = (closest_point_ray - closest_point_seg).length();

            if separation < t_ray * fov * PICK_THRESHOLD {
                events.send(TrajectoryHitPoint {
                    entity,
                    time: *t1 + (*t2 - *t1) * t_seg as f64,
                    separation,
                    distance: t_ray,
                });
            }
        }
    }
}

fn plot_trajectories(mut gizmos: Gizmos, query: Query<(&PlotPoints, &TrajectoryPlot)>) {
    for (points, plot) in query.iter() {
        gizmos.linestrip(points.iter().map(|(_, p)| *p), plot.color);
        // for &p in &points {
        //     let dir = camera_transform.translation() - p;
        //     let size = dir.length() * 0.01;
        //     gizmos
        //         .circle(p, Dir3::new(dir).unwrap(), size, plot.color)
        //         .resolution(16);
        // }
    }
}

fn plot_burns(
    mut gizmos: Gizmos,
    query: Query<(&Trajectory, &TrajectoryPlot, &PlotPoints, &FlightPlan)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
    camera: Query<&GlobalTransform, With<Camera>>,
    ctx: Res<PredictionCtx<DiscreteStatesBuilder>>,
) {
    let camera_transform = camera.single();
    let root = root.single();

    for (trajectory, plot, points, flight_plan) in query.iter() {
        if !plot.enabled || trajectory.is_empty() {
            continue;
        }

        for burn in flight_plan.burns.iter() {
            if !burn.enabled || burn.overlaps {
                continue;
            }
            let Some(pos) = points.evaluate(burn.start) else {
                continue;
            };
            let Some(sv) = trajectory.state_vector(burn.start) else {
                continue;
            };

            let (prograde, radial, normal) =
                burn.reference_frame().direction((burn.start, sv), &ctx);
            let prograde = transform_vector3(prograde, root);
            let radial = transform_vector3(radial, root);
            let normal = transform_vector3(normal, root);

            let direction = camera_transform.translation() - pos;
            let size = direction.length() * 0.02;
            gizmos.arrow(pos, pos + prograde * size, LinearRgba::GREEN);
            gizmos.arrow(pos, pos + normal * size, LinearRgba::BLUE);
            gizmos.arrow(pos, pos + radial * size, LinearRgba::RED);
        }
    }
}

#[inline]
fn angular_distance(camera_transform: &GlobalTransform, p1: DVec3, p2: DVec3) -> f64 {
    let camera_position = camera_transform.translation().as_dvec3();
    let v1 = (p1 - camera_position).normalize();
    let v2 = (p2 - camera_position).normalize();

    let wedge = v1.cross(v2);
    wedge.length_squared() / v1.dot(v2).powi(2)
}
