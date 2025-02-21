use crate::{
    flight_plan::FlightPlan,
    floating_origin::{BigSpace, Grid},
    prediction::{
        DiscreteStatesBuilder, RelativeTrajectory, StateVector, Trajectory, TrajectoryData,
    },
    selection::Selectable,
    time::SimulationTime,
    ui::WorldUiSet,
    MainState,
};

use bevy::prelude::*;
use bevy::{math::DVec3, picking::backend::ray::RayMap};
use hifitime::Epoch;

#[derive(Component)]
#[require(PlotPoints)]
pub struct TrajectoryPlot {
    /// For now this should always refers to the parent of the plot, but eventually when we have
    /// entity-entity relationships we can have a `SourceOf` component on entities that are
    /// referenced by a `TrajectoryPlot`.
    pub source: Entity,
    pub enabled: bool,
    pub start: Epoch,
    pub end: Epoch,
    pub color: Color,
    pub threshold: f32,
    pub max_points: usize,
    pub reference: Option<Entity>,
}

pub type SourceOf = Children;

impl TrajectoryPlot {
    #[inline]
    pub fn relative_trajectory<'w>(
        &self,
        query_trajectory: &'w Query<&Trajectory>,
    ) -> Option<RelativeTrajectory<&'w Trajectory>> {
        Some(RelativeTrajectory::new(
            query_trajectory.get(self.source).ok()?,
            self.reference.and_then(|e| query_trajectory.get(e).ok()),
        ))
    }
}

/// Global space position of the points of a trajectory.
#[derive(Default, Debug, Component, Deref, DerefMut)]
pub struct PlotPoints(pub Vec<(Epoch, Vec3)>);

#[derive(Clone, Event)]
pub struct TrajectoryHitPoint {
    pub entity: Entity,
    pub time: Epoch,
    pub separation: f32,
    pub distance: f32,
}

#[derive(Default)]
pub struct PlotPlugin;

impl Plugin for PlotPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<TrajectoryHitPoint>()
            .add_systems(Startup, setup_gizmos)
            .add_systems(
                PostUpdate,
                (
                    compute_plot_points_parallel,
                    (
                        plot_trajectories,
                        plot_burns,
                        trajectory_picking.run_if(not(
                            crate::ui::is_using_pointer.or(crate::camera::is_using_pointer)
                        )),
                    ),
                )
                    .chain()
                    .in_set(WorldUiSet)
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

pub fn transform_vector3(v: DVec3, root: &Grid) -> Vec3 {
    root.local_floating_origin()
        .grid_transform()
        .transform_vector3(v)
        .as_vec3()
}

pub fn to_global_pos(pos: DVec3, root: &Grid) -> Vec3 {
    // TODO: There is probably a simpler way to do this.
    let (cell, translation) = root.translation_to_grid(pos);
    root.global_transform(&cell, &Transform::from_translation(translation))
        .translation()
}

pub fn to_global_sv(sv: StateVector<DVec3>, root: &Grid) -> StateVector<DVec3> {
    StateVector {
        velocity: transform_vector3(sv.velocity, root).as_dvec3(),
        position: to_global_pos(sv.position, root).as_dvec3(),
    }
}

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

pub fn compute_plot_points_parallel(
    commands: ParallelCommands,
    sim_time: Res<SimulationTime>,
    mut query_plot: Query<(Entity, &TrajectoryPlot)>,
    query_trajectory: Query<&Trajectory>,
    root: Single<&Grid, With<BigSpace>>,
    camera: Query<(&GlobalTransform, &Projection)>,
) {
    let Ok((camera_transform, proj)) = camera.get_single() else {
        return;
    };

    query_plot.par_iter_mut().for_each(|(entity, plot)| {
        commands.command_scope(move |mut commands| {
            commands.queue(move |world: &mut World| {
                if let Some(mut points) = world.get_mut::<PlotPoints>(entity) {
                    points.clear();
                }
            });
        });

        if let Some(relative) = plot.relative_trajectory(&query_trajectory) {
            if plot.enabled && !relative.is_empty() && plot.start < plot.end {
                let tan2_angular_resolution = match proj {
                    Projection::Perspective(p) => {
                        const ARC_MINUTE: f32 = 0.000290888;
                        plot.threshold * ARC_MINUTE * p.fov
                    }
                    _ => unreachable!(),
                } as f64;

                let start = relative.start();
                let end = relative.end();

                let min = plot.start.clamp(start, end);
                let max = plot.end.clamp(start, end);

                let current = sim_time.current().clamp(relative.start(), relative.end());
                let translation = StateVector::from_position(
                    relative
                        .reference
                        .map(|r| r.position(current).unwrap())
                        .unwrap_or_default(),
                );

                let new_points = PlotPoints::new(
                    |at| to_global_sv(relative.state_vector(at).unwrap() + translation, *root),
                    min,
                    max,
                    camera_transform,
                    tan2_angular_resolution,
                    plot.max_points,
                );

                commands.command_scope(move |mut commands| {
                    commands.queue(move |world: &mut World| {
                        if let Some(mut points) = world.get_mut::<PlotPoints>(entity) {
                            *points = new_points;
                        }
                    });
                });
            }
        };
    });
}

pub const PICK_THRESHOLD: f32 = 6e-3;

pub fn trajectory_picking(
    ray_map: Res<RayMap>,
    query_clickable: Query<(&GlobalTransform, &Selectable)>,
    query_camera: Query<&Projection, With<Camera>>,
    query_points: Query<(Entity, &PlotPoints)>,
    mut events: EventWriter<TrajectoryHitPoint>,
) {
    let Ok(proj) = query_camera.get_single() else {
        return;
    };

    let fov = match proj {
        Projection::Perspective(p) => p.fov,
        _ => return,
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let mut ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

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
    }
}

fn plot_burns(
    mut gizmos: Gizmos,
    query: Query<(&Trajectory, &FlightPlan, &SourceOf, &DiscreteStatesBuilder)>,
    query_plot: Query<(&PlotPoints, &TrajectoryPlot)>,
    root: Single<&Grid, With<BigSpace>>,
    camera: Query<&GlobalTransform, With<Camera>>,
) {
    let camera_transform = camera.single();

    for (trajectory, flight_plan, source_of, builder) in query.iter() {
        for (points, plot) in query_plot.iter_many(source_of) {
            if !plot.enabled || trajectory.is_empty() {
                continue;
            }

            for burn in flight_plan.burns.iter() {
                if !burn.enabled || burn.overlaps {
                    continue;
                }

                let Some(sv) = trajectory.state_vector(burn.start) else {
                    continue;
                };
                let (prograde, radial, normal) = burn
                    .reference_frame()
                    .direction((burn.start, sv), builder.context());
                let prograde = transform_vector3(prograde, *root);
                let radial = transform_vector3(radial, *root);
                let normal = transform_vector3(normal, *root);

                let Some(world_pos) = points.evaluate(burn.start) else {
                    continue;
                };
                let direction = camera_transform.translation() - world_pos;
                let size = direction.length() * 0.02;
                gizmos.arrow(world_pos, world_pos + prograde * size, LinearRgba::GREEN);
                gizmos.arrow(world_pos, world_pos + normal * size, LinearRgba::BLUE);
                gizmos.arrow(world_pos, world_pos + radial * size, LinearRgba::RED);
            }
        }
    }
}

#[expect(unused)]
fn plot_debug_points_discrete(
    mut gizmos: Gizmos,
    sim_time: Res<SimulationTime>,
    query_plot: Query<&TrajectoryPlot>,
    query_trajectory: Query<&Trajectory>,
    root: Single<&Grid, With<BigSpace>>,
    query_camera: Query<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, proj) = query_camera.single();
    let fov = match proj {
        Projection::Perspective(p) => p.fov,
        _ => return,
    };

    for plot in query_plot.iter() {
        let Ok(trajectory) = query_trajectory.get(plot.source) else {
            continue;
        };
        let locked_trajectory = trajectory.read();
        let Some(states) = locked_trajectory
            .as_any()
            .downcast_ref::<crate::prediction::DiscreteStates>()
        else {
            continue;
        };
        let reference_trajectory = plot.reference.and_then(|e| query_trajectory.get(e).ok());
        let relative = RelativeTrajectory::new(trajectory, reference_trajectory);

        let current = sim_time.current().clamp(relative.start(), relative.end());
        let translation = relative
            .reference
            .map(|r| r.position(current).unwrap())
            .unwrap_or_default();

        for (t, _) in states.points() {
            if t < &plot.start || t > &plot.end {
                continue;
            }

            let Some(position) = relative.position(*t) else {
                continue;
            };
            let position = to_global_pos(position + translation, *root);
            let direction = camera_transform.translation() - position;
            let size = direction.length() * 4e-3 * fov;
            gizmos.circle(
                Transform::from_translation(position)
                    .looking_to(direction, Vec3::Y)
                    .to_isometry(),
                size,
                LinearRgba::WHITE,
            );
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
