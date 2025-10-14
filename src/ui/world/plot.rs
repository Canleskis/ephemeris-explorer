use crate::{
    MainState,
    dynamics::{CubicHermiteSplineSamples, UniformSpline},
    flight_plan::FlightPlan,
    floating_origin::{BigSpace, Grid},
    prediction::Trajectory,
    simulation::SimulationTime,
    ui::{MANOEUVRE_SIZE, WorldUiSet},
};

use bevy::prelude::*;
use bevy::{math::DVec3, math::bounding::RayCast3d};
use ephemeris::{
    BoundedTrajectory, EvaluateTrajectory, ManoeuvreFrame, RelativeTrajectory, StateVector,
};
use ftime::Epoch;

#[derive(Default, Debug, PartialEq, Eq, Component, Deref)]
#[relationship_target(relationship = PlotSource)]
pub struct PlotSourceOf(Vec<Entity>);

#[derive(Component)]
#[relationship(relationship_target = PlotSourceOf)]
pub struct PlotSource(pub Entity);

impl PlotSource {
    pub fn entity(&self) -> Entity {
        self.0
    }
}

#[derive(Component)]
#[require(PlotPoints)]
pub struct PlotConfig {
    pub enabled: bool,
    pub start: Epoch,
    pub end: Epoch,
    pub color: Color,
    pub threshold: f32,
    pub max_points: usize,
    pub reference: Option<Entity>,
}

impl PlotConfig {
    #[inline]
    pub fn get_relative_trajectory<'w>(
        &self,
        source: Entity,
        query_trajectory: &'w Query<&Trajectory>,
    ) -> Option<RelativeTrajectory<&'w Trajectory>> {
        Some(RelativeTrajectory::new(
            query_trajectory.get(source).ok()?,
            self.reference.and_then(|e| query_trajectory.get(e).ok()),
        ))
    }
}

/// Global space position of the points for a plotted trajectory.
#[derive(Default, Debug, Component, Deref, DerefMut)]
pub struct PlotPoints(pub Vec<(Epoch, Vec3)>);

impl PlotPoints {
    // Adapted from Principia's PlotMethod3.
    // (https://github.com/mockingbirdnest/Principia/blob/2024080411-Klein/ksp_plugin/planetarium_body.hpp)
    // TODO: Rewrite this into more idiomatic Rust.
    #[inline]
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
        let mut delta = final_time - previous_time;

        let mut points = Vec::new();
        points.push((previous_time, previous.position.as_vec3()));

        let mut t: Epoch;
        let mut estimated_tan2_error = None::<f64>;
        let mut current: StateVector<DVec3>;

        while points.len() < max_points && previous_time < final_time {
            loop {
                if let Some(estimated_tan2_error) = estimated_tan2_error {
                    delta = delta
                        * 0.9
                        * (tan2_angular_resolution / estimated_tan2_error)
                            .sqrt()
                            .sqrt();
                }

                t = previous_time + delta;

                if t > final_time {
                    t = final_time;
                    delta = t - previous_time;
                }

                let extrapolated_position =
                    previous.position + previous.velocity * delta.as_seconds();
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
                let t = ((at - t1).as_seconds() / (t2 - t1).as_seconds()) as f32;

                Some(p1.lerp(p2, t))
            }
        }
    }

    pub fn ray_distances<'a>(
        &'a self,
        ray: &'a RayCast3d,
    ) -> impl Iterator<Item = (Epoch, f32, f32)> + 'a {
        self.iter()
            .zip(self.iter().skip(1))
            .filter_map(|((t1, p1), (t2, p2))| {
                let p1 = bevy::math::Vec3A::from(*p1);
                let p2 = bevy::math::Vec3A::from(*p2);

                let u = p2 - p1;
                let v = *ray.direction;
                let w = p1 - ray.origin;

                let a = u.dot(u);
                let b = u.dot(v);
                let c = v.dot(v);
                let d = u.dot(w);
                let e = v.dot(w);

                let denom = a * c - b * b;
                let t_seg: f32;
                let t_ray: f32;

                // Parallel case
                if denom < 1e-7 {
                    t_seg = 0.0;
                    t_ray = if b > c { d / b } else { e / c };
                } else {
                    t_seg = (b * e - c * d) / denom;
                    t_ray = (a * e - b * d) / denom;
                }

                if t_ray > ray.max {
                    return None;
                }

                if !(0.0..=1.0).contains(&t_seg) {
                    return None;
                }

                let p_seg = p1 + u * t_seg;
                let p_ray = ray.origin + v * t_ray;
                let separation = p_ray.distance(p_seg);

                Some((*t1 + (*t2 - *t1) * t_seg as f64, separation, t_ray))
            })
    }
}

#[derive(Clone, Copy, Event)]
pub struct TrajectoryHitPoint {
    pub entity: Entity,
    pub time: Epoch,
    // Closest approach distance between the ray and the trajectory.
    pub separation: f32,
    // Distance along the ray.
    pub distance: f32,
}

#[derive(Clone, Copy, Event)]
pub struct ManoeuvreHitPoint {
    pub plot_entity: Entity,
    pub flight_plan_entity: Entity,
    pub id: uuid::Uuid,
    // Distance along the ray.
    pub distance: f32,
}

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct MarkerGizmoConfigGroup;

#[derive(Default)]
pub struct PlotPlugin;

impl Plugin for PlotPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<MarkerGizmoConfigGroup>()
            .add_event::<TrajectoryHitPoint>()
            .add_event::<ManoeuvreHitPoint>()
            .add_systems(
                PostUpdate,
                (
                    compute_plot_points_parallel,
                    (
                        // plot_knots,
                        plot_trajectories,
                        plot_manoeuvres,
                    ),
                )
                    .chain()
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running))
                    .after(bevy::transform::TransformSystem::TransformPropagate),
            );
    }
}

#[inline]
pub fn transform_vector3(v: DVec3, root: &Grid) -> Vec3 {
    root.local_floating_origin()
        .grid_transform()
        .transform_vector3(v)
        .as_vec3()
}

#[inline]
pub fn to_global_pos(pos: DVec3, root: &Grid) -> Vec3 {
    // TODO: There is probably a simpler way to do this.
    let (cell, translation) = root.translation_to_grid(pos);
    root.global_transform(&cell, &Transform::from_translation(translation))
        .translation()
}

#[inline]
pub fn to_global_sv(sv: StateVector<DVec3>, root: &Grid) -> StateVector<DVec3> {
    StateVector {
        velocity: transform_vector3(sv.velocity, root).as_dvec3(),
        position: to_global_pos(sv.position, root).as_dvec3(),
    }
}

pub fn compute_plot_points_parallel(
    commands: ParallelCommands,
    sim_time: Res<SimulationTime>,
    mut query_plot: Query<(Entity, &PlotConfig, &PlotSource)>,
    query_trajectory: Query<&Trajectory>,
    root: Single<&Grid, With<BigSpace>>,
    camera: Single<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    query_plot
        .par_iter_mut()
        .for_each(|(entity, plot, source)| {
            commands.command_scope(move |mut commands| {
                commands.queue(move |world: &mut World| {
                    if let Some(mut points) = world.get_mut::<PlotPoints>(entity) {
                        points.clear();
                    }
                });
            });

            if let Some(relative) = plot.get_relative_trajectory(source.entity(), &query_trajectory)
                && plot.enabled
                && !relative.is_empty()
                && plot.start < plot.end
            {
                const ARC_MINUTE: f32 = 0.000290888;
                let tan2_angular_resolution =
                    (plot.threshold * ARC_MINUTE * perspective.fov) as f64;

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

                // TODO: Make new fallible
                let new_points = PlotPoints::new(
                    |at| {
                        to_global_sv(
                            relative
                                .state_vector(at)
                                .unwrap_or_else(|| panic!("No state vector at {at}"))
                                + translation,
                            *root,
                        )
                    },
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
            };
        });
}

fn plot_trajectories(mut gizmos: Gizmos, query: Query<(&PlotPoints, &PlotConfig)>) {
    for (points, plot) in query.iter() {
        gizmos.linestrip(points.iter().map(|(_, p)| *p), plot.color);
    }
}

fn plot_manoeuvres(
    mut gizmos: Gizmos<MarkerGizmoConfigGroup>,
    query: Query<(&Trajectory, &FlightPlan, &PlotSourceOf)>,
    query_trajectory: Query<&Trajectory>,
    query_plot: Query<(&PlotPoints, &PlotConfig)>,
    root: Single<&Grid, With<BigSpace>>,
    camera: Single<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, Projection::Perspective(perspective)) = *camera else {
        unreachable!("Camera is not perspective");
    };

    for (trajectory, flight_plan, source_of) in query.iter() {
        for (points, plot) in query_plot.iter_many(source_of.iter()) {
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
                let Some(transform) = burn
                    .try_reference_frame(&query_trajectory)
                    .expect("invalid burn reference entity")
                    .transform(burn.start, &sv)
                else {
                    continue;
                };
                let prograde = transform_vector3(transform.0.x_axis, *root);
                let radial = transform_vector3(transform.0.y_axis, *root);
                let normal = transform_vector3(transform.0.z_axis, *root);

                let Some(world_pos) = points.evaluate(burn.start) else {
                    continue;
                };
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

#[expect(unused)]
fn plot_knots(
    mut gizmos: Gizmos,
    query_plot: Query<(&PlotConfig, &PlotPoints, &PlotSource)>,
    query_trajectory: Query<&Trajectory>,
    camera: Single<(&GlobalTransform, &Projection)>,
) {
    fn plot_knots(
        gizmos: &mut Gizmos,
        camera: (&GlobalTransform, &Projection),
        knots: impl Iterator<Item = Epoch>,
        plot_points: &PlotPoints,
    ) {
        let (camera_transform, Projection::Perspective(perspective)) = camera else {
            unreachable!("Camera is not perspective");
        };

        for t in knots {
            let Some(position) = plot_points.evaluate(t) else {
                continue;
            };
            let direction = camera_transform.translation() - position;
            let size = direction.length() * 4e-3 * perspective.fov;
            gizmos.circle(
                Transform::from_translation(position)
                    .looking_to(direction, Vec3::Y)
                    .to_isometry(),
                size,
                LinearRgba::WHITE,
            );
        }
    }

    for (plot, points, source) in query_plot.iter() {
        if !plot.enabled {
            continue;
        }
        let Ok(trajectory) = query_trajectory.get(source.entity()) else {
            continue;
        };
        let traj = trajectory.read();

        if let Some(traj) = traj.as_any().downcast_ref::<CubicHermiteSplineSamples>() {
            let knots = traj.points().iter().map(|(t, _)| *t);
            plot_knots(&mut gizmos, *camera, knots, points);
        };

        if let Some(traj) = traj.as_any().downcast_ref::<UniformSpline>() {
            let knots = (0..traj.len()).map(|i| traj.start() + traj.interval() * i as f64);
            plot_knots(&mut gizmos, *camera, knots, points);
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
