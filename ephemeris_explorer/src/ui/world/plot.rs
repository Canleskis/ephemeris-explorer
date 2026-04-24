use crate::{
    MainState,
    dynamics::{PredictionTrajectory, StateVector, Trajectory},
    floating_origin::{Grid, GridExt},
    load::SystemRoot,
    simulation::SimulationTime,
    ui::WorldUiSet,
};

use bevy::prelude::*;
use bevy::{math::DVec3, math::bounding::RayCast3d};
use ephemeris::{BoundedTrajectory, EvaluateTrajectory, RelativeTrajectory};
use ftime::Epoch;

#[derive(Default, Debug, PartialEq, Eq, Component, Deref)]
#[relationship_target(relationship = PlotSource, linked_spawn)]
pub struct PlotSourceOf(Vec<Entity>);

#[derive(Clone, Copy, Debug, Component)]
#[relationship(relationship_target = PlotSourceOf)]
pub struct PlotSource {
    #[relationship]
    pub entity: Entity,
    pub reference: Option<Entity>,
}

impl PlotSource {
    #[inline]
    pub fn new(entity: Entity, reference: Option<Entity>) -> Self {
        Self { entity, reference }
    }

    #[inline]
    pub fn get_relative_trajectory<'w>(
        &self,
        query_trajectory: &'w Query<&Trajectory>,
    ) -> Result<RelativeTrajectory<&'w Trajectory>, bevy::ecs::query::QueryEntityError> {
        Ok(RelativeTrajectory::new(
            query_trajectory.get(self.entity)?,
            self.reference
                .map(|e| query_trajectory.get(e))
                .transpose()?,
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlotBound {
    None,
    Start,
    End,
}

impl PlotBound {
    #[inline]
    pub fn is_start(&self) -> bool {
        matches!(self, Self::Start)
    }

    #[inline]
    pub fn is_end(&self) -> bool {
        matches!(self, Self::End)
    }

    #[inline]
    pub fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }
}

#[derive(Clone, Copy, Component)]
#[require(PlotPoints)]
pub struct PlotConfig {
    pub enabled: bool,
    pub start: Epoch,
    pub end: Epoch,
    pub bound: PlotBound,
    pub color: Color,
    pub threshold: f32,
    pub max_points: usize,
    pub dashed: bool,
}

/// Global space position of the points for a plotted trajectory segment.
#[derive(Default, Debug, Component, Deref, DerefMut)]
pub struct PlotPoints(pub Vec<(Epoch, Vec3)>);

impl PlotPoints {
    // Adapted from Principia's PlotMethod3.
    // (https://github.com/mockingbirdnest/Principia/blob/2024080411-Klein/ksp_plugin/planetarium_body.hpp)
    // TODO: Rewrite this into more idiomatic Rust.
    #[inline]
    pub fn new(
        mut eval: impl FnMut(Epoch) -> Option<StateVector>,
        min: Epoch,
        max: Epoch,
        camera_transform: &GlobalTransform,
        tan2_angular_resolution: f64,
        max_points: usize,
    ) -> Result<Self, Epoch> {
        if max_points == 0 {
            return Ok(Self(Vec::new()));
        }

        let target_tan2_error = tan2_angular_resolution * tan2_angular_resolution;
        let mut previous_time = min;
        let mut previous = eval(previous_time).ok_or(previous_time)?;
        let mut delta = max - previous_time;
        let mut estimated_tan2_error = None::<f64>;

        let mut points = Vec::with_capacity(max_points.max(1024));
        points.push((previous_time, previous.position.as_vec3()));

        while previous_time < max && points.len() < max_points {
            let (t, current, next_error) = loop {
                if let Some(error) = estimated_tan2_error.filter(|&e| e > 0.0) {
                    delta = delta * 0.9 * (target_tan2_error / error).sqrt().sqrt();
                }

                let mut t = previous_time + delta;
                if t > max {
                    t = max;
                }

                delta = t - previous_time;

                let extrapolated_position =
                    previous.position + previous.velocity * delta.as_seconds();

                let current = eval(t).ok_or(t)?;
                let error =
                    angular_distance(camera_transform, extrapolated_position, current.position)
                        / 16.0;

                if error <= target_tan2_error {
                    break (t, current, error);
                }

                estimated_tan2_error = Some(error);
            };

            previous_time = t;
            previous = current;
            estimated_tan2_error = Some(next_error);
            points.push((t, previous.position.as_vec3()));
        }

        Ok(Self(points))
    }

    #[inline]
    pub fn evaluate(&self, at: Epoch) -> Option<Vec3> {
        if !self.contains(at) {
            return None;
        }

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

    #[inline]
    pub fn contains(&self, at: Epoch) -> bool {
        self.first().is_some_and(|(start, _)| start <= &at)
            && self.last().is_some_and(|(end, _)| end >= &at)
    }

    #[inline]
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

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct LineGizmoConfigGroup;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct DashedLineGizmoConfigGroup;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct MarkerGizmoConfigGroup;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub enum PlotSystems {
    ComputePoints,
}

#[derive(Default)]
pub struct PlotPlugin;

impl Plugin for PlotPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<LineGizmoConfigGroup>()
            .init_gizmo_group::<DashedLineGizmoConfigGroup>()
            .init_gizmo_group::<MarkerGizmoConfigGroup>()
            .add_systems(
                PostUpdate,
                (
                    compute_plot_points_parallel.in_set(PlotSystems::ComputePoints),
                    (
                        // plot_knots,
                        plot_lines,
                    ),
                )
                    .chain()
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running))
                    .after(TransformSystems::Propagate),
            );

        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        let dashed_config = store.config_mut::<DashedLineGizmoConfigGroup>().0;
        dashed_config.line.style = GizmoLineStyle::Dashed {
            gap_scale: 6.0,
            line_scale: 10.0,
        };
    }
}

// TODO: Find a way to reuse allocations.
pub fn compute_plot_points_parallel(
    commands: ParallelCommands,
    sim_time: Res<SimulationTime>,
    mut query_plot: Query<(Entity, &PlotConfig, &PlotSource)>,
    query_traj: Query<&Trajectory>,
    root: Single<&Grid, With<SystemRoot>>,
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

            let Ok(trajectory) = query_traj.get(source.entity) else {
                return;
            };
            let Ok(reference) = source.reference.map(|e| query_traj.get(e)).transpose() else {
                return;
            };
            // Lock now so that upcoming evaluations don't need to repeatedly acquire the lock.
            let trajectory = trajectory.read();
            let reference = reference.map(|r| r.read());
            let relative = RelativeTrajectory::new(&*trajectory, reference.as_deref());

            if plot.enabled && !relative.is_empty() {
                const ARC_MINUTE: f32 = 0.000290888;
                let tan2_angular_resolution =
                    (plot.threshold * ARC_MINUTE * perspective.fov) as f64;

                let start = relative.start();
                let end = relative.end();

                let current = sim_time.current();
                let current_clamped = current.clamp(relative.start(), relative.end());

                let mut min = plot.start.clamp(start, end);
                let mut max = plot.end.clamp(start, end);
                match plot.bound {
                    PlotBound::Start => min = min.max(current_clamped),
                    PlotBound::End => max = max.min(current_clamped),
                    PlotBound::None => (),
                }

                if min >= max {
                    return;
                }

                let translation = StateVector::from_position(
                    relative
                        .reference
                        .map(|r| r.position(current.clamp(r.start(), r.end())).unwrap())
                        .unwrap_or_default(),
                );

                let new_points = PlotPoints::new(
                    |t| Some(root.to_global_sv(relative.state_vector(t)? + translation)),
                    min,
                    max,
                    camera_transform,
                    tan2_angular_resolution,
                    plot.max_points,
                )
                .unwrap_or_else(|t| panic!("Failed to evaluate state vector at {t}"));

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

fn plot_lines(
    mut line_gizmos: Gizmos<LineGizmoConfigGroup>,
    mut dashed_gizmos: Gizmos<DashedLineGizmoConfigGroup>,
    query: Query<(&PlotPoints, &PlotConfig)>,
) {
    for (points, plot) in query.iter() {
        if plot.dashed {
            dashed_gizmos.linestrip(points.iter().map(|(_, p)| *p), plot.color);
        } else {
            line_gizmos.linestrip(points.iter().map(|(_, p)| *p), plot.color);
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
        let Ok(trajectory) = query_trajectory.get(source.entity) else {
            continue;
        };
        let traj = trajectory.read();

        match &*trajectory.read() {
            PredictionTrajectory::UniformSpline(traj) => {
                let knots =
                    (0..traj.segment_count()).map(|i| traj.start() + traj.interval() * i as f64);
                plot_knots(&mut gizmos, *camera, knots, points);
            }
            PredictionTrajectory::CubicHermiteSpline(traj) => {
                let knots = traj.points().iter().map(|(t, _)| *t);
                plot_knots(&mut gizmos, *camera, knots, points);
            }
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
