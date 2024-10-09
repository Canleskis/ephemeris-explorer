use crate::{
    ephemerides::{StateVector, Trajectory},
    floating_origin::{BigSpace, ReferenceFrame},
    MainState,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::Epoch;

#[derive(Resource, Default)]
pub struct EphemerisPlotConfig {
    pub current_time: Epoch,
    pub plot_history: bool,
}

#[derive(Component, Default)]
pub struct EphemerisPlot {
    pub enabled: bool,
    pub color: Color,
    pub start: Epoch,
    pub end: Epoch,
    pub threshold: f32,
    pub reference: Option<Entity>,
}

#[derive(Default)]
pub struct EphemerisPlotPlugin;

impl Plugin for EphemerisPlotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PostUpdate,
            plot_trajectories
                .run_if(in_state(MainState::Running))
                .after(bevy::transform::TransformSystem::TransformPropagate),
        );
    }
}

fn plot_trajectories(
    mut gizmos: Gizmos,
    config: Res<EphemerisPlotConfig>,
    query: Query<(&Trajectory, &EphemerisPlot)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
    camera: Query<(&GlobalTransform, &Projection)>,
) {
    let (camera_transform, proj) = camera.single();
    let root = root.single();

    let global = |sv: StateVector<DVec3>| StateVector {
        // The velocity might need to be rotated.
        velocity: root
            .local_floating_origin()
            .reference_frame_transform()
            .transform_vector3(sv.velocity)
            .as_vec3(),
        position: {
            // Maybe there is a less convoluted way to do this?
            let (cell, translation) = root.translation_to_grid(sv.position);
            root.global_transform(&cell, &Transform::from_translation(translation))
                .translation()
        },
    };

    for (trajectory, plot) in query.iter() {
        if !plot.enabled || trajectory.is_empty() {
            continue;
        }

        let start = match config.plot_history {
            false => plot.start.max(config.current_time),
            true => plot.start,
        };

        let threshold = match proj {
            Projection::Perspective(p) => {
                const ARC_MINUTE: f32 = 0.000290888;
                plot.threshold * ARC_MINUTE * p.fov
            }
            _ => unreachable!(),
        };

        let (eval, min, max): (&dyn Fn(_) -> _, _, _) =
            if let Some((ref_trajectory, _)) = plot.reference.and_then(|r| query.get(r).ok()) {
                let Some(ref_current_pos) = ref_trajectory.evaluate_position(config.current_time)
                else {
                    continue;
                };
                (
                    &move |at| {
                        let sv = trajectory.evaluate_state_vector(at).unwrap();
                        let ref_sv = ref_trajectory.evaluate_state_vector(at).unwrap();
                        global(sv - ref_sv + StateVector::from_position(ref_current_pos))
                    },
                    start.max(trajectory.start()).max(ref_trajectory.start()),
                    plot.end.min(trajectory.end()).min(ref_trajectory.end()),
                )
            } else {
                (
                    &|at| global(trajectory.evaluate_state_vector(at).unwrap()),
                    start.max(trajectory.start()),
                    plot.end.min(trajectory.end()),
                )
            };

        let points = plot_points(eval, min, max, camera_transform, threshold);
        // for &p in &points {
        //     let dir = camera_transform.translation() - p;
        //     let size = dir.length() * 0.01;
        //     gizmos
        //         .circle(p, Dir3::new(dir).unwrap(), size, plot.color)
        //         .resolution(16);
        // }

        // println!("Points: {}", points.len());
        gizmos.linestrip(points, plot.color);
    }
}

// Adapted from Principia's PlotMethod3.
// (https://github.com/mockingbirdnest/Principia/blob/2024080411-Klein/ksp_plugin/planetarium_body.hpp)
fn plot_points(
    eval: impl Fn(Epoch) -> StateVector<Vec3>,
    min: Epoch,
    max: Epoch,
    camera_transform: &GlobalTransform,
    tan2_angular_resolution: f32,
) -> Vec<Vec3> {
    let tan2_angular_resolution = tan2_angular_resolution * tan2_angular_resolution;
    let final_time = max;
    let mut previous_time = min;

    let mut previous = eval(previous_time);
    let mut delta = (final_time - previous_time).to_seconds() as f32;

    let mut points = Vec::new();
    points.push(previous.position);

    let mut t: Epoch;
    let mut estimated_tan2_error = None::<f32>;
    let mut current: StateVector<Vec3>;

    while points.len() < 10_000 && previous_time < final_time {
        loop {
            if let Some(estimated_tan2_error) = estimated_tan2_error {
                delta *= 0.9
                    * (tan2_angular_resolution / estimated_tan2_error)
                        .sqrt()
                        .sqrt();
            }

            t = previous_time + delta as f64;

            if t > final_time {
                t = final_time;
                delta = (t - previous_time).to_seconds() as f32;
            }

            let extrapolated_position = previous.position + previous.velocity * delta;
            current = eval(t);

            // Prevent catastrophic retries
            if estimated_tan2_error.is_some_and(|error| error == 0.0) {
                break;
            }

            estimated_tan2_error = Some(
                angular_distance(camera_transform, extrapolated_position, current.position) / 16.0,
            );

            if estimated_tan2_error.is_some_and(|error| error <= tan2_angular_resolution) {
                break;
            }
        }

        previous_time = t;
        previous = current;
        points.push(current.position);
    }

    points
}

fn angular_distance(camera_transform: &GlobalTransform, p1: Vec3, p2: Vec3) -> f32 {
    let camera_position = camera_transform.translation();
    let v1 = (p1 - camera_position).normalize();
    let v2 = (p2 - camera_position).normalize();

    let wedge = v1.cross(v2);
    wedge.length_squared() / v1.dot(v2).powi(2)
}
