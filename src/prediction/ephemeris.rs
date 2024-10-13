use super::{
    integration::{IntegrationState, PEFRL},
    trajectory::{interpolate, BuildTrajectory, Segment, Trajectory, MAX_DIV},
};

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::Duration;
use particular::gravity::newtonian::Acceleration;
use particular::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct SegmentSamples {
    index: usize,
    xs: [f64; MAX_DIV],
    ys: [f64; MAX_DIV],
    zs: [f64; MAX_DIV],
}

impl SegmentSamples {
    #[inline]
    pub fn with(position: DVec3) -> Self {
        Self {
            index: 1,
            xs: [position.x; MAX_DIV],
            ys: [position.y; MAX_DIV],
            zs: [position.z; MAX_DIV],
        }
    }

    #[inline]
    pub fn push(&mut self, position: DVec3) {
        assert!(self.index < MAX_DIV, "Too many points in segment.");
        self.xs[self.index] = position.x;
        self.ys[self.index] = position.y;
        self.zs[self.index] = position.z;
        self.index += 1;
    }

    #[inline]
    pub fn fill(&mut self, position: DVec3) {
        self.xs.fill(position.x);
        self.ys.fill(position.y);
        self.zs.fill(position.z);
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.index
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == MAX_DIV
    }

    #[inline]
    fn interpolated(&self, ts: [f64; MAX_DIV], deg: usize) -> Segment {
        Segment {
            x: interpolate(deg, &ts, &self.xs),
            y: interpolate(deg, &ts, &self.ys),
            z: interpolate(deg, &ts, &self.zs),
        }
    }
}

/// A structure that represents an ephemeris being built.
#[derive(Debug, Component, Position, Mass)]
pub struct EphemerisBuilder<Direction> {
    time: Duration,
    deg: usize,
    velocity: DVec3,
    position: DVec3,
    mu: f64,
    samples: SegmentSamples,
    _marker: std::marker::PhantomData<fn(Direction)>,
}

impl<Direction> Clone for EphemerisBuilder<Direction> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<Direction> Copy for EphemerisBuilder<Direction> {}

impl<Direction> EphemerisBuilder<Direction> {
    pub fn new(deg: usize, position: DVec3, velocity: DVec3, mu: f64) -> Self {
        Self {
            time: Duration::ZERO,
            deg,
            samples: SegmentSamples::with(position),
            position,
            mu,
            velocity,
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn accelerations_into(accelerations: &mut Vec<DVec3>, slice: &[Self], _: f64) {
        accelerations.extend(slice.brute_force_pairs(Acceleration::checked()));
    }

    #[inline]
    pub fn update_trajectory_with(
        &mut self,
        trajectory: &mut Trajectory,
        update: impl FnOnce(&mut Trajectory, SegmentSamples, usize),
    ) {
        let time = self.time.total_nanoseconds();
        let interval = trajectory.granule().total_nanoseconds() / (MAX_DIV - 1) as i128;
        if time % interval == 0 {
            self.samples.push(self.position);
            if self.samples.is_full() {
                update(trajectory, self.samples, self.deg);
                self.time = Duration::ZERO;
                self.samples.fill(self.position);
                self.samples.index = 1;
            }
        }
    }
}

impl<Direction> IntegrationState for EphemerisBuilder<Direction> {
    #[inline]
    fn position(&mut self) -> &mut DVec3 {
        &mut self.position
    }

    #[inline]
    fn velocity(&mut self) -> &mut DVec3 {
        &mut self.velocity
    }

    #[inline]
    fn step(&mut self, delta: Duration) {
        self.time += delta;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Forward;

impl BuildTrajectory<EphemerisBuilder<Forward>> for Trajectory {
    fn continued(&self) -> Self {
        Trajectory::new(self.end(), self.granule())
    }

    #[inline]
    fn build_trajectories(
        trajectories: &mut [Self],
        builders: &mut [EphemerisBuilder<Forward>],
        dt: Duration,
    ) {
        const TS: [f64; MAX_DIV] = {
            let mut out = [0.0; MAX_DIV];
            let mut i = 0;
            while i < MAX_DIV {
                out[i] = i as f64 / (MAX_DIV - 1) as f64;
                i += 1;
            }
            out
        };

        PEFRL::integrate(dt, builders, EphemerisBuilder::accelerations_into);
        builders
            .iter_mut()
            .zip(trajectories.iter_mut())
            .for_each(|(builder, trajectory)| {
                builder.update_trajectory_with(trajectory, |trajectory, points, deg| {
                    trajectory.append_segment(points.interpolated(TS, deg));
                })
            });
    }

    fn join(&mut self, trajectory: Self) {
        self.append(trajectory);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Backward;

impl BuildTrajectory<EphemerisBuilder<Backward>> for Trajectory {
    fn continued(&self) -> Self {
        Trajectory::new(self.start(), self.granule())
    }

    fn build_trajectories(
        trajectories: &mut [Self],
        builders: &mut [EphemerisBuilder<Backward>],
        dt: Duration,
    ) {
        const RTS: [f64; MAX_DIV] = {
            let mut out = [0.0; MAX_DIV];
            let mut i = 0;
            while i < MAX_DIV {
                out[i] = 1.0 - i as f64 / (MAX_DIV - 1) as f64;
                i += 1;
            }
            out
        };

        PEFRL::integrate(-dt, builders, EphemerisBuilder::accelerations_into);
        builders
            .iter_mut()
            .zip(trajectories.iter_mut())
            .for_each(|(builder, trajectory)| {
                builder.update_trajectory_with(trajectory, |trajectory, points, deg| {
                    trajectory.prepend_segment(points.interpolated(RTS, deg));
                })
            });
    }

    fn join(&mut self, trajectory: Self) {
        self.prepend(trajectory);
    }
}
