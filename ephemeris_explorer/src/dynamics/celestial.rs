use crate::{
    dynamics::{PredictionTrajectory, Trajectory},
    prediction::{PredictionControllerOf, PredictionPropagator, PredictionTarget},
};

use std::ops::{Add, Div, Mul, Sub};

use bevy::ecs::query::QueryData;
use bevy::math::DVec3;
use bevy::prelude::*;
use double_double as dd;
use ephemeris::{
    AccelerationPairedAssign, BoundedTrajectory, EvaluateTrajectory, ExtendSpline, GravitySource,
    InterpolationSample, InterpolationTolerance, KinematicState, PropagationDirection,
};
pub use ephemeris::{Backward, Forward};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::{
    gravity::newtonian::AccelerationPaired,
    prelude::{Mass, Position},
};

pub type UniformSpline = ephemeris::UniformSpline<DVec3>;
pub type QuinticHermiteSpline = ephemeris::QuinticHermiteSpline<DVec3>;
pub type QuinticHermiteSegment = ephemeris::QuinticHermiteSegment<DVec3>;

// We use the less accurate operations here, since the difference is acceptable (at most a couple
// kilometers for some major moons after 100 years), whilst the performance is nearly doubled.
// See the `integrators_comparison` test.
#[derive(Clone, Copy, Debug, Default)]
pub struct DoubleDVec3 {
    pub h: DVec3,
    pub l: DVec3,
}

impl DoubleDVec3 {
    #[inline]
    pub fn new(h: DVec3) -> Self {
        Self { h, l: DVec3::ZERO }
    }

    #[inline]
    pub fn as_dvec3(&self) -> DVec3 {
        self.h
    }
}

impl From<DoubleDVec3> for (DVec3, DVec3) {
    #[inline]
    fn from(value: DoubleDVec3) -> Self {
        (value.h, value.l)
    }
}

impl From<(DVec3, DVec3)> for DoubleDVec3 {
    #[inline]
    fn from((h, l): (DVec3, DVec3)) -> Self {
        Self { h, l }
    }
}

impl Add for DoubleDVec3 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        dd::sloppy::dw_add_dw(self.into(), rhs.into()).into()
    }
}

impl Sub for DoubleDVec3 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        dd::sloppy::dw_sub_dw(self.into(), rhs.into()).into()
    }
}

impl Mul<f64> for DoubleDVec3 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        dd::naive::dw_mul(self.into(), rhs).into()
    }
}

impl Div<f64> for DoubleDVec3 {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        dd::naive::dw_div(self.into(), rhs).into()
    }
}

// Double precision for point-mass acceleration is enough. Implemented to work with double-double.
#[derive(Position, Mass)]
pub struct PointMass {
    position: DVec3,
    mu: f64,
}

impl AccelerationPairedAssign for PointMass {
    type Output = DoubleDVec3;

    #[inline]
    fn acceleration_paired_assign(
        &self,
        other: &Self,
        a1: &mut Self::Output,
        a2: &mut Self::Output,
    ) {
        let (ai, aj) = self.acceleration_paired(other, &0.0);
        a1.h += ai;
        a2.h += aj;
    }
}

impl GravitySource for PointMass {
    type Vector = DoubleDVec3;

    type Scalar = f64;

    #[inline]
    fn init(position: Self::Vector, mu: Self::Scalar) -> Self {
        Self {
            position: position.h,
            mu,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct QuinticHermiteSample {
    pub time: Epoch,
    pub position: DVec3,
    pub velocity: DVec3,
    pub acceleration: DVec3,
}

impl QuinticHermiteSample {
    #[inline]
    pub fn new(time: Epoch, position: DVec3, velocity: DVec3, acceleration: DVec3) -> Self {
        Self {
            time,
            position,
            velocity,
            acceleration,
        }
    }

    #[inline]
    pub fn kinematic_state(&self) -> KinematicState<DVec3> {
        KinematicState {
            position: self.position,
            velocity: self.velocity,
            acceleration: self.acceleration,
        }
    }
}

impl InterpolationSample<DoubleDVec3> for QuinticHermiteSample {
    type Trajectory = QuinticHermiteSpline;

    #[inline]
    fn new(
        time: Epoch,
        position: DoubleDVec3,
        velocity: DoubleDVec3,
        acceleration: DoubleDVec3,
    ) -> Self {
        Self {
            time,
            position: position.h,
            velocity: velocity.h,
            acceleration: acceleration.h,
        }
    }
}

impl ExtendSpline<Forward, QuinticHermiteSample> for QuinticHermiteSpline {
    #[inline]
    fn push_at_bound(&mut self, sample: QuinticHermiteSample) {
        self.push_back(
            sample.time,
            sample.position,
            sample.velocity,
            sample.acceleration,
        );
    }
}

impl ExtendSpline<Backward, QuinticHermiteSample> for QuinticHermiteSpline {
    #[inline]
    fn push_at_bound(&mut self, sample: QuinticHermiteSample) {
        self.push_front(
            sample.time,
            sample.position,
            sample.velocity,
            sample.acceleration,
        );
    }
}

#[derive(Clone, Copy)]
pub struct MidDistance(f64);

impl MidDistance {
    #[inline]
    pub fn new(tolerance: f64) -> Self {
        Self(tolerance * tolerance)
    }
}

impl InterpolationTolerance<QuinticHermiteSample> for MidDistance {
    #[inline]
    fn is_valid(&mut self, samples: &[QuinticHermiteSample]) -> bool {
        let len = samples.len();
        if len < 3 {
            return true;
        }

        let first = &samples[0];
        let last = &samples[len - 1];
        let segment = QuinticHermiteSegment::new_unordered(
            first.time,
            first.kinematic_state(),
            last.time,
            last.kinematic_state(),
        );
        let mid_sample = &samples[len / 2];

        segment
            .position(mid_sample.time)
            .is_some_and(|position| position.distance_squared(mid_sample.position) <= self.0)
    }
}

pub type Vector = DoubleDVec3;
pub type ForceModel = ephemeris::NewtonianGravity<PointMass>;
pub type NBodyProblem = ephemeris::NBodyProblem<Vector, ForceModel>;
pub type SplineInterpolators<D> =
    ephemeris::SplineInterpolators<D, QuinticHermiteSample, Vector, MidDistance>;
pub type NBodyPropagator<D> =
    ephemeris::NBodyPropagator<Vector, ForceModel, QuinlanTremaine12<f64>, SplineInterpolators<D>>;

#[derive(QueryData)]
#[query_data(mutable)]
pub struct CelestialTrajectory<D> {
    pub trajectory: &'static mut Trajectory,
    marker: std::marker::PhantomData<D>,
}

impl<D> CelestialTrajectory<D>
where
    CelestialTrajectory<D>: PredictionTarget<Propagator = NBodyPropagator<D>>,
    SplineInterpolators<D>: Solout<NBodyProblem>,
    D: PropagationDirection,
{
    #[inline]
    pub fn new_propagator(
        delta: Duration,
        tolerance: f64,
        mut problem: NBodyProblem,
    ) -> PredictionPropagator<Self> {
        let solout = SplineInterpolators::new(&mut problem, MidDistance::new(tolerance));
        PredictionPropagator(NBodyPropagator::new_with_solout::<D>(
            delta, problem, solout,
        ))
    }

    #[inline]
    pub fn new_controller_of(entities: Vec<Entity>) -> PredictionControllerOf<Self> {
        PredictionControllerOf::new(entities)
    }
}

impl PredictionTarget for CelestialTrajectory<Forward> {
    type Propagator = NBodyPropagator<Forward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_, '_>, propagated: QuinticHermiteSpline) {
        let PredictionTrajectory::QuinticHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        world_traj.clear_after(propagated.start());
        world_traj.append(propagated);
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_, '_>, propagated: QuinticHermiteSpline) {
        // Should we keep the previous allocation?
        let PredictionTrajectory::QuinticHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        *world_traj = propagated;
    }

    #[inline]
    fn insert(commands: &mut EntityCommands, propagated: QuinticHermiteSpline) {
        commands.insert_if_new(Trajectory::from(propagated));
    }
}

impl PredictionTarget for CelestialTrajectory<Backward> {
    type Propagator = NBodyPropagator<Backward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_, '_>, propagated: QuinticHermiteSpline) {
        let PredictionTrajectory::QuinticHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        world_traj.clear_before(propagated.end());
        world_traj.prepend(propagated);
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_, '_>, propagated: QuinticHermiteSpline) {
        let PredictionTrajectory::QuinticHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        *world_traj = propagated;
    }

    #[inline]
    fn insert(commands: &mut EntityCommands, propagated: QuinticHermiteSpline) {
        commands.insert_if_new(Trajectory::from(propagated));
    }
}
