use crate::{
    dynamics::{PredictionTrajectory, Trajectory},
    prediction::{PredictionControllerOf, PredictionPropagator, PredictionTarget},
};

use std::ops::{Add, Div, Mul, Neg, Sub};

use bevy::ecs::query::QueryData;
use bevy::math::DVec3;
use bevy::prelude::*;
use ephemeris::{
    AccelerationPairedAssign, BoundedTrajectory, ExtendSpline, GravitySource, InterpolationSample,
    InterpolationTolerance, PropagationDirection, QuinticHermite,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;

pub use ephemeris::{Backward, Forward};
use particular::{
    gravity::newtonian::AccelerationPaired,
    prelude::{Mass, Position},
};
pub type UniformSpline = ephemeris::UniformSpline<DVec3>;
pub type QuinticHermiteSpline = ephemeris::QuinticHermiteSpline<DVec3>;

#[derive(Clone, Copy, Debug, Default)]
pub struct Double<T> {
    value: T,
    error: T,
}

impl<T> From<Double<T>> for (T, T) {
    #[inline]
    fn from(d: Double<T>) -> Self {
        (d.value, d.error)
    }
}

impl<T> Double<T> {
    #[inline]
    pub fn new(value: T) -> Self
    where
        T: Default,
    {
        Self {
            value,
            error: T::default(),
        }
    }

    #[inline]
    pub fn value(&self) -> T
    where
        T: Copy,
    {
        self.value
    }

    #[inline]
    fn two_sum(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Copy,
    {
        let value = a + b;
        let v = value - a;
        let error = (a - (value - v)) + (b - v);

        Self { value, error }
    }

    #[inline]
    fn two_sub(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Neg<Output = T> + Copy,
    {
        Self::two_sum(a, -b)
    }

    #[inline]
    fn fast_two_sum(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Copy,
    {
        let value = a + b;
        let error = b - (value - a);

        Self { value, error }
    }
}

#[inline]
pub fn fast_two_sum<T>(a: T, b: T) -> Double<T>
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    // Joldes et al. (2017) Algorithm 1
    let s = a + b;
    let z = s - a;
    Double {
        value: s,
        error: b - z,
    }
}

impl<T> Add for Double<T>
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        let sum = Double::two_sum(self.value, rhs.value);
        Double::fast_two_sum(sum.value, (sum.error + self.error) + rhs.error)
    }
}

impl<T> Sub for Double<T>
where
    T: Add<Output = T> + Sub<Output = T> + Neg<Output = T> + Copy,
{
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        let sum = Double::two_sub(self.value, rhs.value);
        Double::fast_two_sum(sum.value, (sum.error + self.error) - rhs.error)
    }
}

impl Mul<f64> for Double<DVec3> {
    type Output = Double<DVec3>;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Double {
            value: self.value * rhs,
            error: self.error * rhs,
        }
    }
}

impl Div<f64> for Double<DVec3> {
    type Output = Double<DVec3>;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Double {
            value: self.value / rhs,
            error: self.error / rhs,
        }
    }
}

// Double precision for point-mass is sufficient. Implemented to work with double-double.
#[derive(Position, Mass)]
pub struct PointMass {
    position: DVec3,
    mu: f64,
}

impl AccelerationPairedAssign for PointMass {
    type Output = Double<DVec3>;

    #[inline]
    fn acceleration_paired_assign(
        &self,
        other: &Self,
        a1: &mut Self::Output,
        a2: &mut Self::Output,
    ) {
        let (ai, aj) = self.acceleration_paired(other, &0.0);
        a1.value += ai;
        a2.value += aj;
    }
}

impl GravitySource for PointMass {
    type Vector = Double<DVec3>;

    type Scalar = f64;

    #[inline]
    fn new(position: Self::Vector, mu: Self::Scalar) -> Self {
        Self {
            position: position.value,
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
}

impl InterpolationSample<Double<DVec3>> for QuinticHermiteSample {
    type Trajectory = QuinticHermiteSpline;

    #[inline]
    fn new(
        time: Epoch,
        position: Double<DVec3>,
        velocity: Double<DVec3>,
        acceleration: Double<DVec3>,
    ) -> Self {
        Self {
            time,
            position: position.value,
            velocity: velocity.value,
            acceleration: acceleration.value,
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

        let first = samples[0];
        let last = samples[len - 1];
        let hermite = QuinticHermite::new(
            (
                first.time.as_offset_seconds(),
                last.time.as_offset_seconds(),
            ),
            (first.position, last.position),
            (first.velocity, last.velocity),
            (first.acceleration, last.acceleration),
        );

        let mid = len / 2;
        let mid_sample = &samples[mid];

        hermite
            .eval(mid_sample.time.as_offset_seconds())
            .distance_squared(mid_sample.position)
            <= self.0
    }
}

pub type Vector = Double<DVec3>;
pub type ForceModel = ephemeris::CachedNewtonianGravity<PointMass>;
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
    D: From<Duration> + PropagationDirection,
{
    #[inline]
    pub fn new_propagator(
        delta: Duration,
        tolerance: f64,
        mut problem: NBodyProblem,
    ) -> PredictionPropagator<Self> {
        let solout = SplineInterpolators::new(&mut problem, MidDistance::new(tolerance));
        PredictionPropagator(NBodyPropagator::new_solout(D::from(delta), problem, solout))
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
