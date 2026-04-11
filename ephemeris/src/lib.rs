pub mod propagators;
pub mod trajectory;

pub use propagators::*;
pub use trajectory::*;

use ftime::{Duration, Epoch};

pub trait Propagator {
    type Trajectories;
}

/// Trait for propagators that have a defined direction. Useful for tracking the progress of the
/// propagation.
pub trait DirectionalPropagator: Propagator {
    /// Offsets an epoch in time relative to the propagator's direction.
    fn offset(time: Epoch, duration: Duration) -> Epoch;

    /// Returns the temporal distance from one epoch to another along the propagator's direction.
    fn distance(from: Epoch, to: Epoch) -> Duration;

    /// Returns the time boundary of the trajectories.
    fn boundaries(trajectory: &Self::Trajectories) -> impl Iterator<Item = Epoch> + '_;

    /// Returns the ordering of two epochs according to the propagator's direction.
    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        Duration::ZERO.cmp(&Self::distance(*lhs, *rhs))
    }

    /// Returns true if the trajectories have reached the specified epoch.
    #[inline]
    fn has_reached(trajectory: &Self::Trajectories, time: Epoch) -> bool {
        Self::boundaries(trajectory).all(|boundary| Self::cmp(&boundary, &time).is_ge())
    }
}

/// A trait for propagators that support creating independent trajectory branches.
pub trait BranchingPropagator: Propagator {
    /// Returns a new collection of trajectory that form an independent branch starting at the
    /// propagator's current boundary.
    fn branch(&self) -> Self::Trajectories;
}

pub trait IncrementalPropagator: Propagator {
    type Error;

    /// Advances all trajectories by a single step using the propagator.
    fn step(&mut self, trajs: &mut Self::Trajectories) -> Result<(), Self::Error>;

    /// Advances all trajectories until the specified time bound is reached.
    #[inline]
    fn step_to(&mut self, trajs: &mut Self::Trajectories, bound: Epoch) -> Result<(), Self::Error>
    where
        Self: DirectionalPropagator,
    {
        while !Self::has_reached(trajs, bound) {
            self.step(trajs)?;
        }
        Ok(())
    }
}

pub trait BoundedPropagator: Propagator {
    type Error;

    /// Advances all trajectories to the specified time.
    fn propagate(&mut self, trajs: &mut Self::Trajectories, to: Epoch) -> Result<(), Self::Error>;
}

impl<P> BoundedPropagator for P
where
    P: IncrementalPropagator + DirectionalPropagator,
{
    type Error = P::Error;

    #[inline]
    fn propagate(&mut self, trajs: &mut Self::Trajectories, to: Epoch) -> Result<(), Self::Error> {
        self.step_to(trajs, to)
    }
}

#[derive(Debug)]
pub struct Propagation<P: Propagator> {
    propagator: P,
    trajectories: P::Trajectories,
}

impl<P: Propagator> Propagation<P> {
    #[inline]
    pub fn new(propagator: P) -> Self
    where
        P: BranchingPropagator,
    {
        Self {
            trajectories: propagator.branch(),
            propagator,
        }
    }

    #[inline]
    pub fn with_trajectories(propagator: P, trajectories: P::Trajectories) -> Self {
        Self {
            propagator,
            trajectories,
        }
    }

    /// Returns the inner propagator and trajectories.
    #[inline]
    pub fn into_inner(self) -> (P, P::Trajectories) {
        (self.propagator, self.trajectories)
    }

    #[inline]
    pub fn propagator(&self) -> &P {
        &self.propagator
    }

    #[inline]
    pub fn trajectories(&self) -> &P::Trajectories {
        &self.trajectories
    }

    #[inline]
    pub fn boundaries(&self) -> impl Iterator<Item = Epoch> + '_
    where
        P: DirectionalPropagator,
    {
        P::boundaries(&self.trajectories)
    }

    #[inline]
    pub fn has_reached(&self, time: Epoch) -> bool
    where
        P: DirectionalPropagator,
    {
        P::has_reached(&self.trajectories, time)
    }

    /// Returns the current time of the propagation.
    #[inline]
    pub fn time(&self) -> Epoch
    where
        P: DirectionalPropagator,
    {
        self.boundaries()
            .min_by(P::cmp)
            .unwrap_or_else(|| P::offset(Epoch::default(), -Duration::MAX))
    }

    #[inline]
    pub fn branch(&self) -> Self
    where
        P: BranchingPropagator + Clone,
    {
        Self {
            propagator: self.propagator.clone(),
            trajectories: self.propagator.branch(),
        }
    }

    #[inline]
    pub fn step(&mut self) -> Result<(), P::Error>
    where
        P: IncrementalPropagator,
    {
        self.propagator.step(&mut self.trajectories)
    }

    #[inline]
    pub fn propagate(&mut self, to: Epoch) -> Result<(), P::Error>
    where
        P: BoundedPropagator,
    {
        self.propagator.propagate(&mut self.trajectories, to)
    }
}
