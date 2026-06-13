pub mod propagators;
pub mod trajectory;

pub use propagators::*;
pub use trajectory::*;

use ftime::{Duration, Epoch};

pub trait Propagator {
    type Solution;

    fn take_solution(&mut self) -> Self::Solution;
}

/// Trait for propagators that have a defined direction. Useful for tracking the progress of the
/// propagation.
pub trait DirectionalPropagator: Propagator {
    /// Offsets an epoch in time relative to the propagator's direction.
    fn offset(time: Epoch, duration: Duration) -> Epoch;

    /// Returns the temporal distance from one epoch to another along the propagator's direction.
    fn distance(from: Epoch, to: Epoch) -> Duration;

    /// Returns the ordering of two epochs according to the propagator's direction.
    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        Duration::ZERO.cmp(&Self::distance(*lhs, *rhs))
    }

    /// Returns the time of the propagator.
    fn time(&self) -> Epoch;

    /// Returns true if the propagator has reached the specified epoch.
    #[inline]
    fn has_reached(&self, time: Epoch) -> bool {
        Self::cmp(&self.time(), &time).is_ge()
    }
}

pub trait IncrementalPropagator: Propagator {
    type Error;

    /// Advances the propagator once.
    fn step(&mut self) -> Result<(), Self::Error>;

    /// Advances the propagator until the given time has been reached.
    #[inline]
    fn step_to(&mut self, time: Epoch) -> Result<(), Self::Error>
    where
        Self: DirectionalPropagator,
    {
        loop {
            if self.has_reached(time) {
                return Ok(());
            }
            self.step()?;
        }
    }
}

pub trait BoundedPropagator: Propagator {
    type Error;

    fn propagate(&mut self, to: Epoch) -> Result<Self::Solution, Self::Error>;
}

impl<P> BoundedPropagator for P
where
    P: IncrementalPropagator + DirectionalPropagator,
{
    type Error = P::Error;

    #[inline]
    fn propagate(&mut self, to: Epoch) -> Result<Self::Solution, Self::Error> {
        self.step_to(to)?;

        Ok(self.take_solution())
    }
}
