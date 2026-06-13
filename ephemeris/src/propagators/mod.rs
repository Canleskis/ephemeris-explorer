mod nbody;
mod spacecraft;

pub use nbody::*;
pub use spacecraft::*;

use ftime::{Duration, Epoch};
use integration::Solout;

pub trait PropagationDirection {
    fn signed_delta(&self) -> Duration;

    fn offset(to: Epoch, duration: Duration) -> Epoch;

    fn distance(from: Epoch, to: Epoch) -> Duration;

    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        Duration::ZERO.cmp(&Self::distance(*lhs, *rhs))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Forward {
    delta: Duration,
}

impl From<Duration> for Forward {
    #[inline]
    fn from(value: Duration) -> Self {
        Self::new(value)
    }
}

impl Forward {
    #[inline]
    pub fn new(delta: Duration) -> Self {
        Self { delta: delta.abs() }
    }
}

impl PropagationDirection for Forward {
    #[inline]
    fn signed_delta(&self) -> Duration {
        self.delta
    }

    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        to + duration
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        to - from
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Backward {
    delta: Duration,
}

impl From<Duration> for Backward {
    #[inline]
    fn from(value: Duration) -> Self {
        Self::new(value)
    }
}

impl Backward {
    #[inline]
    pub fn new(delta: Duration) -> Self {
        Self { delta: delta.abs() }
    }
}

impl PropagationDirection for Backward {
    #[inline]
    fn signed_delta(&self) -> Duration {
        -self.delta
    }

    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        to - duration
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        from - to
    }
}

pub trait DirectionalSolout<P>: Solout<P> {
    type Direction: PropagationDirection;

    fn solution_time(solution: &Self::Solution) -> Epoch;

    fn has_reached(solution: &Self::Solution, time: Epoch) -> bool;
}
