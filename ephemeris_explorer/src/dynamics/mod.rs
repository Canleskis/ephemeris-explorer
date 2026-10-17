mod celestial;
mod spacecraft;

pub use celestial::*;
pub use spacecraft::*;

use bevy::math::DVec3;
use bevy::prelude::*;
use deepsize::DeepSizeOf;
use ephemeris::{BoundedTrajectory, EvaluateTrajectory};
use ftime::Epoch;

pub type StateVector = ephemeris::StateVector<DVec3>;

#[derive(Component, Clone)]
pub struct Trajectory(std::sync::Arc<std::sync::RwLock<QuinticHermiteSpline>>);

impl std::fmt::Debug for Trajectory {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Trajectory")
            .field("start", &self.start())
            .field("end", &self.end())
            .finish()
    }
}

impl From<QuinticHermiteSpline> for Trajectory {
    #[inline]
    fn from(trajectory: QuinticHermiteSpline) -> Self {
        Self::new(trajectory)
    }
}

impl Trajectory {
    #[inline]
    pub fn new(trajectory: QuinticHermiteSpline) -> Self {
        Self(std::sync::Arc::new(std::sync::RwLock::new(trajectory)))
    }

    #[inline]
    pub fn read(&self) -> std::sync::RwLockReadGuard<'_, QuinticHermiteSpline> {
        self.0.read().unwrap()
    }

    #[inline]
    pub fn write(&mut self) -> std::sync::RwLockWriteGuard<'_, QuinticHermiteSpline> {
        self.0.write().unwrap()
    }

    #[inline]
    pub fn heap_size(&self) -> usize {
        self.read().deep_size_of()
    }

    #[inline]
    pub fn distance_squared_at<T>(&self, other: &T, at: Epoch) -> Option<f64>
    where
        T: EvaluateTrajectory<Vector = DVec3>,
    {
        Some(self.position(at)?.distance_squared(other.position(at)?))
    }

    #[inline]
    pub fn distance_at<T>(&self, other: &T, at: Epoch) -> Option<f64>
    where
        T: EvaluateTrajectory<Vector = DVec3>,
    {
        Some(self.position(at)?.distance(other.position(at)?))
    }

    #[inline]
    pub fn quintic_segment_at(&self, at: Epoch) -> Option<QuinticHermiteSegment> {
        self.read().segment_at(at)
    }
}

impl BoundedTrajectory for Trajectory {
    #[inline]
    fn start(&self) -> Epoch {
        self.read().start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.read().end()
    }

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        self.read().contains(time)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        self.read().segment_count()
    }
}

impl EvaluateTrajectory for Trajectory {
    type Vector = DVec3;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        self.read().position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector> {
        self.read().state_vector(at)
    }
}

impl PartialEq for Trajectory {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        std::sync::Arc::ptr_eq(&self.0, &other.0)
    }
}
