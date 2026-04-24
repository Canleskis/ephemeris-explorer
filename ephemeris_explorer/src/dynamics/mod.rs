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

pub type CubicHermiteSpline = ephemeris::CubicHermiteSpline<DVec3>;

pub enum PredictionTrajectory {
    UniformSpline(UniformSpline),
    CubicHermiteSpline(CubicHermiteSpline),
}

impl DeepSizeOf for PredictionTrajectory {
    #[inline]
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        match self {
            Self::UniformSpline(traj) => traj.deep_size_of_children(context),
            Self::CubicHermiteSpline(traj) => traj.deep_size_of_children(context),
        }
    }
}

impl BoundedTrajectory for PredictionTrajectory {
    #[inline]
    fn start(&self) -> Epoch {
        match self {
            Self::UniformSpline(traj) => traj.start(),
            Self::CubicHermiteSpline(traj) => traj.start(),
        }
    }

    #[inline]
    fn end(&self) -> Epoch {
        match self {
            Self::UniformSpline(traj) => traj.end(),
            Self::CubicHermiteSpline(traj) => traj.end(),
        }
    }

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        match self {
            Self::UniformSpline(traj) => traj.contains(time),
            Self::CubicHermiteSpline(traj) => traj.contains(time),
        }
    }

    #[inline]
    fn segment_count(&self) -> usize {
        match self {
            Self::UniformSpline(traj) => traj.segment_count(),
            Self::CubicHermiteSpline(traj) => traj.segment_count(),
        }
    }
}

impl EvaluateTrajectory for PredictionTrajectory {
    type Vector = DVec3;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        match self {
            Self::UniformSpline(traj) => traj.position(at),
            Self::CubicHermiteSpline(traj) => traj.position(at),
        }
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector> {
        match self {
            Self::UniformSpline(traj) => traj.state_vector(at),
            Self::CubicHermiteSpline(traj) => traj.state_vector(at),
        }
    }
}

#[derive(Component, Clone)]
pub struct Trajectory(std::sync::Arc<std::sync::RwLock<PredictionTrajectory>>);

impl std::fmt::Debug for Trajectory {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Trajectory")
            .field("start", &self.start())
            .field("end", &self.end())
            .finish()
    }
}

impl From<UniformSpline> for Trajectory {
    #[inline]
    fn from(trajectory: UniformSpline) -> Self {
        Self::new(PredictionTrajectory::UniformSpline(trajectory))
    }
}

impl From<CubicHermiteSpline> for Trajectory {
    #[inline]
    fn from(trajectory: CubicHermiteSpline) -> Self {
        Self::new(PredictionTrajectory::CubicHermiteSpline(trajectory))
    }
}

impl Trajectory {
    #[inline]
    pub fn new(trajectory: PredictionTrajectory) -> Self {
        Self(std::sync::Arc::new(std::sync::RwLock::new(trajectory)))
    }

    #[inline]
    pub fn read(&self) -> std::sync::RwLockReadGuard<'_, PredictionTrajectory> {
        self.0.read().unwrap()
    }

    #[inline]
    pub fn write(&mut self) -> std::sync::RwLockWriteGuard<'_, PredictionTrajectory> {
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
