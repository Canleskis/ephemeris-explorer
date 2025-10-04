pub mod propagators;
pub mod trajectory;

pub use propagators::*;
pub use trajectory::*;

use ftime::{Duration, Epoch};

pub trait Iterable: Sized {
    type Item;

    type Iterator<'a>: Iterator<Item = &'a Self::Item>
    where
        Self: 'a;

    type IteratorMut<'a>: Iterator<Item = &'a mut Self::Item>
    where
        Self: 'a;

    fn iter(&self) -> Self::Iterator<'_>;

    fn iter_mut(&mut self) -> Self::IteratorMut<'_>;
}

impl<T, Item> Iterable for T
where
    for<'a> &'a T: IntoIterator<Item = &'a Item>,
    for<'a> &'a mut T: IntoIterator<Item = &'a mut Item>,
{
    type Item = Item;

    type Iterator<'a>
        = <&'a T as IntoIterator>::IntoIter
    where
        Self: 'a;

    type IteratorMut<'a>
        = <&'a mut T as IntoIterator>::IntoIter
    where
        Self: 'a;

    #[inline]
    fn iter(&self) -> Self::Iterator<'_> {
        self.into_iter()
    }

    #[inline]
    fn iter_mut(&mut self) -> Self::IteratorMut<'_> {
        self.into_iter()
    }
}

pub trait Propagator {
    type Trajectory;

    type Trajectories: Iterable<Item = Self::Trajectory>;
}

/// Trait for propagators that have a defined direction. Useful for tracking the progress of the
/// propagation.
pub trait DirectionalPropagator: Propagator {
    /// Returns the ordering of two epochs according to the propagator's direction.
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering;

    /// Offsets an epoch in time relative to the propagator's direction.
    fn offset(time: Epoch, duration: Duration) -> Epoch;

    /// Returns the time boundary of the trajectory from which propagation will continue.
    fn boundary(trajectory: &Self::Trajectory) -> Epoch;

    /// Returns true if the trajectory has reached the specified epoch.
    #[inline]
    fn reached(trajectory: &Self::Trajectory, time: Epoch) -> bool {
        Self::cmp(&Self::boundary(trajectory), &time).is_ge()
    }
}

/// A trait for propagators that support creating and merging independent trajectory branches.
pub trait BranchingPropagator: Propagator {
    /// Returns a new collection of trajectory that form an independent branch starting at the
    /// propagator's current boundary.
    fn branch(&self) -> Self::Trajectories;

    /// Merges the `rhs` trajectory into the `lhs` one. If the two trajectories overlap in time,
    /// the rhs trajectory overwrites the overlapping portion in the lhs trajectory.
    fn merge(lhs: &mut Self::Trajectory, rhs: Self::Trajectory);
}

pub trait IncrementalPropagator: Propagator {
    type Error;

    /// Advances all trajectories by a single step using the propagator.
    fn step(&mut self, trajs: &mut Self::Trajectories) -> Result<(), Self::Error>;

    /// Advances all trajectories until the specified time is reached.
    #[inline]
    fn step_until(&mut self, trajs: &mut Self::Trajectories, to: Epoch) -> Result<(), Self::Error>
    where
        Self: DirectionalPropagator,
    {
        if trajs.iter().all(|traj| Self::reached(traj, to)) {
            return Ok(());
        }

        loop {
            self.step(trajs)?;

            if trajs.iter().all(|traj| Self::reached(traj, to)) {
                return Ok(());
            }
        }
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
        self.step_until(trajs, to)
    }
}

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

    /// Consumes self and returns the inner propagator and trajectories.
    #[inline]
    pub fn into_inner(self) -> (P, P::Trajectories) {
        (self.propagator, self.trajectories)
    }

    #[inline]
    pub fn boundaries(&self) -> impl Iterator<Item = Epoch> + '_
    where
        P: DirectionalPropagator,
    {
        self.trajectories.iter().map(P::boundary)
    }

    #[inline]
    pub fn reached(&self, time: Epoch) -> bool
    where
        P: DirectionalPropagator,
    {
        self.trajectories.iter().all(|traj| P::reached(traj, time))
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
