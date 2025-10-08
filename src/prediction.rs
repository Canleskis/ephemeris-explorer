//! Asynchronous trajectory prediction plugin.
//!
//! This module provides the [`PredictionPlugin`], designed to compute trajectories on a background
//! thread and stream incremental results back to the Bevy ECS. This allows for progressive
//! visualisation without blocking the main thread.
//!
//! The prediction process requires attaching a [`PredictionContext<P>`] component to a "controller"
//! entity. This context groups a set of target entities (each with a [`Trajectory`] component) and
//! a specific [`Propagator`] `P` that defines the prediction logic.
//!
//! To start a prediction, a [`ComputePredictionEvent<P>`] is sent. This triggers the creation of an
//! asynchronous task for each matching [`PredictionContext<P>`].
//!
//! Inside the task, a [`Propagation`] is created and used to extend the trajectories. Periodically,
//! it sends a snapshot of its current state back to the main thread (the periodicity is as
//! frequently as possible, but is also determined by the `min_steps` parameter).
//!
//! On the main thread, the [`process_prediction_data`] system receives these snapshots and merges
//! them into the corresponding [`Trajectory`] components in the Bevy ECS.
//!
//! While a prediction is running, a [`PredictionTracker<P>`] component is added to the controller
//! entity, allowing for progress monitoring and control (e.g., pausing). Target entities are also
//! marked with a [`Predicting<P>`] component.

use bevy::math::DVec3;
use bevy::prelude::*;
use deepsize::DeepSizeOf;
use ephemeris::{
    BoundedTrajectory, BranchingPropagator, DirectionalPropagator, EvaluateTrajectory,
    IncrementalPropagator, Iterable, Propagation, Propagator, StateVector,
};
use ftime::{Duration, Epoch};

pub trait PredictionTrajectory:
    BoundedTrajectory + EvaluateTrajectory<Vector = DVec3> + DeepSizeOf + Send + Sync
{
    fn as_any(&self) -> &dyn std::any::Any;

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl<T> PredictionTrajectory for T
where
    T: BoundedTrajectory + EvaluateTrajectory<Vector = DVec3> + DeepSizeOf + Send + Sync + 'static,
{
    #[inline]
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    #[inline]
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[derive(Component, Clone)]
pub struct Trajectory(std::sync::Arc<std::sync::RwLock<dyn PredictionTrajectory>>);

impl Trajectory {
    #[inline]
    pub fn new<T: PredictionTrajectory + 'static>(trajectory: T) -> Self {
        Self(std::sync::Arc::new(std::sync::RwLock::new(trajectory)))
    }

    #[inline]
    pub fn read(&self) -> std::sync::RwLockReadGuard<'_, dyn PredictionTrajectory> {
        self.0.read().unwrap()
    }

    #[inline]
    pub fn write(&mut self, f: impl FnOnce(&mut dyn PredictionTrajectory)) {
        f(&mut *self.0.write().unwrap())
    }

    #[inline]
    pub fn heap_size(&self) -> usize {
        self.read().deep_size_of()
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
    fn len(&self) -> usize {
        self.read().len()
    }
}

impl EvaluateTrajectory for Trajectory {
    type Vector = DVec3;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        self.read().position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        self.read().state_vector(at)
    }
}

/// Trait for propagators that can be used for predictions.
pub trait PredictionPropagator:
    Propagator<Trajectories: IntoIterator<Item = Self::Trajectory> + Send>
    + IncrementalPropagator<Error: std::error::Error>
    + DirectionalPropagator
    + BranchingPropagator
    + Clone
    + Send
    + Sync
    + 'static
{
}
impl<P> PredictionPropagator for P where
    P: Propagator<Trajectories: IntoIterator<Item = P::Trajectory> + Send>
        + IncrementalPropagator<Error: std::error::Error>
        + DirectionalPropagator
        + BranchingPropagator
        + Clone
        + Send
        + Sync
        + 'static
{
}

/// Context linking a propagator to the ECS trajectories it propagates (via entities).
#[derive(Clone, Component)]
pub struct PredictionContext<P> {
    pub entities: Vec<Entity>,
    pub propagator: P,
}

impl<P> PredictionContext<P> {
    #[inline]
    pub fn new(entities: Vec<Entity>, propagator: P) -> Self {
        Self {
            entities,
            propagator,
        }
    }

    #[inline]
    pub fn to_propagation(&self) -> Propagation<P>
    where
        P: BranchingPropagator + Clone,
    {
        Propagation::new(self.propagator.clone())
    }
}

/// Computes the prediction for the given duration for all target entities associated with matching
/// [`PredictionContext<P>`] components.
///
/// If `EXTEND` is `false`, the previous prediction will be discarded. Otherwise, the propagator
/// will only discard the part of the previous prediction that overlaps with the new prediction.
#[derive(Event)]
pub struct ComputePredictionEvent<P, const EXTEND: bool = false> {
    pub entities: bevy::ecs::entity::EntityHashSet,
    pub duration: Duration,
    /// Minimum propagation steps between streamed snapshots.
    pub min_steps: usize,
    pub _marker: std::marker::PhantomData<P>,
}

impl<P, const EXTEND: bool> Clone for ComputePredictionEvent<P, EXTEND> {
    #[inline]
    fn clone(&self) -> Self {
        Self {
            entities: self.entities.clone(),
            ..*self
        }
    }
}

impl<P, const EXTEND: bool> ComputePredictionEvent<P, EXTEND> {
    #[inline]
    pub fn all(duration: Duration, min_steps: usize) -> Self {
        Self::with([], duration, min_steps)
    }

    #[inline]
    pub fn with<I>(entities: I, duration: Duration, min_steps: usize) -> Self
    where
        I: IntoIterator<Item = Entity>,
    {
        Self {
            entities: entities.into_iter().collect(),
            duration,
            min_steps,
            _marker: std::marker::PhantomData,
        }
    }
}

pub type ExtendPredictionEvent<P> = ComputePredictionEvent<P, true>;

/// Marker component to indicate that an entity's trajectory prediction is being computed.
#[derive(Clone, Copy, Component)]
pub struct Predicting<P>(std::marker::PhantomData<P>);

pub struct PredictionPlugin<P>(std::marker::PhantomData<fn(P)>);

impl<P> Default for PredictionPlugin<P> {
    #[inline]
    fn default() -> Self {
        Self(std::marker::PhantomData)
    }
}

impl<P> Plugin for PredictionPlugin<P>
where
    P: PredictionPropagator,
{
    fn build(&self, app: &mut App) {
        app.add_event::<ExtendPredictionEvent<P>>()
            .add_observer(dispatch_predictions::<P, true>)
            .add_observer(dispatch_predictions::<P, false>)
            .add_observer(add_predicting_marker::<P>)
            .add_observer(remove_predicting_marker::<P>)
            .add_systems(PreUpdate, process_prediction_data::<P>);
    }
}

/// Tracks an asynchronous prediction for a specific propagator of type `P`.
#[derive(Component, Debug)]
pub struct PredictionTracker<P: Propagator> {
    thread: bevy::tasks::Task<()>,
    recver: async_channel::Receiver<Propagation<P>>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    extend: bool,
    current: Epoch,
    start: Epoch,
    target: Epoch,
    _marker: std::marker::PhantomData<fn(P)>,
}

impl<P: Propagator> PredictionTracker<P> {
    #[inline]
    pub fn progress(&self) -> f32 {
        ((self.current - self.start).as_seconds() / (self.target - self.start).as_seconds())
            .abs()
            .min(1.0) as f32
    }

    #[inline]
    pub fn pause_or_resume(&self) {
        self.paused.store(
            !self.paused.load(std::sync::atomic::Ordering::Relaxed),
            std::sync::atomic::Ordering::Relaxed,
        );
    }

    #[inline]
    pub fn is_paused(&self) -> bool {
        self.paused.load(std::sync::atomic::Ordering::Relaxed)
    }

    #[inline]
    pub fn is_in_progress(&self) -> bool {
        !self.is_paused() && !self.thread.is_finished()
    }
}

/// Starts async tasks for each targeted [`PredictionContext<P>`] that step the propagator and batch
/// these steps before sending snapshots (based on `min_steps`).
fn dispatch_predictions<P, const EXTEND: bool>(
    trigger: Trigger<ComputePredictionEvent<P, EXTEND>>,
    mut commands: Commands,
    query_prediction: Query<(Entity, &PredictionContext<P>)>,
) where
    P: PredictionPropagator,
{
    let ComputePredictionEvent {
        entities,
        duration,
        min_steps,
        ..
    } = trigger.event();

    let duration = *duration;
    let min_steps = 1.max(*min_steps);

    if duration.is_negative() || duration == Duration::ZERO {
        bevy::log::error!("invalid duration: {}", duration);
        return;
    }

    let predictions: Vec<(Entity, &PredictionContext<P>)> = match entities.len() {
        0 => query_prediction.iter().collect(),
        _ => query_prediction.iter_many(entities).collect(),
    };

    for (entity, prediction) in predictions.iter() {
        let mut propagation = prediction.to_propagation();

        let current = propagation.time();
        let mut end = P::offset(current, duration);

        let (sender, recver) = async_channel::bounded(1);

        let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread = bevy::tasks::AsyncComputeTaskPool::get().spawn({
            let paused = std::sync::Arc::downgrade(&paused);

            async move {
                let name = pretty_type_name::pretty_type_name::<P>();
                bevy::log::debug!("Computing {name} prediction for {duration}");
                let t0 = std::time::Instant::now();

                let mut remaining = min_steps;
                let propagation = &mut propagation;
                loop {
                    if let Some(paused) = paused.upgrade() {
                        while paused.load(std::sync::atomic::Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(50));
                        }
                    }

                    if let Err(err) = propagation.step() {
                        bevy::log::warn!("{name}: {err}");
                        end = propagation.time();
                    }
                    remaining = remaining.saturating_sub(1);

                    let reached_end = propagation.reached(end);
                    if (remaining == 0 && sender.is_empty()) || reached_end || sender.is_closed() {
                        let completed = std::mem::replace(propagation, propagation.branch());
                        if sender.send(completed).await.is_err() || reached_end {
                            break;
                        }
                        remaining = min_steps;
                    }
                }

                bevy::log::debug!("Stopping {} prediction thread", name);
                bevy::log::debug!("Computing {} prediction took: {:?}", name, t0.elapsed());
            }
        });

        commands.entity(*entity).insert(PredictionTracker::<P> {
            thread,
            recver,
            paused,
            current,
            extend: EXTEND,
            start: current,
            target: end,
            _marker: std::marker::PhantomData,
        });
    }
}

/// Inserts streamed propagator snapshots back into the ECS.
pub fn process_prediction_data<P>(
    mut commands: Commands,
    mut query_prediction: Query<(Entity, &mut PredictionContext<P>, &mut PredictionTracker<P>)>,
    mut query_trajectory: Query<&mut Trajectory>,
) where
    P: PredictionPropagator,
{
    for (entity, mut prediction, mut tracker) in &mut query_prediction {
        let is_finished = tracker.thread.is_finished();

        if let Ok(recv) = tracker.recver.try_recv() {
            tracker.current = recv.time();
            let (propagator, trajectories) = recv.into_inner();

            prediction.propagator = propagator;
            for (entity, recv_trajectory) in prediction.entities.iter().zip(trajectories) {
                if let Ok(mut trajectory) = query_trajectory.get_mut(*entity) {
                    trajectory.write(|trajectory| {
                        if tracker.extend {
                            P::merge(
                                trajectory.as_any_mut().downcast_mut().unwrap(),
                                recv_trajectory,
                            )
                        } else {
                            *trajectory.as_any_mut().downcast_mut().unwrap() = recv_trajectory;
                            tracker.extend = true;
                        }
                    });
                }
            }
        }

        if is_finished {
            commands.entity(entity).remove::<PredictionTracker<P>>();
        }
    }
}

fn add_predicting_marker<P>(
    trigger: Trigger<OnInsert, PredictionTracker<P>>,
    mut commands: Commands,
    query_prediction: Query<&PredictionContext<P>>,
) where
    P: PredictionPropagator,
{
    if let Ok(prediction) = query_prediction.get(trigger.target()) {
        for entity in &prediction.entities {
            commands
                .entity(*entity)
                .insert(Predicting::<P>(std::marker::PhantomData));
        }
    };
}

fn remove_predicting_marker<P>(
    trigger: Trigger<OnRemove, PredictionTracker<P>>,
    mut commands: Commands,
    query_prediction: Query<&PredictionContext<P>>,
) where
    P: PredictionPropagator,
{
    if let Ok(prediction) = query_prediction.get(trigger.target()) {
        for entity in &prediction.entities {
            if let Ok(mut entity) = commands.get_entity(*entity) {
                entity.remove::<Predicting<P>>();
            }
        }
    };
}
