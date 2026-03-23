//! Asynchronous trajectory prediction plugin.
//!
//! This module provides the [`PredictionPlugin`], designed to compute trajectories on a background
//! thread and stream incremental results back to the Bevy ECS allowing progressive visualisation
//! without blocking the main thread.
//!
//! The prediction process requires attaching a [`PredictionContext<T>`] component to a "controller"
//! entity. This context groups a [`Propagator`] `P` that defines the prediction logic for a set of
//! target `entities`.
//!
//! To start a prediction, a [`ComputePrediction<T>`] event is sent. This triggers the creation of
//! an asynchronous task for the matching [`PredictionContext<T>`].
//!
//! Inside the task, the given [`Propagation`] is used to extend the trajectories. Periodically,
//! snapshots are sent back to the main thread (the periodicity is as frequently as possible, but is
//! also determined by the `min_steps` parameter).
//!
//! On the main thread, the [`process_prediction_data`] system receives these snapshots and merges
//! them into the corresponding components in the Bevy ECS.
//!
//! While a prediction is running, a [`PredictionTracker<T>`] component is added to the controller
//! entity, allowing for progress monitoring and control (e.g., pausing). Target entities are also
//! marked with a [`Predicting<T>`].

use bevy::ecs::query::QueryData;
use bevy::prelude::*;
use ephemeris::{
    BranchingPropagator, DirectionalPropagator, IncrementalPropagator, Iterable, Propagation,
    Propagator,
};
use ftime::{Duration, Epoch};

pub trait PropagationTarget: QueryData + 'static {
    type Propagator: PredictionPropagator;

    fn merge(
        item: &mut Self::Item<'_, '_>,
        propagated: <Self::Propagator as PredictionPropagator>::Trajectory,
    );

    fn overwrite(
        item: &mut Self::Item<'_, '_>,
        propagated: <Self::Propagator as PredictionPropagator>::Trajectory,
    );
}

/// Context linking a propagator to the ECS trajectories it propagates (via entities).
#[derive(Clone, Component)]
pub struct PredictionContext<T: PropagationTarget> {
    pub entities: Vec<Entity>,
    pub propagator: T::Propagator,
}

impl<T: PropagationTarget> PredictionContext<T> {
    #[inline]
    pub fn new(entities: Vec<Entity>, propagator: T::Propagator) -> Self {
        Self {
            entities,
            propagator,
        }
    }
}

/// Computes the prediction for the given duration.
///
/// If `overwrite` is `false`, the previous prediction will be discarded. Otherwise, the propagator
/// will only discard the part of the previous prediction that overlaps with the new prediction.
#[derive(Clone, Copy, EntityEvent)]
pub struct ComputePrediction<T: PropagationTarget> {
    pub entity: Entity,
    pub propagator: T::Propagator,
    pub duration: Duration,
    pub synchronisation: Synchronisation,
    pub overwrite: bool,
}

impl<T: PropagationTarget> ComputePrediction<T> {
    #[inline]
    pub fn new(
        entity: Entity,
        propagator: T::Propagator,
        duration: Duration,
        synchronisation: Synchronisation,
        overwrite: bool,
    ) -> Self {
        Self {
            entity,
            propagator,
            duration,
            synchronisation,
            overwrite,
        }
    }

    #[inline]
    pub fn extend(
        entity: Entity,
        propagator: T::Propagator,
        duration: Duration,
        synchronisation: Synchronisation,
    ) -> Self {
        Self::new(entity, propagator, duration, synchronisation, false)
    }
}

/// Trait for propagators that can be used for predictions.
pub trait PredictionPropagator:
    Propagator<Trajectories: IntoIterator<Item = Self::Trajectory> + Send + Sync>
    + IncrementalPropagator<Error: std::error::Error>
    + DirectionalPropagator
    + BranchingPropagator
    + Clone
    + Send
    + Sync
{
    type Trajectory;
}
impl<P> PredictionPropagator for P
where
    P: Propagator<Trajectories: IntoIterator + Send + Sync>
        + IncrementalPropagator<Error: std::error::Error>
        + DirectionalPropagator
        + BranchingPropagator
        + Clone
        + Send
        + Sync,
{
    type Trajectory = <Self::Trajectories as IntoIterator>::Item;
}

/// Component to indicate how many predictions are currently being computed for an entity.
#[derive(Clone, Copy, Component)]
pub struct Predicting(pub usize);

/// Marker component to indicate that an entity's trajectory prediction of type `P` is being
/// computed.
#[derive(Clone, Copy, Component)]
pub struct PredictingWith<T>(std::marker::PhantomData<fn(T)>);

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct PredictionSystems;

pub struct PredictionPlugin<T>(std::marker::PhantomData<fn(T)>);

impl<T> Default for PredictionPlugin<T> {
    #[inline]
    fn default() -> Self {
        Self(std::marker::PhantomData)
    }
}

impl<T> Plugin for PredictionPlugin<T>
where
    T: PropagationTarget,
{
    fn build(&self, app: &mut App) {
        app.add_observer(dispatch_predictions::<T>)
            .add_observer(add_predicting_marker::<T>)
            .add_observer(remove_predicting_marker::<T>)
            .add_systems(
                PreUpdate,
                process_prediction_data::<T>.in_set(PredictionSystems),
            );
    }
}

/// Tracks an asynchronous prediction for a specific propagator of type `P`.
#[derive(Component)]
pub struct PredictionTracker<T: PropagationTarget> {
    thread: bevy::tasks::Task<()>,
    recver: async_channel::Receiver<Propagation<T::Propagator>>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    overwrite: bool,
    current: Epoch,
    start: Epoch,
    target: Epoch,
}

impl<T: PropagationTarget> PredictionTracker<T> {
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

#[derive(Clone, Copy, Debug)]
pub enum Synchronisation {
    Steps {
        minimum: usize,
        remaining: usize,
    },
    Frequency {
        time: std::time::Duration,
        last: std::time::Instant,
    },
}

impl Default for Synchronisation {
    #[inline]
    fn default() -> Self {
        Self::steps(1000)
    }
}

impl Synchronisation {
    #[inline]
    pub fn completed() -> Self {
        Self::steps(usize::MAX)
    }

    #[inline]
    pub fn steps(minimum: usize) -> Self {
        let minimum = 1.max(minimum);
        Self::Steps {
            minimum,
            remaining: minimum,
        }
    }

    #[inline]
    pub fn frequency(time: std::time::Duration) -> Self {
        Self::Frequency {
            time,
            last: std::time::Instant::now(),
        }
    }

    #[inline]
    pub fn hertz(hertz: usize) -> Self {
        Self::frequency(std::time::Duration::from_secs_f64(1.0 / hertz as f64))
    }

    #[inline]
    pub fn increment(&mut self) {
        match self {
            Synchronisation::Steps { remaining, .. } => *remaining = remaining.saturating_sub(1),
            Synchronisation::Frequency { .. } => (),
        }
    }

    #[inline]
    pub fn is_ready(&self) -> bool {
        match self {
            Synchronisation::Steps { remaining, .. } => *remaining == 0,
            Synchronisation::Frequency { time, last } => last.elapsed() >= *time,
        }
    }

    #[inline]
    pub fn reset(&mut self) {
        match self {
            Synchronisation::Steps { remaining, minimum } => *remaining = *minimum,
            Synchronisation::Frequency { last, .. } => *last = std::time::Instant::now(),
        }
    }
}

/// Steps the propagator and batches these steps before sending snapshots (based on `min_steps`).
#[inline]
async fn prediction_task<T>(
    mut propagation: Propagation<T::Propagator>,
    mut end: Epoch,
    mut sync: Synchronisation,
    sender: async_channel::Sender<Propagation<T::Propagator>>,
    paused: std::sync::Weak<std::sync::atomic::AtomicBool>,
) where
    T: PropagationTarget,
{
    let name = pretty_type_name::pretty_type_name::<T>();
    info!("Computing {name} prediction until {end}");
    let t0 = std::time::Instant::now();

    let propagation = &mut propagation;
    loop {
        if let Some(paused) = paused.upgrade() {
            while paused.load(std::sync::atomic::Ordering::Relaxed) {
                bevy::tasks::futures_lite::future::yield_now().await;
            }
        }

        if let Err(err) = propagation.step() {
            warn!("{name}: {err}");
            end = propagation.time();
        }
        sync.increment();

        let reached_end = propagation.has_reached(end);
        if (sync.is_ready() && sender.is_empty()) || reached_end || sender.is_closed() {
            let completed = std::mem::replace(propagation, propagation.branch());
            if sender.send(completed).await.is_err() || reached_end {
                break;
            }
            sync.reset();
        }
    }

    info!("Computing {name} prediction took: {:?}", t0.elapsed());
}

/// Starts async task for the targeted [`PredictionContext<P>`].
fn dispatch_predictions<T>(trigger: On<ComputePrediction<T>>, mut commands: Commands)
where
    T: PropagationTarget,
{
    let name = pretty_type_name::pretty_type_name::<T::Propagator>();

    let ComputePrediction {
        propagator,
        duration,
        synchronisation,
        overwrite,
        ..
    } = trigger.event();

    if duration.is_negative() || *duration == Duration::ZERO {
        error!("Cancelled {name} prediction: invalid duration: {duration}");
        return;
    }

    debug!("Starting {name} prediction for {duration}",);

    let propagation = Propagation::new(propagator.clone());
    let current = propagation.time();
    let end = T::Propagator::offset(current, *duration);

    let (sender, recver) = async_channel::bounded(1);
    let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));

    let thread = bevy::tasks::AsyncComputeTaskPool::get().spawn(prediction_task::<T>(
        propagation,
        end,
        *synchronisation,
        sender,
        std::sync::Arc::downgrade(&paused),
    ));

    commands
        .entity(trigger.event_target())
        .insert(PredictionTracker::<T> {
            thread,
            recver,
            paused,
            current,
            overwrite: *overwrite,
            start: current,
            target: end,
        });
}

/// Inserts streamed propagator snapshots back into the ECS.
pub fn process_prediction_data<T>(
    mut commands: Commands,
    mut query_prediction: Query<(Entity, &mut PredictionContext<T>, &mut PredictionTracker<T>)>,
    mut query_target: Query<T>,
) where
    T: PropagationTarget,
{
    for (entity, mut prediction, mut tracker) in &mut query_prediction {
        if tracker.thread.is_finished() {
            commands.entity(entity).remove::<PredictionTracker<T>>();
        }

        if let Ok(recv) = tracker.recver.try_recv() {
            tracker.current = recv.time();
            let (propagator, trajectories) = recv.into_inner();

            prediction.propagator = propagator;
            for (entity, recv_trajectory) in prediction.entities.iter().zip(trajectories) {
                if let Ok(mut target) = query_target.get_mut(*entity) {
                    if tracker.overwrite {
                        T::overwrite(&mut target, recv_trajectory);
                    } else {
                        T::merge(&mut target, recv_trajectory);
                    }
                }
            }
            if tracker.overwrite {
                tracker.overwrite = false;
            }
        }
    }
}

fn add_predicting_marker<T>(
    trigger: On<Add, PredictionTracker<T>>,
    mut commands: Commands,
    query_prediction: Query<&PredictionContext<T>>,
) where
    T: PropagationTarget,
{
    if let Ok(prediction) = query_prediction.get(trigger.event_target()) {
        for entity in &prediction.entities {
            if let Ok(mut entity) = commands.get_entity(*entity) {
                entity.queue_silenced(|mut entity: EntityWorldMut| {
                    entity.insert(PredictingWith::<T>(Default::default()));
                    match entity.get_mut::<Predicting>().as_deref_mut() {
                        Some(Predicting(count)) => *count += 1,
                        None => {
                            entity.insert(Predicting(1));
                        }
                    }
                });
            }
        }
    };
}

fn remove_predicting_marker<T>(
    trigger: On<Remove, PredictionTracker<T>>,
    mut commands: Commands,
    query_prediction: Query<&PredictionContext<T>>,
) where
    T: PropagationTarget,
{
    if let Ok(prediction) = query_prediction.get(trigger.event_target()) {
        for entity in &prediction.entities {
            if let Ok(mut entity) = commands.get_entity(*entity) { 
                entity.queue_silenced(|mut entity: EntityWorldMut| {
                    entity.remove::<PredictingWith<T>>();
                    if let Some(Predicting(count)) = entity.get_mut::<Predicting>().as_deref_mut() {
                        *count -= 1;
                        if *count == 0 {
                            entity.remove::<Predicting>();
                        }
                    }
                });
            }
        }
    };
}
