//! Asynchronous trajectory prediction plugin.
//!
//! This module provides the [`PredictionPlugin`], designed to compute trajectories on a background
//! thread and stream incremental results back to the Bevy ECS allowing progressive visualisation
//! without blocking the main thread.
//!
//! The prediction process requires attaching the [`PredictionPropagator<T>`] and [`PredictionController<T>`]
//! components to a "controller" entity. This groups a [`Propagator`] that defines the prediction
//! logic with a set of target `entities`.
//!
//! To start a prediction, a [`ComputePrediction<T>`] event is sent. This triggers the creation of
//! an asynchronous task for the matching [`PredictionPropagator<T>`].
//!
//! Inside the task, the given [`Propagation`] is used to extend the trajectories. Periodically,
//! snapshots are sent back to the main thread.
//!
//! On the main thread, the [`process_prediction_data`] system receives these snapshots and merges
//! them into the corresponding components in the Bevy ECS.
//!
//! While a prediction is running, a [`PredictionTracker<T>`] component is added to the controller
//! entity, allowing for progress monitoring and control (e.g., pausing). Target entities are also
//! marked with the [`InPredictionWith<T>`] and [`InPrediction`] components.

use bevy::ecs::query::QueryData;
use bevy::prelude::*;
use ephemeris::{
    BranchingPropagator, DirectionalPropagator, IncrementalPropagator, Propagation, Propagator,
};
use ftime::{Duration, Epoch};

pub trait PredictionTarget: QueryData + 'static {
    type Propagator: Propagator<Trajectories: IntoIterator + Send + Sync>
        + IncrementalPropagator<Error: std::error::Error>
        + DirectionalPropagator
        + BranchingPropagator
        + Clone
        + Send
        + Sync;

    fn merge(
        item: &mut Self::Item<'_, '_>,
        propagated: <<Self::Propagator as Propagator>::Trajectories as IntoIterator>::Item,
    );

    fn overwrite(
        item: &mut Self::Item<'_, '_>,
        propagated: <<Self::Propagator as Propagator>::Trajectories as IntoIterator>::Item,
    );
}

#[derive(Clone, Component, Deref, DerefMut)]
pub struct PredictionPropagator<T: PredictionTarget>(pub T::Propagator);

// No relation for now since they don't support self-references which are coming in 0.19. For now,
// manual insertions works since we only use them as static components.
/// Component linking a propagator to the ECS trajectories it propagates.
#[derive(Default, Debug, PartialEq, Eq, Component)]
pub struct PredictionControllerOf<T: PredictionTarget>(
    #[relationship] Vec<Entity>,
    std::marker::PhantomData<fn(T)>,
);

impl<T: PredictionTarget> PredictionControllerOf<T> {
    #[inline]
    pub fn new(entities: Vec<Entity>) -> Self {
        Self(entities, Default::default())
    }

    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = Entity> {
        self.0.iter().copied()
    }
}

#[derive(Clone, Copy, Debug, Component)]
pub struct PredictionController<T: PredictionTarget>(
    #[relationship] pub Entity,
    std::marker::PhantomData<fn(T)>,
);

impl<T: PredictionTarget> PredictionController<T> {
    #[inline]
    pub fn new(controller: Entity) -> Self {
        Self(controller, Default::default())
    }

    #[inline]
    pub fn controller(&self) -> Entity {
        self.0
    }
}

/// Computes the prediction for the given duration.
///
/// If `overwrite` is `false`, the previous prediction will be discarded. Otherwise, the propagator
/// will only discard the part of the previous prediction that overlaps with the new prediction.
#[derive(Clone, Copy, EntityEvent)]
pub struct ComputePrediction<T: PredictionTarget> {
    pub entity: Entity,
    pub propagator: Option<T::Propagator>,
    pub duration: Duration,
    pub synchronisation: Synchronisation,
    pub overwrite: bool,
}

impl<T: PredictionTarget> ComputePrediction<T> {
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
            propagator: Some(propagator),
            duration,
            synchronisation,
            overwrite,
        }
    }

    #[inline]
    pub fn extend(entity: Entity, duration: Duration, synchronisation: Synchronisation) -> Self {
        Self {
            entity,
            propagator: None,
            duration,
            synchronisation,
            overwrite: false,
        }
    }

    #[inline]
    pub fn with_propagator(mut self, propagator: T::Propagator) -> Self {
        self.propagator = Some(propagator);
        self
    }
}

/// Component to indicate how many predictions are currently being computed for an entity.
#[derive(Debug, Clone, Copy, Component, Deref, DerefMut)]
pub struct InPrediction(pub usize);

/// Marker component to indicate that an entity's trajectory prediction of type `T` is being
/// computed.
#[derive(Clone, Copy, Component)]
pub struct InPredictionWith<T>(std::marker::PhantomData<fn(T)>);

impl<T> Default for InPredictionWith<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}

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
    T: PredictionTarget,
{
    fn build(&self, app: &mut App) {
        app.add_observer(dispatch_predictions::<T>)
            .add_observer(add_prediction_markers::<T>)
            .add_observer(remove_prediction_markers::<T>)
            .add_systems(
                PreUpdate,
                process_prediction_data::<T>.in_set(PredictionSystems),
            );
    }
}

/// Component tracking how many predictions are being computed by a controller entity.
#[derive(Debug, Clone, Copy, Component, Deref, DerefMut)]
pub struct Predicting(pub usize);

/// Tracks an asynchronous prediction for a specific propagation target of type `T`.
#[derive(Component)]
pub struct PredictionTracker<T: PredictionTarget> {
    thread: bevy::tasks::Task<()>,
    recver: async_channel::Receiver<Propagation<T::Propagator>>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    overwrite: bool,
    current: Epoch,
    start: Epoch,
    target: Epoch,
}

impl<T: PredictionTarget> PredictionTracker<T> {
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

/// Starts async task for the targeted [`PredictionPropagator`].
fn dispatch_predictions<T>(
    trigger: On<ComputePrediction<T>>,
    query: Query<(&PredictionPropagator<T>, Option<&PredictionTracker<T>>)>,
    mut commands: Commands,
) where
    T: PredictionTarget,
{
    let name = pretty_type_name::pretty_type_name::<T::Propagator>();

    let ComputePrediction {
        propagator: input_propagator,
        duration,
        synchronisation,
        overwrite,
        ..
    } = trigger.event();

    let Ok((world_propagator, tracker)) = query.get(trigger.event_target()) else {
        error!("Cancelled {name} prediction: failed to get propagator");
        return;
    };

    if tracker.is_some() {
        info!("Stopped {name} prediction: a new one was started");
        commands
            .entity(trigger.event_target())
            .remove::<PredictionTracker<T>>();
    }

    if duration.is_negative() || *duration == Duration::ZERO {
        warn!("Cancelled {name} prediction: invalid duration: {duration}");
        return;
    }

    let propagator = input_propagator.as_ref().unwrap_or(world_propagator);
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

/// Steps the propagator and batches these steps before sending snapshots (based on `min_steps`).
#[inline]
async fn prediction_task<T>(
    mut propagation: Propagation<T::Propagator>,
    mut end: Epoch,
    mut sync: Synchronisation,
    sender: async_channel::Sender<Propagation<T::Propagator>>,
    paused: std::sync::Weak<std::sync::atomic::AtomicBool>,
) where
    T: PredictionTarget,
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

/// Inserts streamed propagator snapshots back into the ECS.
#[expect(clippy::type_complexity)]
pub fn process_prediction_data<T>(
    mut commands: Commands,
    mut query_prediction: Query<(
        Entity,
        &PredictionControllerOf<T>,
        &mut PredictionPropagator<T>,
        &mut PredictionTracker<T>,
    )>,
    mut query_target: Query<T>,
) where
    T: PredictionTarget,
{
    for (entity, controller_of, mut propagator, mut tracker) in &mut query_prediction {
        if tracker.thread.is_finished() {
            commands.entity(entity).remove::<PredictionTracker<T>>();
        }

        if let Ok(recv) = tracker.recver.try_recv() {
            tracker.current = recv.time();
            let (recv_propagator, recv_trajectories) = recv.into_inner();

            **propagator = recv_propagator;
            for (entity, recv_trajectory) in controller_of.iter().zip(recv_trajectories) {
                if let Ok(mut target) = query_target.get_mut(entity) {
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

fn add_prediction_markers<T>(
    trigger: On<Add, PredictionTracker<T>>,
    mut commands: Commands,
    mut query_prediction: Query<(&PredictionControllerOf<T>, Option<&mut Predicting>)>,
    mut query_in_prediction: Query<&mut InPrediction>,
) where
    T: PredictionTarget,
{
    if let Ok((controller_of, predicting)) = query_prediction.get_mut(trigger.event_target()) {
        match predicting {
            Some(mut predicting) => **predicting += 1,
            None => {
                commands
                    .entity(trigger.event_target())
                    .insert(Predicting(1));
            }
        }

        for e in controller_of.iter() {
            commands.entity(e).insert(InPredictionWith::<T>::default());
            match query_in_prediction.get_mut(e) {
                Ok(mut in_prediction) => **in_prediction += 1,
                Err(_) => {
                    commands.entity(e).insert(InPrediction(1));
                }
            }
        }
    };
}

fn remove_prediction_markers<T>(
    trigger: On<Remove, PredictionTracker<T>>,
    mut commands: Commands,
    mut query_prediction: Query<(&PredictionControllerOf<T>, &mut Predicting)>,
    mut query_in_prediction: Query<&mut InPrediction>,
) where
    T: PredictionTarget,
{
    if let Ok((controller_of, mut predicting)) = query_prediction.get_mut(trigger.event_target()) {
        **predicting = predicting.saturating_sub(1);
        if **predicting == 0 {
            commands
                .entity(trigger.event_target())
                .remove::<Predicting>();
        }

        for entity in controller_of.iter() {
            match query_in_prediction.get_mut(entity) {
                Ok(mut in_prediction) => {
                    **in_prediction = in_prediction.saturating_sub(1);
                    if **in_prediction == 0 {
                        commands.entity(entity).remove::<InPrediction>();
                    }
                }
                Err(_) => unreachable!(),
            }
        }
    };
}
