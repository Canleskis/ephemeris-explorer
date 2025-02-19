pub mod ephemeris;
pub mod integration;
pub mod trajectory;

pub use ephemeris::*;
pub use integration::*;
pub use trajectory::*;

use bevy::prelude::*;
use hifitime::{Duration, Epoch};

/// Computes the prediction for the given duration using the given synchronization count.
///
/// If `EXTEND` is `false`, the previous prediction will be discarded. Otherwise, the builder will
/// only discard the part of the previous prediction that overlaps with the new prediction.
#[derive(Event)]
pub struct ComputePredictionEvent<B, const EXTEND: bool = false> {
    pub entities: bevy::utils::HashSet<Entity>,
    pub duration: Duration,
    /// Minimum number of steps needed between synchronisations.
    pub min_steps: usize,
    pub _marker: std::marker::PhantomData<fn(B)>,
}

impl<B, const EXTEND: bool> Clone for ComputePredictionEvent<B, EXTEND> {
    fn clone(&self) -> Self {
        Self {
            entities: self.entities.clone(),
            ..*self
        }
    }
}

impl<B, const EXTEND: bool> ComputePredictionEvent<B, EXTEND> {
    pub fn all(duration: Duration, min_steps: usize) -> Self {
        Self::with([], duration, min_steps)
    }

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

pub type ExtendPredictionEvent<B> = ComputePredictionEvent<B, true>;

#[derive(Clone, Copy, Component)]
pub struct Predicting<B>(std::marker::PhantomData<fn(B)>);

pub trait TrajectoryBuilder: Sized {
    /// The trajectory data type this builder operates on.
    type Trajectory: TrajectoryInner + 'static;

    /// Returns the ordering of the two epochs as defined by the builder.
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering;

    /// Returns the epoch that is the sum of the given epoch and duration as defined by the builder.
    fn add(to: Epoch, duration: Duration) -> Epoch;

    /// Returns the boundary of the given trajectory, which is usually the epoch at which the
    /// trajectory starts or ends, as defined by the builder.
    fn boundary(trajectory: &Self::Trajectory) -> Epoch;

    /// Returns a new empty trajectory that continues the given trajectory in the current context of
    /// the builder.
    fn continued(&self, trajectory: &Self::Trajectory) -> Self::Trajectory;

    /// Joins the two trajectories.
    fn join(lhs: &mut Self::Trajectory, rhs: Self::Trajectory);

    /// Returns the entities this builder builds trajectories for.
    fn entities(&self) -> &[Entity];

    /// Steps the builder and updates the given trajectories using the given context.
    fn step<'a, I>(&mut self, trajs: I) -> Result<(), StepError>
    where
        I: IntoIterator<Item = &'a mut Self::Trajectory>;
}

pub struct PredictionInstance<B: TrajectoryBuilder> {
    pub builder: B,
    pub trajectories: Vec<B::Trajectory>,
}

impl<B: TrajectoryBuilder + Component + Clone> PredictionInstance<B> {
    pub fn builder_from_world(entity: Entity, world: &World) -> B {
        world
            .entity(entity)
            .get::<B>()
            .unwrap_or_else(|| panic!("Missing builder component for entity {:?}", entity))
            .clone()
    }

    pub fn continued_trajectories_from_world(builder: &B, world: &World) -> Vec<B::Trajectory> {
        builder
            .entities()
            .iter()
            .map(|entity| {
                world
                    .entity(*entity)
                    .get::<Trajectory>()
                    .unwrap_or_else(|| {
                        panic!("Missing trajectory component for entity {:?}", entity)
                    })
            })
            .map(|trajectory| builder.continued(trajectory.read().as_any().downcast_ref().unwrap()))
            .collect()
    }

    pub fn from_world(builder: Entity, world: &World) -> Self {
        let builder = Self::builder_from_world(builder, world);
        Self {
            trajectories: Self::continued_trajectories_from_world(&builder, world),
            builder,
        }
    }
}

impl<B: TrajectoryBuilder> PredictionInstance<B> {
    #[inline]
    pub fn boundaries(&self) -> impl Iterator<Item = Epoch> + '_ {
        self.trajectories.iter().map(B::boundary)
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        self.boundaries().min_by(B::cmp).unwrap()
    }

    #[inline]
    pub fn step(&mut self) -> Result<(), StepError> {
        self.builder.step(&mut self.trajectories)
    }

    #[inline]
    pub fn continued(&self) -> Self
    where
        B: Clone,
    {
        Self {
            builder: self.builder.clone(),
            trajectories: self
                .trajectories
                .iter()
                .map(|trajectory| self.builder.continued(trajectory))
                .collect(),
        }
    }
}

pub struct PredictionPlugin<B>(std::marker::PhantomData<fn(B)>);

impl<B> Default for PredictionPlugin<B> {
    fn default() -> Self {
        Self(std::marker::PhantomData)
    }
}

impl<B> Plugin for PredictionPlugin<B>
where
    B: TrajectoryBuilder + Component + Clone,
{
    fn build(&self, app: &mut App) {
        app.add_event::<ExtendPredictionEvent<B>>()
            .add_observer(dispatch_predictions::<B, true>)
            .add_observer(dispatch_predictions::<B, false>)
            .add_observer(add_predicting_marker::<B>)
            .add_observer(remove_predicting_marker::<B>)
            .add_systems(PreUpdate, process_prediction_data::<B>);
    }
}

#[derive(Component, Debug)]
pub struct PredictionTracker<B: TrajectoryBuilder> {
    thread: bevy::tasks::Task<()>,
    recver: async_channel::Receiver<PredictionInstance<B>>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    extend: bool,
    current: Epoch,
    start: Epoch,
    target: Epoch,
    _marker: std::marker::PhantomData<fn(B)>,
}

impl<B: TrajectoryBuilder + Component> PredictionTracker<B> {
    pub fn progress(&self) -> f32 {
        ((self.current - self.start).to_seconds() / (self.target - self.start).to_seconds())
            .abs()
            .min(1.0) as f32
    }

    pub fn pause_or_resume(&self) {
        self.paused.store(
            !self.paused.load(std::sync::atomic::Ordering::Relaxed),
            std::sync::atomic::Ordering::Relaxed,
        );
    }

    pub fn is_paused(&self) -> bool {
        self.paused.load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn in_progress(&self) -> bool {
        !self.is_paused() && !self.thread.is_finished()
    }
}

fn dispatch_predictions<B, const EXTEND: bool>(
    trigger: Trigger<ComputePredictionEvent<B, EXTEND>>,
    world: &World,
    mut commands: Commands,
    query_builder: Query<Entity, With<B>>,
) where
    B: TrajectoryBuilder + Component + Clone,
{
    let ComputePredictionEvent {
        entities,
        duration,
        min_steps,
        ..
    } = trigger.event();

    let name = std::any::type_name::<B>();

    let duration = *duration;
    let min_steps = 1.max(*min_steps);

    if duration.is_negative() || duration == Duration::ZERO {
        bevy::log::error!("invalid duration: {}", duration);
        return;
    }

    let entities = match entities.len() {
        0 => &query_builder.iter().collect(),
        _ => entities,
    };

    for &entity in entities.iter() {
        let mut instance = PredictionInstance::<B>::from_world(entity, world);

        let current = instance.time();
        let mut end = B::add(current, duration);

        let (sender, recver) = async_channel::bounded(1);

        let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread = bevy::tasks::AsyncComputeTaskPool::get().spawn({
            let paused = std::sync::Arc::downgrade(&paused);

            async move {
                bevy::log::debug!("Computing {} prediction for {}", name, duration);
                let t0 = std::time::Instant::now();

                let mut remaining = min_steps;
                let instance = &mut instance;
                loop {
                    if let Some(paused) = paused.upgrade() {
                        while paused.load(std::sync::atomic::Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(50));
                        }
                    }

                    if let Err(step_error) = instance.step() {
                        bevy::log::debug!("{name}: {step_error}");
                        end = instance.time();
                    }
                    remaining = remaining.saturating_sub(1);

                    let reached_end = instance.boundaries().all(|t| B::cmp(&t, &end).is_ge());
                    if (remaining > 0 || sender.is_full()) && !reached_end && !sender.is_closed() {
                        continue;
                    }

                    let completed = std::mem::replace(instance, instance.continued());
                    if sender.send(completed).await.is_err() || reached_end {
                        break;
                    }
                    remaining = min_steps;
                }

                bevy::log::debug!("Stopping {} prediction thread", name);
                bevy::log::debug!("Computing {} prediction took: {:?}", name, t0.elapsed());
            }
        });

        commands.entity(entity).insert(PredictionTracker::<B> {
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

pub fn process_prediction_data<B>(
    mut commands: Commands,
    mut query_builder: Query<(Entity, &mut B, &mut PredictionTracker<B>)>,
    mut query_trajectory: Query<&mut Trajectory>,
) where
    B: TrajectoryBuilder + Component,
{
    for (entity, mut builder, mut tracker) in &mut query_builder {
        let is_finished = tracker.thread.is_finished();

        if let Ok(recv) = tracker.recver.try_recv() {
            tracker.current = recv.time();
            *builder = recv.builder;

            for (entity, recv_trajectory) in builder.entities().iter().zip(recv.trajectories) {
                if let Ok(mut trajectory) = query_trajectory.get_mut(*entity) {
                    trajectory.write(|trajectory| {
                        if tracker.extend {
                            B::join(
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
            commands.entity(entity).remove::<PredictionTracker<B>>();
        }
    }
}

fn add_predicting_marker<B>(
    trigger: Trigger<OnInsert, PredictionTracker<B>>,
    mut commands: Commands,
    query_builder: Query<&B>,
) where
    B: TrajectoryBuilder + Component,
{
    if let Ok(builder) = query_builder.get(trigger.entity()) {
        for entity in builder.entities() {
            commands
                .entity(*entity)
                .insert(Predicting::<B>(std::marker::PhantomData));
        }
    };
}

fn remove_predicting_marker<B>(
    trigger: Trigger<OnRemove, PredictionTracker<B>>,
    mut commands: Commands,
    query_builder: Query<&B>,
) where
    B: TrajectoryBuilder + Component,
{
    if let Ok(builder) = query_builder.get(trigger.entity()) {
        for entity in builder.entities() {
            commands.entity(*entity).remove::<Predicting<B>>();
        }
    };
}
