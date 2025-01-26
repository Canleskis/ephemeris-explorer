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
    pub sync_frequency: usize,
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
    pub fn all(duration: Duration, sync_frequency: usize) -> Self {
        Self::with([], duration, sync_frequency)
    }

    pub fn with<I>(entities: I, duration: Duration, sync_frequency: usize) -> Self
    where
        I: IntoIterator<Item = Entity>,
    {
        Self {
            entities: entities.into_iter().collect(),
            duration,
            sync_frequency,
            _marker: std::marker::PhantomData,
        }
    }
}

pub type ExtendPredictionEvent<B> = ComputePredictionEvent<B, true>;

pub trait BuilderContext: Sized {
    /// Creates a new context from a reference to the world.
    fn from_world(world: &World) -> Self;

    /// Returns whether the current context is valid.
    fn is_valid(&self, world: &World) -> bool;
}

impl<C: Default + 'static> BuilderContext for C {
    fn from_world(_: &World) -> Self {
        Default::default()
    }

    fn is_valid(&self, _: &World) -> bool {
        true
    }
}

pub trait TrajectoryBuilder: Sized {
    /// The trajectory data type this builder operates on.
    type Trajectory: TrajectoryInner + 'static;

    /// The context in which the builder operates.
    type Context;

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
    fn step<'a, I>(&mut self, trajs: I, ctx: &Self::Context) -> Result<(), StepError>
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
            .map(|trajectory| builder.continued(trajectory.downcast_ref()))
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
    pub fn step(&mut self, ctx: &B::Context) -> Result<(), StepError> {
        self.builder.step(&mut self.trajectories, ctx)
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
    B::Context: BuilderContext + Send + Sync,
{
    fn build(&self, app: &mut App) {
        app.add_event::<ExtendPredictionEvent<B>>()
            .add_observer(dispatch_predictions::<B, true>)
            .add_observer(dispatch_predictions::<B, false>)
            .add_systems(PreUpdate, process_prediction_data::<B>);

        let ctx = B::Context::from_world(app.world());
        app.insert_resource(PredictionCtx::<B>(std::sync::Arc::new(ctx)));
    }
}

#[derive(Component, Debug)]
pub struct PredictionTracker<B: TrajectoryBuilder> {
    thread: bevy::tasks::Task<()>,
    recver: crossbeam_channel::Receiver<PredictionInstance<B>>,
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

#[derive(Resource, Deref, DerefMut)]
pub struct PredictionCtx<B: TrajectoryBuilder>(pub std::sync::Arc<B::Context>);

fn dispatch_predictions<B, const EXTEND: bool>(
    trigger: Trigger<ComputePredictionEvent<B, EXTEND>>,
    world: &World,
    mut commands: Commands,
    query_builder: Query<Entity, With<B>>,
    shared_ctx: Res<PredictionCtx<B>>,
) where
    B: TrajectoryBuilder + Component + Clone,
    B::Context: BuilderContext + Send + Sync,
{
    let ComputePredictionEvent {
        entities,
        duration,
        sync_frequency,
        ..
    } = trigger.event();

    let name = std::any::type_name::<B>();

    let duration = *duration;
    let sync_frequency = 1.max(*sync_frequency);

    if duration.is_negative() || duration == Duration::ZERO {
        bevy::log::error!("invalid duration: {}", duration);
        return;
    }

    let entities = match entities.len() {
        0 => &query_builder.iter().collect(),
        _ => entities,
    };

    let ctx = {
        if !shared_ctx.is_valid(world) {
            let new_ctx = std::sync::Arc::new(B::Context::from_world(world));
            commands.insert_resource(PredictionCtx::<B>(std::sync::Arc::clone(&new_ctx)));
            new_ctx
        } else {
            std::sync::Arc::clone(&shared_ctx)
        }
    };

    for &entity in entities.iter() {
        let mut instance = PredictionInstance::<B>::from_world(entity, world);
        let current = instance.time();
        let mut end = B::add(current, duration);

        let (sender, recver) = crossbeam_channel::unbounded();

        let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread = bevy::tasks::AsyncComputeTaskPool::get().spawn({
            let paused = std::sync::Arc::downgrade(&paused);
            let ctx = std::sync::Arc::clone(&ctx);

            async move {
                bevy::log::debug!("Computing {} prediction for {}", name, duration);
                let t0 = std::time::Instant::now();

                let mut remaining = sync_frequency;
                let instance = &mut instance;
                loop {
                    if let Some(paused) = paused.upgrade() {
                        while paused.load(std::sync::atomic::Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(100));
                        }
                    }

                    if let Err(step_error) = instance.step(&ctx) {
                        bevy::log::debug!("{name}: {step_error}");
                        end = instance.time();
                    }
                    remaining -= 1;

                    if instance.boundaries().all(|t| B::cmp(&t, &end).is_ge()) {
                        remaining = 0;
                    }

                    if remaining == 0 {
                        let current = instance.time();

                        let completed = std::mem::replace(instance, instance.continued());
                        if sender.send(completed).is_err() || B::cmp(&current, &end).is_ge() {
                            bevy::log::info!("Stopping {} prediction thread", name);
                            break;
                        }

                        remaining = sync_frequency;
                    }
                }

                bevy::log::info!("Computing {} prediction took: {:?}", name, t0.elapsed());
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

        let mut extend = tracker.extend;
        let mut current = tracker.current;
        for recv in tracker.recver.try_iter() {
            current = recv.time();

            for (entity, recv_trajectory) in builder.entities().iter().zip(recv.trajectories) {
                if let Ok(mut trajectory) = query_trajectory.get_mut(*entity) {
                    if extend {
                        B::join(trajectory.downcast_mut(), recv_trajectory);
                    } else {
                        *trajectory.downcast_mut() = recv_trajectory;
                        extend = true;
                    }
                }
            }
            *builder = recv.builder;
        }
        tracker.extend = extend;
        tracker.current = current;

        if is_finished {
            commands.entity(entity).remove::<PredictionTracker<B>>();
        }
    }
}
