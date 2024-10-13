pub mod ephemeris;
pub mod integration;
pub mod trajectory;

pub use ephemeris::*;
pub use trajectory::*;

use bevy::prelude::*;
use hifitime::Duration;

#[derive(Event)]
pub struct ComputePredictionEvent<Builder> {
    pub root: Entity,
    pub duration: Duration,
    pub sync_count: usize,
    pub _marker: std::marker::PhantomData<fn(Builder)>,
}

impl<Builder> Clone for ComputePredictionEvent<Builder> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<Builder> Copy for ComputePredictionEvent<Builder> {}

impl<Builder> ComputePredictionEvent<Builder> {
    pub fn new(root: Entity, duration: Duration, sync_count: usize) -> Self {
        Self {
            root,
            duration,
            sync_count,
            _marker: std::marker::PhantomData,
        }
    }
}

#[derive(Resource)]
pub struct PredictionSettings<Builder> {
    pub dt: Duration,
    pub _marker: std::marker::PhantomData<fn(Builder)>,
}

impl<Builder> PredictionSettings<Builder> {
    pub fn new(dt: Duration) -> Self {
        Self {
            dt,
            _marker: std::marker::PhantomData,
        }
    }
}

pub struct PredictionPlugin<Builder>(std::marker::PhantomData<fn(Builder)>);

impl<Builder> Default for PredictionPlugin<Builder> {
    fn default() -> Self {
        Self(std::marker::PhantomData)
    }
}

impl<Builder> Plugin for PredictionPlugin<Builder>
where
    Builder: Component + Clone,
    Trajectory: BuildTrajectory<Builder>,
{
    fn build(&self, app: &mut App) {
        app.add_event::<ComputePredictionEvent<Builder>>()
            .add_systems(
                Update,
                (
                    compute_predictions::<Builder>,
                    process_predictions::<Builder>,
                )
                    .chain(),
            );
    }
}

pub fn adjusted_duration(duration: Duration, granules: impl Iterator<Item = Duration>) -> Duration {
    granules
        .map(|granule| granule * (duration.to_seconds() / granule.to_seconds()).ceil())
        .max()
        .unwrap()
}

type PredictionData<Builder> = Vec<(Entity, Builder, Trajectory)>;

#[derive(Component, Debug)]
pub struct PredictionTracker<Builder: Component> {
    thread: std::thread::JoinHandle<()>,
    receiver: crossbeam_channel::Receiver<PredictionData<Builder>>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    steps: usize,
    current: usize,
}

impl<Direction: Component> PredictionTracker<Direction> {
    pub fn progress(&self) -> f32 {
        (self.current as f32 / self.steps as f32).min(1.0)
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

pub fn compute_predictions<Builder>(
    mut commands: Commands,
    settings: Option<Res<PredictionSettings<Builder>>>,
    prediction_query: Query<(Entity, &Children, Option<&PredictionTracker<Builder>>)>,
    mut compute_events: EventReader<ComputePredictionEvent<Builder>>,
    query: Query<(Entity, &Builder, &Trajectory)>,
) where
    Builder: Component + Clone,
    Trajectory: BuildTrajectory<Builder>,
{
    for ComputePredictionEvent {
        root,
        duration: requested_duration,
        sync_count,
        ..
    } in compute_events.read().copied()
    {
        let Ok((root, entities, prediction)) = prediction_query.get(root) else {
            bevy::log::error!("Root entity not found, cannot compute prediction");
            continue;
        };
        if prediction.is_some() {
            bevy::log::warn!("Prediction computation already in progress");
            continue;
        }

        let Some(dt) = settings.as_ref().map(|s| s.dt) else {
            bevy::log::error!(
                "{} resource not found",
                std::any::type_name::<PredictionSettings<Builder>>()
            );
            continue;
        };

        // Different order results in slightly different acceleration values, which accumulate over
        // time. To prevent this we sort the queried bodies before computing the prediction to
        // ensure predictable results.
        let mut query = query.iter_many(entities).collect::<Vec<_>>();
        query.sort_by_key(|(entity, _, _)| *entity);
        let (entities, (mut builders, mut trajectories)): (Vec<_>, (Vec<_>, Vec<_>)) = query
            .into_iter()
            .map(|(entity, builder, trajectory)| {
                (entity, (builder.clone(), trajectory.continued()))
            })
            .unzip();
        let (sender, receiver) = crossbeam_channel::unbounded();

        let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread = {
            let paused = std::sync::Arc::downgrade(&paused);
            std::thread::spawn(move || {
                // This isn't exactly what we want, but it will do for now.
                let duration = adjusted_duration(
                    requested_duration,
                    trajectories.iter().map(Trajectory::granule),
                );

                let steps = (duration.to_seconds() / dt.to_seconds()).ceil() as usize;
                let sync_count = sync_count.clamp(1, steps);

                bevy::log::info!(
                    "Computing {} prediction for {}",
                    std::any::type_name::<Builder>(),
                    duration
                );
                let t0 = std::time::Instant::now();
                for step in (0..steps).rev() {
                    while paused
                        .upgrade()
                        .is_some_and(|p| p.load(std::sync::atomic::Ordering::Relaxed))
                    {
                        std::thread::sleep(std::time::Duration::from_millis(100));
                    }

                    Trajectory::build_trajectories(&mut trajectories, &mut builders, dt);

                    if step % steps.div_ceil(sync_count) == 0 {
                        let data = entities
                            .iter()
                            .zip(builders.iter())
                            .zip(trajectories.iter_mut())
                            .map(|((entity, builder), trajectory)| {
                                (
                                    *entity,
                                    builder.clone(),
                                    // TODO: We should re-use the previous allocation instead.
                                    std::mem::replace(trajectory, trajectory.continued()),
                                )
                            });

                        if sender.send(data.collect()).is_err() {
                            bevy::log::warn!("Failed to send prediction data, stopping thread");
                            break;
                        }
                    }
                }
                bevy::log::info!(
                    "Computing {} prediction took: {:?}",
                    std::any::type_name::<Builder>(),
                    t0.elapsed()
                );
            })
        };

        commands.entity(root).insert(PredictionTracker {
            thread,
            receiver,
            steps: sync_count,
            current: 0,
            paused,
        });
    }
}

pub fn process_predictions<Builder>(
    mut commands: Commands,
    mut tracker: Query<(Entity, &mut PredictionTracker<Builder>)>,
    mut query: Query<(&mut Builder, &mut Trajectory)>,
) where
    Builder: Component,
    Trajectory: BuildTrajectory<Builder>,
{
    for (root, mut prediction) in &mut tracker {
        let is_finished = prediction.thread.is_finished();

        let mut received = 0;
        for recv_data in prediction.receiver.try_iter() {
            received += 1;
            for (entity, recv_builder, recv_trajectory) in recv_data {
                if let Ok((mut builder, mut trajectory)) = query.get_mut(entity) {
                    *builder = recv_builder;
                    trajectory.join(recv_trajectory);
                }
            }
        }
        prediction.current += received;

        if is_finished {
            commands.entity(root).remove::<PredictionTracker<Builder>>();
        }
    }
}
