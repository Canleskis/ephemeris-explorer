pub mod ephemeris;
pub mod integration;

pub use ephemeris::*;
pub use integration::*;

use bevy::prelude::*;
use hifitime::Duration;

#[derive(Event, Clone, Copy)]
pub struct ComputeEphemeridesEvent {
    pub root: Entity,
    pub duration: Duration,
    pub sync_count: usize,
}

#[derive(Resource)]
pub struct EphemeridesComputeSettings {
    pub dt: Duration,
}

pub struct EphemerisComputePlugin;

impl Plugin for EphemerisComputePlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<ComputeEphemeridesEvent>()
            .add_systems(Update, (compute_ephemerides, process_ephemerides).chain());
    }
}

pub fn adjusted_duration(duration: Duration, granules: impl Iterator<Item = Duration>) -> Duration {
    granules
        .map(|granule| (duration.to_seconds() / granule.to_seconds()).ceil() * granule)
        .max()
        .unwrap()
}

type PredictionData = Vec<(Entity, EphemerisBuilder, Trajectory)>;

#[derive(Component, Debug)]
pub struct PredictionTracking {
    thread: std::thread::JoinHandle<()>,
    receiver: crossbeam_channel::Receiver<PredictionData>,
    paused: std::sync::Arc<std::sync::atomic::AtomicBool>,
    steps: usize,
    current: usize,
}

impl PredictionTracking {
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

pub fn compute_ephemerides(
    mut commands: Commands,
    settings: Option<Res<EphemeridesComputeSettings>>,
    prediction_query: Query<(Entity, &Children, Option<&PredictionTracking>)>,
    mut compute_events: EventReader<ComputeEphemeridesEvent>,
    query: Query<(Entity, &EphemerisBuilder, &Trajectory)>,
) {
    for ComputeEphemeridesEvent {
        root,
        duration,
        sync_count,
    } in compute_events.read().copied()
    {
        let Ok((root, entities, prediction)) = prediction_query.get(root) else {
            bevy::log::warn!("Root entity not found");
            continue;
        };
        if prediction.is_some() {
            bevy::log::warn!("Ephemerides computation already in progress");
            continue;
        }

        let dt = settings.as_ref().unwrap().dt;

        // Different order results in slightly different acceleration values, which accumulate over
        // time. To prevent this we could sort the queried bodies before computing the ephemerides.
        let (entities, (mut builders, mut trajectories)): (Vec<_>, (Vec<_>, Vec<_>)) = query
            .iter_many(entities)
            .map(|(entity, builder, trajectory)| (entity, (*builder, trajectory.continued())))
            .unzip();

        let duration = adjusted_duration(duration, trajectories.iter().map(Trajectory::granule));

        let steps = (duration.to_seconds() / dt.to_seconds()).ceil() as usize;
        let sync_count = sync_count.clamp(1, steps);

        let (sender, receiver) = crossbeam_channel::unbounded();

        let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread = {
            let paused = std::sync::Arc::downgrade(&paused);
            std::thread::spawn(move || {
                bevy::log::info!("Computing ephemerides for {}", duration);
                let t0 = std::time::Instant::now();
                for step in (0..steps).rev() {
                    while paused
                        .upgrade()
                        .is_some_and(|p| p.load(std::sync::atomic::Ordering::Relaxed))
                    {
                        std::thread::sleep(std::time::Duration::from_millis(100));
                    }

                    Perfl::integrate(dt, &mut builders, EphemerisBuilder::acceleration);
                    builders
                        .iter_mut()
                        .zip(trajectories.iter_mut())
                        .for_each(|(builder, trajectory)| builder.update_trajectory(trajectory));

                    if step % steps.div_ceil(sync_count) == 0 {
                        let data = entities
                            .iter()
                            .zip(builders.iter())
                            .zip(trajectories.iter_mut())
                            .map(|((entity, builder), trajectory)| {
                                (*entity, *builder, trajectory.split_off(0))
                            });

                        if sender.send(data.collect()).is_err() {
                            bevy::log::warn!("Failed to send ephemerides data, stopping thread");
                            break;
                        }
                    }
                }
                bevy::log::info!("Computing ephemerides took: {:?}", t0.elapsed());
            })
        };

        commands.entity(root).insert(PredictionTracking {
            thread,
            receiver,
            steps: sync_count,
            current: 0,
            paused,
        });
    }
}

pub fn process_ephemerides(
    mut commands: Commands,
    mut predictions: Query<(Entity, &mut PredictionTracking)>,
    mut query: Query<(&mut EphemerisBuilder, &mut Trajectory)>,
) {
    for (root, mut prediction) in &mut predictions {
        let is_finished = prediction.thread.is_finished();

        let mut received = 0;
        for recv_data in prediction.receiver.try_iter() {
            received += 1;
            for (entity, recv_builder, segment) in recv_data {
                if let Ok((mut builder, mut trajectory)) = query.get_mut(entity) {
                    *builder = recv_builder;
                    trajectory.append(segment);
                }
            }
        }
        prediction.current += received;

        if is_finished {
            let approx_size = query.iter().map(|(_, data)| data.size()).sum::<usize>();
            bevy::log::info!("Approx ephemerides size: {} KB", approx_size / 1024);
            commands.entity(root).remove::<PredictionTracking>();
        }
    }
}
