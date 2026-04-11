use crate::{
    MainState,
    prediction::{
        ComputePrediction, PredictionControllerOf, PredictionTracker, PropagationTarget,
        Synchronisation,
    },
    simulation::{SimulationTime, SimulationTimeSystems},
};

use bevy::prelude::*;
use ephemeris::DirectionalPropagator;
use ftime::{Duration, Epoch};

#[derive(Resource)]
pub struct ExtendSettings {
    pub enabled: bool,
}

/// Request for the simulation bounds to be extended to the given epoch by extending the prediction
/// for the given [`PropagationTarget`]s. If `forced` is false, requests will be ignored when one is
/// already being fulfilled. If true, a new request will result in any ongoing extension to be
/// cancelled, even if the new one is already fullfilled.
/// The `buffer` corresponds the additonal duration that will be computed whenever an extension is
/// requested.
#[derive(Debug, Clone, Copy, EntityEvent)]
pub struct ExtendRequest {
    pub entity: Entity,
    pub target: Epoch,
    pub forced: bool,
    pub buffer: Duration,
}

impl ExtendRequest {
    #[inline]
    pub fn new(entity: Entity, target: Epoch) -> Self {
        Self {
            entity,
            target,
            forced: true,
            buffer: Duration::ZERO,
        }
    }

    #[inline]
    pub fn with_force(mut self, forced: bool) -> Self {
        self.forced = forced;
        self
    }

    #[inline]
    pub fn with_buffer(mut self, buffer: Duration) -> Self {
        self.buffer = buffer.abs();
        self
    }
}

/// Request for the simulation bounds to be extended to the given epoch by extending the prediction
/// for all registered [`PredictionTarget`]s.
#[derive(Debug, Clone, Copy, Event)]
pub struct ExtendAllRequest {
    pub target: Epoch,
    pub forced: bool,
    pub buffer: Duration,
}

impl ExtendAllRequest {
    #[inline]
    pub fn new(target: Epoch) -> Self {
        Self {
            target,
            forced: true,
            buffer: Duration::ZERO,
        }
    }

    #[inline]
    pub fn with_forced(mut self, forced: bool) -> Self {
        self.forced = forced;
        self
    }

    #[inline]
    pub fn with_buffer(mut self, buffer: Duration) -> Self {
        self.buffer = buffer.abs();
        self
    }
}

pub struct ExtendPlugin<T: PropagationTarget>(pub std::marker::PhantomData<fn(T)>);

impl<T: PropagationTarget> Default for ExtendPlugin<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<T: PropagationTarget> Plugin for ExtendPlugin<T> {
    fn build(&self, app: &mut App) {
        app.add_observer(handle_extend_request::<T>)
            .add_observer(handle_extend_all_request::<T>);
    }
}

fn handle_extend_request<T>(
    trigger: On<ExtendRequest>,
    mut commands: Commands,
    sim_time: Res<SimulationTime>,
    query: Query<&PredictionTracker<T>>,
) where
    T: PropagationTarget,
{
    let request = trigger.event();

    let target = request.target;
    let boundary = std::cmp::max_by(sim_time.start(), sim_time.end(), T::Propagator::cmp);
    let duration = T::Propagator::distance(boundary, target);
    let should_extend = duration > Duration::ZERO;

    let is_predicting = query.contains(request.entity);

    if (!is_predicting && should_extend) || (is_predicting && request.forced) {
        commands.trigger(ComputePrediction::<T>::extend(
            trigger.event().entity,
            (duration + request.buffer).max(Duration::ZERO),
            Synchronisation::hertz(1000),
        ));
    }
}

fn handle_extend_all_request<T>(
    trigger: On<ExtendAllRequest>,
    mut commands: Commands,
    query_controller: Query<Entity, With<PredictionControllerOf<T>>>,
) where
    T: PropagationTarget,
{
    for entity in query_controller.iter() {
        commands.trigger(ExtendRequest {
            entity,
            target: trigger.event().target,
            forced: trigger.event().forced,
            buffer: trigger.event().buffer,
        });
    }
}

#[derive(Resource)]
pub struct AutoExtendSettings {
    pub enabled: bool,
}

impl Default for AutoExtendSettings {
    #[inline]
    fn default() -> Self {
        Self { enabled: true }
    }
}

impl AutoExtendSettings {
    #[inline]
    pub fn new(enabled: bool) -> Self {
        Self { enabled }
    }
}

pub struct AutoExtendPlugin;

impl Plugin for AutoExtendPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(AutoExtendSettings::new(true))
            .add_systems(
                First,
                auto_extend
                    .after(SimulationTimeSystems)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

#[allow(clippy::type_complexity)]
fn auto_extend(
    mut commands: Commands,
    sim_time: Res<SimulationTime>,
    settings: Res<AutoExtendSettings>,
    mut previous_time_scale: Local<f64>,
) {
    if !settings.enabled || sim_time.paused {
        return;
    }

    let look_ahead = Duration::from_seconds(5.0);
    let delta = look_ahead * sim_time.time_scale;

    commands.trigger(
        ExtendAllRequest::new(sim_time.current() + delta)
            .with_forced(*previous_time_scale != sim_time.time_scale)
            .with_buffer(delta.abs()),
    );

    *previous_time_scale = sim_time.time_scale;
}
