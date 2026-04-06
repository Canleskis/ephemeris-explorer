use crate::{
    MainState,
    prediction::{
        ComputePrediction, PredictionContext, PredictionPropagator, PredictionTracker,
        PropagationTarget, Synchronisation,
    },
    simulation::{SimulationTime, SimulationTimeSet},
};

use bevy::prelude::*;
use ephemeris::DirectionalPropagator;
use ftime::Duration;

#[derive(Resource)]
pub struct AutoExtendSettings<T> {
    pub enabled: bool,
    pub _marker: std::marker::PhantomData<fn(T)>,
}

impl<T> Default for AutoExtendSettings<T> {
    fn default() -> Self {
        Self {
            enabled: false,
            _marker: Default::default(),
        }
    }
}

impl<T> AutoExtendSettings<T> {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled,
            _marker: Default::default(),
        }
    }
}

pub struct AutoExtendPlugin<T>(pub std::marker::PhantomData<fn(T)>);

impl<T> Default for AutoExtendPlugin<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<T> Plugin for AutoExtendPlugin<T>
where
    T: PropagationTarget + 'static,
    T::Propagator: PredictionPropagator,
{
    fn build(&self, app: &mut App) {
        app.insert_resource(AutoExtendSettings::<T>::new(true))
            .add_systems(
                First,
                auto_extend::<T>
                    .after(SimulationTimeSet)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

#[allow(clippy::type_complexity)]
fn auto_extend<T>(
    mut commands: Commands,
    sim_time: Res<SimulationTime>,
    auto_extend: Res<AutoExtendSettings<T>>,
    query_tracker: Query<(Entity, Has<PredictionTracker<T>>), With<PredictionContext<T>>>,
    mut last_time_scale: Local<f64>,
) where
    T: PropagationTarget,
{
    if !auto_extend.enabled || sim_time.paused {
        return;
    }

    let look_ahead = Duration::from_seconds(2.0);
    let delta = look_ahead * sim_time.time_scale;

    let next = sim_time.current() + delta;
    let boundary = std::cmp::max_by(sim_time.start(), sim_time.end(), T::Propagator::cmp);
    if T::Propagator::cmp(&next, &boundary).is_lt() {
        return;
    }

    for (entity, has_tracker) in query_tracker.iter() {
        if has_tracker && *last_time_scale == sim_time.time_scale {
            continue;
        }

        commands.trigger(ComputePrediction::<T>::extend(
            entity,
            delta.abs(),
            Synchronisation::hertz(1000),
        ));
    }
    *last_time_scale = sim_time.time_scale;
}
