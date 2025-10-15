use crate::{
    MainState,
    prediction::{
        ExtendPredictionEvent, PredictionContext, PredictionPropagator, PredictionTracker,
    },
    simulation::SimulationTime,
};

use bevy::prelude::*;
use ftime::Duration;

#[derive(Resource)]
pub struct AutoExtendSettings<P> {
    pub enabled: bool,
    pub _marker: std::marker::PhantomData<fn(P)>,
}

impl<P> Default for AutoExtendSettings<P> {
    fn default() -> Self {
        Self {
            enabled: false,
            _marker: Default::default(),
        }
    }
}

impl<P> AutoExtendSettings<P> {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled,
            _marker: Default::default(),
        }
    }
}

pub struct AutoExtendPlugin<P>(pub std::marker::PhantomData<fn(P)>);

impl<P> Default for AutoExtendPlugin<P> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<P> Plugin for AutoExtendPlugin<P>
where
    P: PredictionPropagator,
{
    fn build(&self, app: &mut App) {
        app.insert_resource(AutoExtendSettings::<P>::new(true))
            .add_systems(
                First,
                auto_extend::<P>
                    .after(bevy::time::TimeSystem)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

fn auto_extend<P>(
    mut commands: Commands,
    time: Res<Time>,
    sim_time: Res<SimulationTime>,
    auto_extend: Res<AutoExtendSettings<P>>,
    query_tracker: Query<Option<&PredictionTracker<P>>, With<PredictionContext<P>>>,
) where
    P: PredictionPropagator,
{
    if !auto_extend.enabled || sim_time.paused {
        return;
    }

    let boundary = std::cmp::max_by(sim_time.start(), sim_time.end(), P::cmp);
    let delta = Duration::from_seconds(1.0) * sim_time.time_scale;
    let next = sim_time.current() + delta;
    if !P::cmp(&next, &boundary).is_ge() {
        return;
    }
    // TODO: Make this configurable or based on the system somehow.
    let min = Duration::from_days(1.0);
    let duration = (delta.abs() * time.delta_secs_f64()).max(min);

    for tracker in query_tracker.iter() {
        if tracker.is_some() {
            return;
        }

        // Synchronisation every 100 step.
        commands.trigger(ExtendPredictionEvent::<P>::all(duration, 100));
    }
}
