use crate::{
    prediction::{ExtendPredictionEvent, PredictionTracker, TrajectoryBuilder},
    time::SimulationTime,
    MainState,
};

use bevy::prelude::*;
use hifitime::Duration;

#[derive(Resource)]
pub struct AutoExtendSettings<B> {
    pub enabled: bool,
    pub _marker: std::marker::PhantomData<fn(B)>,
}

impl<B> Default for AutoExtendSettings<B> {
    fn default() -> Self {
        Self {
            enabled: false,
            _marker: Default::default(),
        }
    }
}

impl<B> AutoExtendSettings<B> {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled,
            _marker: Default::default(),
        }
    }
}

pub struct AutoExtendPlugin<B>(pub std::marker::PhantomData<fn(B)>);

impl<B> Default for AutoExtendPlugin<B> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<B> Plugin for AutoExtendPlugin<B>
where
    B: TrajectoryBuilder + Component,
{
    fn build(&self, app: &mut App) {
        app.insert_resource(AutoExtendSettings::<B>::new(true))
            .add_systems(
                First,
                auto_extend::<B>
                    .after(bevy::time::TimeSystem)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

fn auto_extend<B>(
    mut commands: Commands,
    time: Res<Time>,
    sim_time: Res<SimulationTime>,
    auto_extend: Res<AutoExtendSettings<B>>,
    root: Query<Option<&PredictionTracker<B>>, With<B>>,
) where
    B: TrajectoryBuilder + Component,
{
    if !auto_extend.enabled || sim_time.paused {
        return;
    }

    let boundary = std::cmp::max_by(sim_time.start(), sim_time.end(), B::cmp);
    let delta = sim_time.time_scale.abs() * Duration::from_seconds(1.0);
    let next = B::add(sim_time.current(), delta);
    if !B::cmp(&next, &boundary).is_ge() {
        return;
    }
    // TODO: Make this configurable or based on the system somehow.
    let min = Duration::from_days(1.0);
    let duration = (delta * time.delta_seconds_f64().min(0.1)).max(min);

    for tracker in root.iter() {
        if tracker.is_some() {
            return;
        }

        commands.trigger(ExtendPredictionEvent::<B>::all(duration, 100));
    }
}
