use crate::{
    MainState,
    auto_extend::{AutoExtendSettings, ExtendAllRequest},
    simulation::{SimulationTime, SimulationTimeSystems, advance_simulation_time},
};

use bevy::prelude::*;
use ftime::Epoch;

#[derive(Event, Debug, Clone, Copy)]
pub struct StartWarp {
    pub target: Epoch,
    pub duration: std::time::Duration,
}

impl StartWarp {
    #[inline]
    pub fn new(target: Epoch) -> Self {
        Self {
            target,
            duration: std::time::Duration::from_secs(2),
        }
    }

    #[inline]
    pub fn with_duration(mut self, duration: std::time::Duration) -> Self {
        self.duration = duration;
        self
    }
}

pub struct WarpPlugin;

impl Plugin for WarpPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(handle_warp_request).add_systems(
            First,
            advance_warp_time
                .in_set(SimulationTimeSystems)
                .after(advance_simulation_time)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Resource)]
pub struct WarpTime {
    pub start: Epoch,
    pub target: Epoch,
    pub duration: std::time::Duration,
    pub elapsed: std::time::Duration,
}

fn handle_warp_request(
    trigger: On<StartWarp>,
    mut commands: Commands,
    mut auto_extend: ResMut<AutoExtendSettings>,
    mut sim_time: ResMut<SimulationTime>,
) {
    let requested = trigger.event();

    auto_extend.enabled = false;
    sim_time.paused = true;
    commands.trigger(ExtendAllRequest::new(requested.target));
    commands.insert_resource(WarpTime {
        start: sim_time.current(),
        target: requested.target,
        duration: requested.duration,
        elapsed: std::time::Duration::ZERO,
    });
}

fn cubic_ease_in_out(t: f64) -> f64 {
    if t < 0.5 {
        4.0 * t * t * t
    } else {
        let f = 2.0 * t - 2.0;
        0.5 * f * f * f + 1.0
    }
}

fn advance_warp_time(
    mut commands: Commands,
    mut auto_extend: ResMut<AutoExtendSettings>,
    mut sim_time: ResMut<SimulationTime>,
    warp: Option<ResMut<WarpTime>>,
    time: Res<Time>,
) {
    let Some(mut warp) = warp else { return };

    let delta = time.delta();
    let next_elapsed = warp.elapsed + delta;
    let next_t = (next_elapsed.as_secs_f64() / warp.duration.as_secs_f64()).clamp(0.0, 1.0);
    let next_epoch = warp.start + (warp.target - warp.start) * cubic_ease_in_out(next_t);

    sim_time.set_current(next_epoch);

    if sim_time.contains(next_epoch) {
        warp.elapsed = next_elapsed;

        if next_t == 1.0 {
            auto_extend.enabled = true;
            commands.remove_resource::<WarpTime>();
        }
    }
}
