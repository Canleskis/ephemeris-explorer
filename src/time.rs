use crate::{
    floating_origin::{BigSpace, GridCell, ReferenceFrame},
    plot::TrajectoryPlotConfig,
    prediction::Trajectory,
    MainState,
};

use bevy::prelude::*;
use hifitime::{Duration, Epoch};

#[derive(Resource)]
pub struct SimulationTime {
    pub paused: bool,
    pub time_scale: f64,
    real_time_scale: f64,
    start: Epoch,
    end: Epoch,
    current: Epoch,
}

impl SimulationTime {
    pub fn new(epoch: Epoch) -> Self {
        Self {
            current: epoch,
            paused: false,
            time_scale: 1.0,
            real_time_scale: 1.0,
            start: epoch,
            end: epoch,
        }
    }

    pub fn start(&self) -> Epoch {
        self.start
    }

    pub fn end(&self) -> Epoch {
        self.end
    }

    pub fn current(&self) -> Epoch {
        self.current
    }

    pub fn set_epoch_clamped(&mut self, epoch: Epoch) {
        self.current = epoch.clamp(self.start, self.end);
    }

    pub fn real_time_scale(&self) -> f64 {
        self.real_time_scale
    }

    fn set_start(&mut self, start: Epoch) {
        self.start = start;
        self.current = self.current.max(start);
    }

    fn set_end(&mut self, end: Epoch) {
        self.end = end;
        self.current = self.current.min(end);
    }
}

pub struct SimulationTimePlugin;

impl Plugin for SimulationTimePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            First,
            (flow_time, sync_sim_time, sync_plot_config)
                .chain()
                .after(bevy::time::TimeSystem)
                .run_if(in_state(MainState::Running)),
        )
        .add_systems(
            Update,
            (sync_position_to_time, sync_rotation_to_time).run_if(in_state(MainState::Running)),
        );
    }
}

fn flow_time(time: Res<Time>, mut sim_time: ResMut<SimulationTime>) {
    let delta = Duration::from(time.delta());
    let previous = sim_time.current;

    if !sim_time.paused {
        let scaled_delta = sim_time.time_scale * delta;
        sim_time.set_epoch_clamped(previous + scaled_delta);
    }

    sim_time.real_time_scale =
        (sim_time.current - previous).total_nanoseconds() as f64 / delta.total_nanoseconds() as f64;
}

fn sync_sim_time(mut sim_time: ResMut<SimulationTime>, query: Query<&Trajectory>) {
    sim_time.set_start(query.iter().map(|e| e.start()).max().unwrap_or_default());
    sim_time.set_end(query.iter().map(|e| e.end()).min().unwrap_or_default());
}

fn sync_plot_config(mut config: ResMut<TrajectoryPlotConfig>, time: Res<SimulationTime>) {
    config.current_time = time.current();
}

fn sync_position_to_time(
    sim_time: Res<SimulationTime>,
    mut query: Query<(&mut Transform, &mut GridCell, &Trajectory)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
) {
    if sim_time.start() >= sim_time.end() {
        return;
    }

    let root = root.single();
    for (mut transform, mut cell, trajectory) in query.iter_mut() {
        let position = trajectory
            .evaluate_position(sim_time.current())
            .expect("simulation time was out of bounds");
        (*cell, transform.translation) = root.translation_to_grid(position);
    }
}

use crate::load::Rotating;
fn sync_rotation_to_time(
    sim_time: Res<SimulationTime>,
    mut query: Query<(&mut Transform, &Rotating)>,
) {
    if sim_time.start() >= sim_time.end() {
        return;
    }

    for (mut transform, rotating) in query.iter_mut() {
        transform.rotation = rotating.at(sim_time.current()).as_quat();
    }
}
