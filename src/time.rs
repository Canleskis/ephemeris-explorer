use crate::{
    floating_origin::{BigSpace, Grid, GridCell},
    prediction::{Trajectory, TrajectoryData},
    rotation::Rotating,
    MainState,
};

use bevy::prelude::*;
use hifitime::{Duration, Epoch};

#[derive(Component)]
pub struct BoundsTime;

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
            paused: false,
            time_scale: 1.0,
            real_time_scale: 1.0,
            start: epoch,
            end: epoch,
            current: epoch,
        }
    }

    #[inline]
    pub fn start(&self) -> Epoch {
        self.start
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        self.end
    }

    #[inline]
    pub fn current(&self) -> Epoch {
        self.current
    }

    #[inline]
    pub fn set_current_clamped(&mut self, epoch: Epoch) {
        self.current = epoch.clamp(self.start, self.end);
    }

    #[inline]
    pub fn real_time_scale(&self) -> f64 {
        self.real_time_scale
    }
}

pub struct SimulationTimePlugin;

impl Plugin for SimulationTimePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            First,
            (sync_bounds, flow_time)
                .chain()
                .after(bevy::time::TimeSystem)
                .run_if(in_state(MainState::Running)),
        )
        .add_systems(
            PostUpdate,
            (sync_position_to_time, sync_rotation_to_time).run_if(in_state(MainState::Running)),
        );
    }
}

fn sync_bounds(mut sim_time: ResMut<SimulationTime>, query: Query<&Trajectory, With<BoundsTime>>) {
    sim_time.start = query
        .iter()
        .map(|e| e.start())
        .max()
        .unwrap_or_else(|| Epoch::from_tai_duration(Duration::MIN));
    sim_time.end = query
        .iter()
        .map(|e| e.end())
        .min()
        .unwrap_or_else(|| Epoch::from_tai_duration(Duration::MAX));
}

fn flow_time(time: Res<Time>, mut sim_time: ResMut<SimulationTime>) {
    let delta = Duration::from(time.delta());
    let previous = sim_time.current;

    if !sim_time.paused {
        let scaled_delta = sim_time.time_scale * delta;
        sim_time.set_current_clamped(previous + scaled_delta);
    }

    sim_time.real_time_scale =
        (sim_time.current - previous).total_nanoseconds() as f64 / delta.total_nanoseconds() as f64;
}

fn sync_position_to_time(
    sim_time: Res<SimulationTime>,
    mut query: Query<(&mut Transform, &mut GridCell, &Trajectory)>,
    root: Single<&Grid, With<BigSpace>>,
) {
    if sim_time.start() >= sim_time.end() {
        return;
    }

    for (mut transform, mut cell, trajectory) in query.iter_mut() {
        let time = sim_time
            .current()
            .clamp(trajectory.start(), trajectory.end());
        // Safe to unwrap because we just clampled the time to the trajectory bounds.
        let position = trajectory.position(time).unwrap();
        (*cell, transform.translation) = root.translation_to_grid(position);
    }
}

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
