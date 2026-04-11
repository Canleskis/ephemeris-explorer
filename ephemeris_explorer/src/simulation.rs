use crate::{
    MainState,
    dynamics::Trajectory,
    floating_origin::{BigSpace, CellCoord, Grid},
    rotation::Rotating,
};

use bevy::prelude::*;
use ephemeris::{BoundedTrajectory, EvaluateTrajectory};
use ftime::{Duration, Epoch};

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
    previous: Epoch,
    delta: std::time::Duration,
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
            previous: epoch,
            delta: std::time::Duration::ZERO,
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
    pub fn contains(&self, epoch: Epoch) -> bool {
        epoch >= self.start && epoch <= self.end
    }

    #[inline]
    pub fn current(&self) -> Epoch {
        self.current
    }

    #[inline]
    pub fn set_current(&mut self, epoch: Epoch) {
        self.current = epoch.clamp(self.start, self.end);
        self.real_time_scale =
            (self.current - self.previous).as_seconds() / self.delta.as_secs_f64();
    }

    #[inline]
    pub fn advance(&mut self) {
        let effective_delta = Duration::from(self.delta) * self.effective_time_scale();
        self.set_current(self.current + effective_delta);
    }

    #[inline]
    pub fn effective_time_scale(&self) -> f64 {
        if self.paused { 0.0 } else { self.time_scale }
    }

    #[inline]
    pub fn real_time_scale(&self) -> f64 {
        self.real_time_scale
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct SimulationTimeSystems;

#[derive(Default)]
pub struct SimulationTimePlugin;

impl Plugin for SimulationTimePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            First,
            (sync_bounds, advance_simulation_time)
                .chain()
                .after(bevy::time::TimeSystems)
                .in_set(SimulationTimeSystems)
                .run_if(in_state(MainState::Running)),
        )
        .add_systems(
            PostUpdate,
            (sync_position_to_time, sync_rotation_to_time).run_if(in_state(MainState::Running)),
        );
    }
}

pub fn sync_bounds(
    mut sim_time: ResMut<SimulationTime>,
    query: Query<&Trajectory, With<BoundsTime>>,
) {
    sim_time.start = query.iter().map(|e| e.start()).max().unwrap_or(Epoch::MIN);
    sim_time.end = query.iter().map(|e| e.end()).min().unwrap_or(Epoch::MAX);
}

pub fn advance_simulation_time(time: Res<Time>, mut sim_time: ResMut<SimulationTime>) {
    sim_time.previous = sim_time.current;
    sim_time.delta = time.delta();
    sim_time.advance();
}

pub fn sync_position_to_time(
    sim_time: Res<SimulationTime>,
    mut query: Query<(&mut Transform, &mut CellCoord, &Trajectory)>,
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

pub fn sync_rotation_to_time(
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
