use crate::{
    ephemerides::Trajectory,
    floating_origin::{BigSpace, GridCell, ReferenceFrame},
    plot::EphemerisPlotConfig,
    MainState,
};

use bevy::prelude::*;
use hifitime::{Duration, Epoch};

#[derive(Resource)]
pub struct EphemeridesTime {
    pub paused: bool,
    pub time_scale: f64,
    real_time_scale: f64,
    start: Epoch,
    end: Epoch,
    current: Epoch,
}

impl EphemeridesTime {
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

pub struct EphemerisTimePlugin;

impl Plugin for EphemerisTimePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            First,
            (flow_time, sync_eph_time, sync_plot_config)
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

fn flow_time(time: Res<Time>, mut eph_time: ResMut<EphemeridesTime>) {
    let delta = Duration::from(time.delta());
    let previous = eph_time.current;

    if !eph_time.paused {
        let scaled_delta = eph_time.time_scale * delta;
        eph_time.set_epoch_clamped(previous + scaled_delta);
    }

    eph_time.real_time_scale =
        (eph_time.current - previous).total_nanoseconds() as f64 / delta.total_nanoseconds() as f64;
}

fn sync_eph_time(mut eph_time: ResMut<EphemeridesTime>, query: Query<&Trajectory>) {
    eph_time.set_start(query.iter().map(|e| e.start()).max().unwrap_or_default());
    eph_time.set_end(query.iter().map(|e| e.end()).min().unwrap_or_default());
}

fn sync_plot_config(mut config: ResMut<EphemerisPlotConfig>, time: Res<EphemeridesTime>) {
    config.current_time = time.current();
}

fn sync_position_to_time(
    eph_time: Res<EphemeridesTime>,
    mut query: Query<(&mut Transform, &mut GridCell, &Trajectory)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
) {
    if eph_time.start() >= eph_time.end() {
        return;
    }

    let root = root.single();
    for (mut transform, mut cell, ephemeris) in query.iter_mut() {
        let position = ephemeris
            .evaluate_position(eph_time.current())
            .expect("ephemerides time was out of bounds");
        (*cell, transform.translation) = root.translation_to_grid(position);
    }
}

use crate::load::Rotating;
fn sync_rotation_to_time(
    eph_time: Res<EphemeridesTime>,
    mut query: Query<(&mut Transform, &Rotating)>,
) {
    if eph_time.start() >= eph_time.end() {
        return;
    }

    for (mut transform, rotating) in query.iter_mut() {
        transform.rotation = rotating.at(eph_time.current()).as_quat();
    }
}
