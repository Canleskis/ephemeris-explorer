mod hierarchy;
mod time;

pub use hierarchy::*;
pub use time::*;

use crate::MainState;

use bevy::prelude::*;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct FixedUiSet;

pub struct FixedUiPlugin;

impl Plugin for FixedUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (time_controls, solar_system_hierarchy)
                .chain()
                .in_set(FixedUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}
