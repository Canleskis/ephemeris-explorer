mod body;
mod debug;
mod export;
mod planner;
mod spawner;

pub use body::*;
pub use debug::*;
pub use export::*;
pub use planner::*;
pub use spawner::*;

use bevy::prelude::*;

pub struct WindowsUiPlugin;

impl Plugin for WindowsUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            ExportPlugin,
            PredictionPlannerPlugin,
            ShipSpawnerPlugin,
            EphemeridesDebugPlugin,
            BodyInfoPlugin,
        ));
    }
}
