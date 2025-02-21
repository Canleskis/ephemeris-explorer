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

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WindowsUiSet;

pub struct WindowsUiPlugin;

impl Plugin for WindowsUiPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(Update, crate::ui::FixedUiSet.before(WindowsUiSet));
        app.add_plugins((
            ExportPlugin,
            PredictionPlannerPlugin,
            ShipSpawnerPlugin,
            EphemeridesDebugPlugin,
            BodyInfoPlugin,
        ));
    }
}
