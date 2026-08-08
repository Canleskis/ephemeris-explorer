mod body;
mod debug;
mod errors;
mod export;
mod flight_plan;
mod planner;
mod settings;
mod spawner;

pub use body::*;
pub use debug::*;
pub use errors::*;
pub use export::*;
pub use flight_plan::*;
pub use planner::*;
pub use settings::*;
pub use spawner::*;

use bevy::prelude::*;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WindowsUiSet;

pub struct WindowsUiPlugin;

impl Plugin for WindowsUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            ExportPlugin,
            PredictionPlannerPlugin,
            ShipSpawnerPlugin,
            EphemeridesDebugPlugin,
            BodyInfoPlugin,
            FlightPlanWindowPlugin,
            SettingsPlugin,
            ErrorsPlugin,
        ));
    }
}
