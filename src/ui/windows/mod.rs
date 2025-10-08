mod body;
mod debug;
mod errors;
mod export;
mod planner;
mod settings;
mod spawner;

pub use body::*;
pub use debug::*;
pub use errors::*;
pub use export::*;
pub use planner::*;
pub use settings::*;
pub use spawner::*;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WindowsUiSet;

pub struct WindowsUiPlugin;

impl Plugin for WindowsUiPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(EguiPrimaryContextPass, crate::ui::FixedUiSet.before(WindowsUiSet));
        app.add_plugins((
            ExportPlugin,
            PredictionPlannerPlugin,
            ShipSpawnerPlugin,
            EphemeridesDebugPlugin,
            BodyInfoPlugin,
            SettingsPlugin,
            ErrorsPlugin,
        ));
    }
}
