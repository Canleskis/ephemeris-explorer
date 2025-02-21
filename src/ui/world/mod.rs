mod plot;
mod tooltip;

pub use plot::*;
pub use tooltip::*;

use bevy::prelude::*;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WorldUiSet;

pub struct WorldUiPlugin;

impl Plugin for WorldUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((TooltipPlugin, PlotPlugin));
    }
}
