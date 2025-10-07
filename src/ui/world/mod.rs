mod labels;
mod plot;
mod tooltip;

pub use labels::*;
pub use plot::*;
pub use tooltip::*;

use bevy::prelude::*;

#[derive(Resource, Clone, Copy, Default)]
pub struct WorldUiInteraction {
    pub using_pointer: bool,
    pub using_keyboard: bool,
}

pub fn world_ui_using_pointer(ui: Res<WorldUiInteraction>) -> bool {
    ui.using_pointer
}

pub fn world_ui_using_keyboard(ui: Res<WorldUiInteraction>) -> bool {
    ui.using_keyboard
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WorldUiSet;

pub struct WorldUiPlugin;

impl Plugin for WorldUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WorldUiInteraction>()
            .add_plugins((TooltipPlugin, PlotPlugin, LabelsPlugin));
    }
}
