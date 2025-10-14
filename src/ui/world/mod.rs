mod labels;
mod picking;
mod plot;
mod tooltip;

pub use labels::*;
pub use picking::*;
pub use plot::*;
pub use tooltip::*;

use bevy::prelude::*;

#[derive(Resource, Clone, Copy, Default)]
pub struct WorldInteraction {
    pub using_pointer: bool,
    pub using_keyboard: bool,
}

pub fn world_ui_using_pointer(ui: Res<WorldInteraction>) -> bool {
    ui.using_pointer
}

pub fn world_ui_using_keyboard(ui: Res<WorldInteraction>) -> bool {
    ui.using_keyboard
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct WorldUiSet;

pub struct WorldUiPlugin;

impl Plugin for WorldUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WorldInteraction>().add_plugins((
            CustomPickingPlugin,
            TooltipPlugin,
            PlotPlugin,
            LabelsPlugin,
        ));
    }
}
