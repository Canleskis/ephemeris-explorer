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

pub trait GizmosTriangleExt {
    fn triangle_3d(&mut self, isometry: impl Into<Isometry3d>, size: Vec2, color: impl Into<Color>);
}

impl<Config, Clear> GizmosTriangleExt for Gizmos<'_, '_, Config, Clear>
where
    Config: GizmoConfigGroup,
    Clear: 'static + Send + Sync,
{
    #[inline]
    fn triangle_3d(
        &mut self,
        isometry: impl Into<Isometry3d>,
        size: Vec2,
        color: impl Into<Color>,
    ) {
        let isometry = isometry.into();
        let [top, br, bl] = triangle_inner(size).map(|v| isometry * v.extend(0.0));
        self.lineloop([top, br, bl], color);
    }
}

#[inline]
fn triangle_inner(size: Vec2) -> [Vec2; 3] {
    let half = size / 2.0;
    let top = Vec2::new(0.0, half.y);
    let br = Vec2::new(half.x, -half.y);
    let bl = Vec2::new(-half.x, -half.y);
    [top, br, bl]
}
