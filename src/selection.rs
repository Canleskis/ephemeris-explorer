use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;

use crate::ui::{HitData, PickingSet, PointerHit, PointerHover};

#[derive(Component, Default)]
pub struct Selectable {
    pub radius: f32,
    pub index: usize,
}

#[derive(Resource, Deref, DerefMut, Default)]
pub struct Selected(pub Option<Entity>);

#[derive(Default)]
pub struct SelectionPlugin;

impl Plugin for SelectionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Selected>()
            .add_systems(PostUpdate, select_hover_clicked.after(PickingSet));
    }
}

fn select_hover_clicked(
    mouse_input: Res<ButtonInput<MouseButton>>,
    mut mouse_move: EventReader<MouseMotion>,
    hover: Res<PointerHover>,
    mut selected: ResMut<Selected>,
) {
    if !mouse_input.just_pressed(MouseButton::Left) || !mouse_move.is_empty() {
        mouse_move.clear();
        return;
    }

    if let Some(PointerHit(entity, HitData::Body(_))) = &hover.0 {
        selected.0 = Some(*entity);
    }
}
