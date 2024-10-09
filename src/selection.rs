use bevy::math::Dir3;
use bevy::prelude::*;

#[derive(Component, Default)]
pub struct Clickable {
    pub radius: f32,
    pub index: usize,
}

#[derive(Event, Deref, DerefMut)]
pub struct ClickEvent(pub Option<Entity>);

#[derive(Resource, Deref, DerefMut, Default)]
pub struct Selected(pub Option<Entity>);

pub struct SelectionPlugin;

impl Plugin for SelectionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Selected>()
            .add_event::<ClickEvent>()
            .add_systems(PostUpdate, (entity_picker, select_clicked).chain());
    }
}

fn entity_picker(
    ctx: bevy_egui::EguiContexts,
    mut click_entity_events: EventWriter<ClickEvent>,
    mouse_input: Res<ButtonInput<MouseButton>>,
    query_window: Query<&Window>,
    query_camera: Query<(&GlobalTransform, &Camera)>,
    query_can_select: Query<(Entity, &GlobalTransform, &Clickable)>,
) {
    if !mouse_input.just_pressed(MouseButton::Left)
        || ctx.try_ctx().is_some_and(|ctx| ctx.is_pointer_over_area())
    {
        return;
    }

    let Ok(window) = query_window.get_single() else {
        return;
    };
    let Ok((camera_transform, camera)) = query_camera.get_single() else {
        return;
    };

    let clicked_entity = window
        .cursor_position()
        .and_then(|position| camera.viewport_to_world(camera_transform, position))
        .and_then(|ray| {
            query_can_select
                .iter()
                .fold(None, |acc, (entity, transform, clickable)| {
                    let distance = transform.translation() - ray.origin;
                    let proj = ray.direction.dot(distance);
                    let mag = distance.length_squared();
                    let radius = clickable.radius + mag.sqrt() / 100.0;
                    let d =
                        (proj * proj + radius * radius >= mag).then_some((entity, clickable.index));

                    acc.filter(|&(_, index)| d.is_none() || index < clickable.index)
                        .or(d)
                })
                .map(|(entity, _)| entity)
        });

    click_entity_events.send(ClickEvent(clicked_entity));
}

fn select_clicked(mut click_events: EventReader<ClickEvent>, mut selected: ResMut<Selected>) {
    for &ClickEvent(entity) in click_events.read() {
        if let Some(entity) = entity {
            selected.replace(entity);
        }
    }
}

fn _show_pickable_zone(
    mut gizmos: Gizmos,
    query_camera: Query<&GlobalTransform, With<Camera>>,
    query_can_select: Query<(&GlobalTransform, &Clickable)>,
) {
    let Ok(camera_transform) = query_camera.get_single() else {
        return;
    };

    for (transform, can_select) in &query_can_select {
        let distance = transform.translation() - camera_transform.translation();
        let mag = distance.length_squared();
        let radius = can_select.radius + mag.sqrt() / 100.0;
        gizmos.circle(
            transform.translation(),
            Dir3::new_unchecked(distance.normalize()),
            radius,
            Color::WHITE,
        );
    }
}
