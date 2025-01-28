use bevy::input::mouse::MouseMotion;
use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;

#[derive(Component, Default)]
pub struct Selectable {
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
            .add_systems(
                PostUpdate,
                (
                    entity_picker.run_if(not(crate::ui::is_using_pointer)),
                    select_clicked,
                )
                    .chain()
                    .after(bevy::transform::TransformSystem::TransformPropagate),
            );
    }
}

fn entity_picker(
    mut click_entity_events: EventWriter<ClickEvent>,
    ray_map: Res<RayMap>,
    mouse_input: Res<ButtonInput<MouseButton>>,
    mut mouse_move: EventReader<MouseMotion>,
    query_camera: Query<&Projection, With<Camera>>,
    query_can_select: Query<(Entity, &GlobalTransform, &Selectable)>,
) {
    if !mouse_input.just_pressed(MouseButton::Left) || !mouse_move.is_empty() {
        mouse_move.clear();
        return;
    }

    let Ok(proj) = query_camera.get_single() else {
        return;
    };
    let fov = match proj {
        Projection::Perspective(p) => p.fov,
        _ => return,
    };

    let Some((_, ray)) = ray_map.iter().next() else {
        return;
    };
    let ray = bevy::math::bounding::RayCast3d::from_ray(*ray, f32::MAX);

    let clicked_entity = query_can_select
        .iter()
        .filter_map(|(entity, transform, clickable)| {
            let distance = transform.translation().distance(Vec3::from(ray.origin));
            let toi = ray.sphere_intersection_at(&bevy::math::bounding::BoundingSphere::new(
                transform.translation(),
                clickable.radius + distance * fov * 1e-2,
            ))?;

            Some((entity, clickable.index as f32 * toi))
        })
        .min_by(|(_, i1), (_, i2)| i1.total_cmp(i2))
        .map(|(entity, ..)| entity);

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
    query_can_select: Query<(&GlobalTransform, &Selectable)>,
) {
    let Ok(camera_transform) = query_camera.get_single() else {
        return;
    };

    for (transform, can_select) in &query_can_select {
        let direction = transform.translation() - camera_transform.translation();
        let radius = can_select.radius + direction.length_squared() / 100.0;
        gizmos.circle(
            transform
                .compute_transform()
                .looking_to(direction, Vec3::Y)
                .to_isometry(),
            radius,
            Color::WHITE,
        );
    }
}
