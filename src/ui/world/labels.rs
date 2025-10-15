use crate::{MainState, selection::Selected, ui::WorldUiSet};

use bevy::prelude::*;

#[derive(Resource)]
pub struct LabelSettings {
    pub enabled: bool,
}

#[derive(Component, Default)]
pub struct Labelled {
    pub font: TextFont,
    pub colour: TextColor,
    pub offset: Vec2,
    pub index: usize,
}

pub struct LabelsPlugin;

impl Plugin for LabelsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(LabelSettings { enabled: true })
            .add_systems(
                PreUpdate,
                (spawn_labels, despawn_labels).chain().in_set(WorldUiSet),
            )
            .add_systems(
                PostUpdate,
                (
                    update_labels_text,
                    (update_labels_position, update_labels_visibility).chain(),
                    update_labels_color,
                )
                    .in_set(WorldUiSet)
                    .run_if(in_state(MainState::Running))
                    .after(bevy::transform::TransformSystem::TransformPropagate),
            );
    }
}

#[derive(Component, Deref, DerefMut)]
struct LabelEntity(Entity);

#[derive(Component)]
struct Label;

/// We should ideally be using billboards here, but none of the available options suit the project's
/// needs for now. Specifically, we want billboards with a fixed size always facing the camera.
fn spawn_labels(
    mut commands: Commands,
    query_labelled: Query<(Entity, &Name, &Labelled), Added<Labelled>>,
) {
    for (entity, name, labelled) in &query_labelled {
        let id = commands
            .spawn((
                Name::new(format!("{name} label")),
                Text::new(name.to_string()),
                labelled.font.clone(),
                labelled.colour,
                Label,
            ))
            .id();

        commands.entity(entity).insert(LabelEntity(id));
    }
}

fn despawn_labels(
    mut commands: Commands,
    query: Query<(Entity, &LabelEntity), Added<LabelEntity>>,
    mut removed_labelled: RemovedComponents<LabelEntity>,
    mut local: Local<bevy::ecs::entity::EntityHashMap<Entity>>,
) {
    for (entity, label) in &query {
        local.insert(entity, **label);
    }

    for entity in removed_labelled.read() {
        if let Some(label_entity) = local.get(&entity) {
            commands.entity(*label_entity).despawn();
            local.remove(&entity);
        }
    }
}

fn update_labels_text(
    query_changed: Query<(&Name, &LabelEntity), Changed<Name>>,
    mut query_labels: Query<&mut Text>,
) {
    for (name, label_entity) in &query_changed {
        if let Ok(mut text) = query_labels.get_mut(**label_entity) {
            **text = name.to_string();
        }
    }
}

fn update_labels_position(
    query_camera: Query<(&Camera, &GlobalTransform), With<IsDefaultUiCamera>>,
    query_labelled: Query<(&LabelEntity, &Labelled, &GlobalTransform)>,
    mut query_labels: Query<(&mut Node, &ComputedNode)>,
) {
    let Ok((camera, camera_transform)) = query_camera.single() else {
        return;
    };

    for (entity, label, transform) in &query_labelled {
        let Ok((mut node, computed_node)) = query_labels.get_mut(**entity) else {
            continue;
        };

        let viewport_position = camera
            .world_to_viewport(
                camera_transform,
                transform.translation()
                    + camera_transform
                        .affine()
                        .matrix3
                        .mul_vec3(label.offset.extend(0.0)),
            )
            .map(|position| position - computed_node.size() / 2.0);

        if let Ok(viewport_position) = viewport_position {
            node.display = Display::DEFAULT;
            node.position_type = PositionType::Absolute;
            node.left = Val::Px(viewport_position.x);
            node.top = Val::Px(viewport_position.y);
        } else {
            node.display = Display::None;
        }
    }
}

fn update_labels_visibility(
    settings: Res<LabelSettings>,
    query_camera: Query<&GlobalTransform, With<IsDefaultUiCamera>>,
    query_labels: Query<(&GlobalTransform, &ComputedNode), With<Label>>,
    query_labelled: Query<(&GlobalTransform, &Labelled, &LabelEntity)>,
    mut query_visibility: Query<&mut Visibility, With<Label>>,
) {
    let Ok(camera_transform) = query_camera.single() else {
        return;
    };

    if !settings.enabled {
        for mut vis in query_visibility.iter_mut() {
            *vis = Visibility::Hidden;
        }
        return;
    } else {
        for mut vis in query_visibility.iter_mut() {
            *vis = Visibility::Visible;
        }
    }

    for (pos, labelled, entity) in query_labelled.iter() {
        let Ok((transform, node)) = query_labels.get(**entity) else {
            continue;
        };

        let distance = pos.translation().distance(camera_transform.translation());
        let rect = Rect::from_center_size(transform.translation().truncate(), node.size());

        let is_overlapped = query_labelled
            .iter()
            .filter(|(.., e)| ***e != **entity)
            .flat_map(|(pos, labelled, entity)| {
                Some((
                    pos,
                    labelled,
                    query_labels.get(**entity).ok()?,
                    query_visibility.get(**entity).ok()?,
                ))
            })
            .any(
                |(other_pos, other_labelled, (other_transform, other_node), other_vis)| {
                    let other_distance = other_pos
                        .translation()
                        .distance(camera_transform.translation());

                    let other_rect = Rect::from_center_size(
                        other_transform.translation().truncate(),
                        other_node.size(),
                    );

                    *other_vis != Visibility::Hidden
                        && !rect.intersect(other_rect).is_empty()
                        && distance * labelled.index as f32
                            > other_distance * other_labelled.index as f32
                },
            );

        let Ok(mut vis) = query_visibility.get_mut(**entity) else {
            continue;
        };

        match is_overlapped {
            true => *vis = Visibility::Hidden,
            false => *vis = Visibility::Visible,
        }
    }
}

fn update_labels_color(
    mut query_labels: Query<&mut TextColor>,
    query_labelled: Query<&LabelEntity>,
    selected: Res<Selected>,
    mut deselected: Local<Option<Entity>>,
) {
    if !selected.is_changed() || **selected == *deselected {
        return;
    }

    let mut set_label_color = |entity, color| {
        if let Ok(mut colour) = query_labelled
            .get(entity)
            .and_then(|e| query_labels.get_mut(**e))
        {
            *colour = color;
        }
    };

    if let Some(entity) = **selected {
        set_label_color(entity, LinearRgba::RED.into());
    }
    if let Some(entity) = *deselected {
        set_label_color(entity, LinearRgba::WHITE.into());
    }

    *deselected = **selected;
}
