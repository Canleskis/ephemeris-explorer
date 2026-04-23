use crate::{MainState, selection::Selected, ui::WorldUiSet};

use bevy::prelude::*;

#[derive(Resource)]
pub struct LabelSettings {
    pub enabled: bool,
}

#[derive(Component, Default)]
#[require(Visibility)]
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
                    .after(TransformSystems::Propagate),
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

// Ui layout being ordered before transform propagation means this is delayed by one frame.
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
            node.reborrow()
                .map_unchanged(|n| &mut n.display)
                .set_if_neq(Display::DEFAULT);
            node.reborrow()
                .map_unchanged(|n| &mut n.position_type)
                .set_if_neq(PositionType::Absolute);
            node.left = Val::Px(viewport_position.x);
            node.top = Val::Px(viewport_position.y);
        } else {
            node.map_unchanged(|n| &mut n.display)
                .set_if_neq(Display::None);
        }
    }
}

fn update_labels_visibility(
    settings: Res<LabelSettings>,
    query_camera: Query<&GlobalTransform, With<IsDefaultUiCamera>>,
    query_labels: Query<(&UiGlobalTransform, &ComputedNode), With<Label>>,
    query_labelled: Query<(Entity, &GlobalTransform, &Labelled, &LabelEntity)>,
    mut query_visibility: Query<&mut Visibility>,
) {
    let Ok(camera_transform) = query_camera.single() else {
        return;
    };

    let mut iter = query_visibility.iter_many_mut(query_labelled.iter().map(|i| **i.3));
    if !settings.enabled {
        while let Some(mut vis) = iter.fetch_next() {
            vis.set_if_neq(Visibility::Hidden);
        }
        return;
    } else {
        while let Some(mut vis) = iter.fetch_next() {
            vis.set_if_neq(Visibility::Inherited);
        }
    }

    // Sync labelled and label visibilities
    for (labelled_entity, .., label) in query_labelled.iter() {
        if let Ok(&visiblity) = query_visibility.get(labelled_entity)
            && let Ok(mut label_visiblity) = query_visibility.get_mut(**label)
        {
            label_visiblity.set_if_neq(visiblity);
        }
    }

    // Handle label overlap.
    for (labelled_entity, transform, labelled, label) in query_labelled.iter() {
        let Ok((ui_transform, node)) = query_labels.get(**label) else {
            continue;
        };

        if query_visibility
            .get(labelled_entity)
            .is_ok_and(|v| matches!(v, Visibility::Hidden))
        {
            continue;
        }

        let distance = transform
            .translation()
            .distance(camera_transform.translation());
        let rect = Rect::from_center_size(ui_transform.translation, node.size());

        let is_hidden = query_labelled
            .iter()
            .filter(|(.., e)| ***e != **label)
            .flat_map(|(_, other_transform, other, other_label)| {
                Some((
                    other_transform,
                    other,
                    query_labels.get(**other_label).ok()?,
                    query_visibility.get(**other_label).ok()?,
                ))
            })
            .any(
                |(other_pos, other_labelled, (other_transform, other_node), other_vis)| {
                    let other_distance = other_pos
                        .translation()
                        .distance(camera_transform.translation());

                    let other_rect =
                        Rect::from_center_size(other_transform.translation, other_node.size());

                    *other_vis != Visibility::Hidden
                        && !rect.intersect(other_rect).is_empty()
                        && distance * labelled.index as f32
                            > other_distance * other_labelled.index as f32
                },
            );

        if let Ok(mut vis) = query_visibility.get_mut(**label) {
            match is_hidden {
                true => vis.set_if_neq(Visibility::Hidden),
                false => vis.set_if_neq(Visibility::Inherited),
            };
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
