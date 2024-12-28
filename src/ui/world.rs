use crate::{
    flight_plan::{Burn, FlightPlan, FlightPlanChanged},
    load::SystemRoot,
    plot::{trajectory_picking, PlotPoints, TrajectoryHitPoint, TrajectoryPlot, PICK_THRESHOLD},
    prediction::{Trajectory, TrajectoryData},
    selection::Selected,
    time::SimulationTime,
    ui::nformat,
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use hifitime::Duration;

#[derive(Component, Default)]
pub struct Labelled {
    pub style: TextStyle,
    pub offset: Vec2,
    pub index: usize,
}

pub struct WorldUiPlugin;

impl Plugin for WorldUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, (spawn_labels, despawn_labels).chain())
            .add_systems(
                PostUpdate,
                (
                    draw_intersections
                        .before(bevy_egui::EguiSet::ProcessOutput)
                        .after(trajectory_picking),
                    (update_labels_position, hide_overlapped_labels).chain(),
                    update_labels_color,
                )
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
/// needs for now.
fn spawn_labels(
    mut commands: Commands,
    query_labelled: Query<(Entity, &Name, &Labelled), Added<Labelled>>,
) {
    for (entity, name, labelled) in &query_labelled {
        let id = commands
            .spawn((
                Name::new(format!("{} label", name)),
                TextBundle {
                    text: Text::from_section(name.to_string(), labelled.style.clone()),
                    ..default()
                },
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
    mut local: Local<bevy::utils::EntityHashMap<Entity, Entity>>,
) {
    for (entity, label) in &query {
        local.insert(entity, **label);
    }

    for entity in removed_labelled.read() {
        if let Some(label_entity) = local.get(&entity) {
            commands.entity(*label_entity).despawn_recursive();
            local.remove(&entity);
        }
    }
}

fn update_labels_position(
    query_camera: Query<(&Camera, &GlobalTransform), With<IsDefaultUiCamera>>,
    query_labelled: Query<(&LabelEntity, &Labelled, &GlobalTransform)>,
    mut query_labels: Query<(&mut Style, &Node)>,
) {
    let Ok((camera, camera_transform)) = query_camera.get_single() else {
        return;
    };

    for (entity, label, transform) in &query_labelled {
        let Ok((mut style, node)) = query_labels.get_mut(**entity) else {
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
            .map(|position| position - node.size() / 2.0);

        if let Some(viewport_position) = viewport_position {
            style.position_type = PositionType::Absolute;
            style.left = Val::Px(viewport_position.x);
            style.top = Val::Px(viewport_position.y);
            style.display = Display::Flex;
        } else {
            style.display = Display::None;
        }
    }
}

fn hide_overlapped_labels(
    query_camera: Query<&GlobalTransform, With<IsDefaultUiCamera>>,
    query_labels: Query<(&GlobalTransform, &Node), With<Label>>,
    query_labelled: Query<(&GlobalTransform, &Labelled, &LabelEntity)>,
    mut query_visibility: Query<&mut Visibility, With<Label>>,
    kb: Res<ButtonInput<KeyCode>>,
    mut hide_all: Local<bool>,
) {
    let Ok(camera_transform) = query_camera.get_single() else {
        return;
    };

    if kb.just_pressed(KeyCode::KeyH) {
        *hide_all = !*hide_all;
    }

    for (pos, labelled, entity) in &query_labelled {
        let Ok((transform, node)) = query_labels.get(**entity) else {
            continue;
        };

        let distance = pos.translation().distance(camera_transform.translation());
        let rect = node.logical_rect(transform);

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

                    let other_rect = other_node.logical_rect(other_transform);

                    *other_vis != Visibility::Hidden
                        && !rect.intersect(other_rect).is_empty()
                        && distance * labelled.index as f32
                            > other_distance * other_labelled.index as f32
                },
            );

        let Ok(mut vis) = query_visibility.get_mut(**entity) else {
            continue;
        };

        match is_overlapped || *hide_all {
            true => *vis = Visibility::Hidden,
            false => *vis = Visibility::Visible,
        }
    }
}

fn update_labels_color(
    mut query_labels: Query<&mut Text>,
    query_labelled: Query<&LabelEntity>,
    selected: Res<Selected>,
    mut deselected: Local<Option<Entity>>,
) {
    if !selected.is_changed() || **selected == *deselected {
        return;
    }

    let mut set_label_color = |entity, color| {
        if let Ok(mut text) = query_labelled
            .get(entity)
            .and_then(|e| query_labels.get_mut(**e))
        {
            text.sections[0].style.color = color;
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

#[expect(clippy::too_many_arguments)]
fn draw_intersections(
    mut contexts: EguiContexts,
    mut gizmos: Gizmos,
    mut commands: Commands,
    input: Res<ButtonInput<MouseButton>>,
    mut sim_time: ResMut<SimulationTime>,
    mut events: EventReader<TrajectoryHitPoint>,
    mut query_points: Query<(&Name, &PlotPoints, &TrajectoryPlot, Option<&mut FlightPlan>)>,
    query_trajectory: Query<&Trajectory>,
    query_camera: Query<(&GlobalTransform, &Camera, &Projection)>,
    mut persisted: Local<Vec<TrajectoryHitPoint>>,
    root: Query<Entity, With<SystemRoot>>,
) {
    let Ok((camera_transform, camera, proj)) = query_camera.get_single() else {
        return;
    };
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let root = root.single();

    let fov = match proj {
        Projection::Perspective(p) => p.fov,
        _ => return,
    };

    let mut buffer = events.read().cloned().collect::<Vec<_>>();
    if let Some(min_hit) = buffer
        .iter()
        .min_by(|a, b| a.separation.total_cmp(&b.separation))
        .cloned()
    {
        let threshold = 0.01;
        let min_distance = min_hit.distance * (1.0 - threshold);
        let max_distance = min_hit.distance * (1.0 + threshold);
        buffer.retain(|hit| {
            hit.entity == min_hit.entity && (min_distance..=max_distance).contains(&hit.distance)
        });
        if input.just_pressed(MouseButton::Left) && !ctx.is_pointer_over_area() {
            persisted.clear();
            persisted.extend(buffer.iter().cloned());
        }
    }

    for (i, hits) in [&mut buffer, &mut persisted].into_iter().enumerate() {
        if hits.is_empty() {
            continue;
        }
        let is_persisted = i == 1;
        // All hits are on the same entity.
        let entity = hits[0].entity;
        let (name, points, plot, mut flight_plan) = query_points.get_mut(entity).unwrap();
        let trajectory = query_trajectory.get(entity).unwrap();
        let ref_trajectory = plot.reference.and_then(|r| query_trajectory.get(r).ok());
        let window_data = hits
            .iter()
            .filter_map(|hit| {
                let real_sv = match ref_trajectory {
                    Some(to) => trajectory.relative_state_vector(hit.time, to),
                    None => trajectory.state_vector(hit.time),
                }?;
                let position = points.evaluate(hit.time)?;
                let vp_position = camera.world_to_viewport(camera_transform, position)?;
                Some((vp_position, position, real_sv, hit.time))
            })
            .collect::<Vec<_>>();
        if window_data.is_empty() {
            continue;
        }
        let window_position = window_data
            .iter()
            .max_by(|a, b| a.0.x.total_cmp(&b.0.x))
            .unwrap()
            .0
            + Vec2::new(20.0, -20.0);

        egui::Window::new(name.as_str())
            .id(egui::Id::new(i.to_string() + "#intersections"))
            .fixed_pos(window_position.to_array())
            .fixed_size([200.0, 200.0])
            .collapsible(false)
            .resizable(false)
            .fade_in(false)
            .fade_out(false)
            .title_bar(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.style_mut().interaction.selectable_labels = false;
                        ui.label(name.as_str());

                        if !is_persisted {
                            return;
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if close_button(ui).clicked() {
                                hits.clear();
                            }
                        });
                    });
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .auto_shrink([false, true])
                    .show_rows(
                        ui,
                        ui.text_style_height(&egui::TextStyle::Body),
                        window_data.len(),
                        |ui, range| {
                            for (j, (_, position, sv, time)) in window_data.iter().enumerate() {
                                let time = time.round(Duration::from_seconds(1.0));
                                let hovered = range
                                    .contains(&j)
                                    .then(|| {
                                        ui.scope(|ui| {
                                            let is_current = time == sim_time.current();
                                            let mut text = egui::RichText::new(time.to_string());
                                            if is_current {
                                                text = text.italics();
                                            }
                                            egui::CollapsingHeader::new(text)
                                                .id_source(format!("{name}#{time}#{j}"))
                                                .show(ui, |ui| {
                                                    ui.label(nformat!(
                                                        "Distance: {:.2} km",
                                                        sv.position.length()
                                                    ));
                                                    ui.horizontal(|ui| {
                                                        if ui.button("Go to").clicked() {
                                                            sim_time.set_current_clamped(time);
                                                        }

                                                        if let Some(flight_plan) =
                                                            flight_plan.as_mut()
                                                        {
                                                            if ui.button("Add manoeuvre").clicked()
                                                            {
                                                                flight_plan.burns.push(Burn::new(
                                                                    time,
                                                                    plot.reference.unwrap_or(root),
                                                                ));
                                                                commands.trigger_targets(
                                                                    FlightPlanChanged,
                                                                    entity,
                                                                );
                                                            }
                                                        }
                                                    })
                                                });
                                        })
                                    })
                                    .map_or(false, |r| r.response.contains_pointer());

                                let direction = camera_transform.translation() - *position;
                                let size = direction.length() * fov * PICK_THRESHOLD;
                                gizmos.circle(
                                    *position,
                                    Dir3::new(direction).unwrap(),
                                    size,
                                    plot.color.with_alpha((hovered as usize as f32 + 1.0) / 2.0),
                                );
                            }
                        },
                    );
            });
    }
}

fn close_button(ui: &mut egui::Ui) -> egui::Response {
    let s_rect = ui.max_rect();
    let size = egui::Vec2::splat(ui.spacing().icon_width);
    let pad = (s_rect.height() - size.y) / 2.0;
    let min = egui::pos2(
        s_rect.right() - pad - size.x,
        s_rect.center().y - 0.5 * size.y,
    );
    let rect = egui::Rect::from_min_size(min, size);

    let close_id = ui.auto_id_with("window_close_button");
    let response = ui.interact(rect, close_id, egui::Sense::click());
    ui.expand_to_include_rect(response.rect);

    let visuals = ui.style().interact(&response);
    let rect = rect.shrink(2.0).expand(visuals.expansion);
    let stroke = visuals.fg_stroke;
    ui.painter() // paints \
        .line_segment([rect.left_top(), rect.right_bottom()], stroke);
    ui.painter() // paints /
        .line_segment([rect.right_top(), rect.left_bottom()], stroke);
    response
}
