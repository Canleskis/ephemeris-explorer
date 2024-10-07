use crate::{
    ephemerides::{
        ComputeEphemeridesEvent, EphemerisBuilder,
        PredictionTracking, Trajectory,
    },
    plot::{EphemerisPlot, EphemerisPlotConfig},
    hierarchy,
    load::LoadSolarSystemEvent,
    selection::{Followed, Selected},
    time::EphemeridesTime,
    GameState, SystemRoot,
};

use std::str::FromStr;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::{Duration, Epoch};
use thousands::Separable;

#[derive(Component, Default)]
pub struct Labelled {
    pub style: TextStyle,
    pub offset: Vec2,
    pub index: usize,
}

#[derive(Debug, Component, Deref, DerefMut)]
pub struct OrbitalPeriod(pub Option<Duration>);

pub fn _not_used_by_ui(
    query: Query<&bevy_egui::EguiContext, With<bevy::window::PrimaryWindow>>,
) -> bool {
    !query.single().get().is_using_pointer()
}

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(
            FileDialogPlugin::new()
                .with_pick_directory::<SolarSystemDir>()
                .with_save_file::<ExportFile>(),
        )
        .add_event::<ExportSolarSystemEvent>()
        .add_systems(First, (spawn_labels, despawn_labels).chain())
        .add_systems(
            Update,
            (
                load_solar_system_state,
                (export_solar_system, exported_solar_system).chain(),
            ),
        )
        .add_systems(
            Update,
            (
                top_menu,
                time_controls,
                solar_system_explorer,
                (
                    ExportSettings::window,
                    PredictionPlanner::window,
                    EphemeridesDebug::window,
                    body_info_window,
                    limit_to_period,
                ),
            )
                .chain()
                .run_if(in_state(GameState::Running)),
        )
        .add_systems(
            Update,
            (
                (update_labels_position, hide_overlapped_labels).chain(),
                update_labels_color,
            )
                .after(bevy::transform::TransformSystem::TransformPropagate),
        );
    }
}

#[derive(Component, Deref, DerefMut)]
struct LabelEntity(Entity);

#[derive(Component)]
struct Label;

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
    query_camera: Query<(&Camera, &GlobalTransform)>,
    query_labelled: Query<(&LabelEntity, &Labelled, &GlobalTransform)>,
    mut query_labels: Query<(&mut Style, &Node)>,
) {
    let (camera, camera_transform) = query_camera.single();

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
    query_camera: Query<&GlobalTransform, With<Camera>>,
    query_labels: Query<(&GlobalTransform, &Node), With<Label>>,
    query_labelled: Query<(&GlobalTransform, &Labelled, &LabelEntity)>,
    mut query_visibility: Query<&mut Visibility, With<Label>>,
    kb: Res<ButtonInput<KeyCode>>,
    mut hide_all: Local<bool>,
) {
    let camera_transform = query_camera.single();

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

#[derive(PartialEq, Eq, Clone, Copy, Default)]
enum TimeScale {
    #[default]
    TimePerSecond,
    Multiplier,
}

impl std::fmt::Display for TimeScale {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            TimeScale::TimePerSecond => write!(f, "Time per second"),
            TimeScale::Multiplier => write!(f, "Multiplier"),
        }
    }
}

impl TimeScale {
    fn values() -> [Self; 2] {
        [TimeScale::TimePerSecond, TimeScale::Multiplier]
    }
}

fn round(x: f64, decimals: usize) -> f64 {
    let factor = 10.0_f64.powi(decimals as i32);
    (x * factor).round() / factor
}

struct Sample {
    time: std::time::Instant,
    value: f64,
}

#[derive(Default)]
struct History {
    values: std::collections::VecDeque<Sample>,
}

impl History {
    fn add(&mut self, value: f64) {
        let now = std::time::Instant::now();

        self.values.retain(|sample| {
            now.duration_since(sample.time) < std::time::Duration::from_millis(200)
        });
        self.values.push_back(Sample { time: now, value });
    }

    fn average(&self) -> f64 {
        if self.values.is_empty() {
            return 0.0;
        }

        self.values.iter().map(|v| v.value).sum::<f64>() / self.values.len() as f64
    }
}

fn time_controls(
    mut contexts: EguiContexts,
    diagnostics: Res<bevy::diagnostic::DiagnosticsStore>,
    mut eph_time: ResMut<EphemeridesTime>,
    mut buffer: Local<String>,
    mut scale: Local<TimeScale>,
    mut time_scale_history: Local<History>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    time_scale_history.add(eph_time.real_time_scale());

    egui::TopBottomPanel::bottom("Time").show(ctx, |ui| {
        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
            ui.spacing_mut().text_edit_width = 215.0;
            let edit = ui.text_edit_singleline(&mut *buffer);

            if edit.lost_focus() {
                if let Ok(buffer) = Epoch::from_str(&buffer) {
                    eph_time.set_epoch_clamped(buffer);
                }
            }

            if !edit.has_focus() {
                *buffer = format!("{:x}", eph_time.current());
            }

            let button = egui::Button::new(if eph_time.paused { "▶" } else { "⏸" });
            if ui.add_sized([40.0, 18.0], button).clicked() {
                eph_time.paused = !eph_time.paused;
            }

            ui.scope(|ui| {
                let computed_time_scale = time_scale_history.average();

                ui.spacing_mut().slider_width = ui.available_width() / 3.0;
                ui.spacing_mut().interact_size.x = 175.0;
                if round(computed_time_scale, 2) != eph_time.time_scale {
                    ui.visuals_mut().override_text_color = Some(egui::Color32::RED);
                }

                let slider = egui::Slider::new(&mut eph_time.time_scale, -1e10..=1e10)
                    .logarithmic(true)
                    .fixed_decimals(2)
                    .smallest_positive(0.01)
                    .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.6 });

                let slider = match *scale {
                    TimeScale::TimePerSecond => slider
                        .suffix(" per second")
                        .custom_formatter(|_, _| {
                            format!(
                                "{}{}",
                                if computed_time_scale < 0.0 { "-" } else { "" },
                                (Duration::from_seconds(computed_time_scale).abs().approx())
                            )
                        })
                        .custom_parser(|text| Some(Duration::from_str(text).ok()?.to_seconds())),
                    TimeScale::Multiplier => slider.prefix("x").custom_formatter(|_, _| {
                        format!("{:.2}", computed_time_scale).separate_with_commas()
                    }),
                };

                ui.add(slider);
            });

            egui::ComboBox::from_id_source("time_scale")
                .selected_text(scale.to_string())
                .show_ui(ui, |ui| {
                    for value in TimeScale::values() {
                        ui.selectable_value(&mut *scale, value, value.to_string());
                    }
                });

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if let Some(fps_smoothed) = diagnostics
                    .get(&bevy::diagnostic::FrameTimeDiagnosticsPlugin::FPS)
                    .and_then(|fps| fps.smoothed())
                {
                    ui.add(egui::Label::new(format!("FPS: {:.0}", fps_smoothed)).selectable(false));
                }
            })
        });
    });
}

#[allow(unused)]
fn limit_to_period(
    config: Res<EphemerisPlotConfig>,
    mut query: Query<(&OrbitalPeriod, &mut EphemerisPlot)>,
) {
    for (period, mut plot) in &mut query {
        // plot.end = config.current_time + period.0.unwrap_or_default();
    }
}

fn show_tree_query(
    root: Entity,
    query: &Query<(Entity, &Name, Option<&hierarchy::Children>)>,
    default_open: impl Fn(usize) -> bool + Copy,
    ui: &mut egui::Ui,
    mut add_header: impl FnMut(
        &mut egui::Ui,
        &mut egui::collapsing_header::CollapsingState,
        (Entity, &Name, Option<&hierarchy::Children>),
        usize,
    ),
) {
    fn show_tree_inner(
        parent: Entity,
        query: &Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        ui: &mut egui::Ui,
        depth: usize,
        default_open: impl Fn(usize) -> bool + Copy,
        add_contents: &mut impl FnMut(
            &mut egui::Ui,
            &mut egui::collapsing_header::CollapsingState,
            (Entity, &Name, Option<&hierarchy::Children>),
            usize,
        ),
    ) {
        if let Ok((parent, name, children)) = query.get(parent) {
            let id = name;
            let text = egui::WidgetText::from(format!("{}{:?}", id, ui.id()));
            let id = egui::Id::new(text.text());

            let has_children = children.is_some_and(|c| !c.is_empty());

            let mut state = egui::collapsing_header::CollapsingState::load_with_default_open(
                ui.ctx(),
                id,
                has_children && default_open(depth),
            );
            if !has_children {
                state.set_open(false);
            }

            let header_res = ui.horizontal(|ui| {
                add_contents(ui, &mut state, (parent, name, children), depth);
            });

            state.show_body_indented(&header_res.response, ui, |ui| {
                for child in children.into_iter().flatten() {
                    show_tree_inner(*child, query, ui, depth + 1, default_open, add_contents);
                }
            });
        }
    }

    show_tree_inner(root, query, ui, 0, default_open, &mut add_header);
}

struct EphemerisInfo<T> {
    text: egui::WidgetText,
    hover_text: Option<egui::WidgetText>,
    info: Option<T>,
}

impl<T> EphemerisInfo<T> {
    fn new(text: impl Into<egui::WidgetText>, info: Option<T>) -> Self {
        Self {
            text: text.into(),
            hover_text: None,
            info,
        }
    }

    fn hover_text(mut self, hover_text: impl Into<egui::WidgetText>) -> Self {
        self.hover_text = Some(hover_text.into());
        self
    }

    fn show(&self, ui: &mut egui::Ui, content: impl FnOnce(&mut egui::Ui, &T)) {
        ui.vertical(|ui| {
            let label = ui.label(self.text.clone());
            if let Some(hover_text) = &self.hover_text {
                label.on_hover_text(hover_text.clone());
            }
            ui.indent(self.text.text(), |ui| {
                if let Some(info) = &self.info {
                    content(ui, info);
                } else {
                    ui.label(format!("No {} data", self.text.text().to_lowercase()));
                }
            });
        });
    }
}

fn body_info_window(
    mut contexts: EguiContexts,
    mut selected: ResMut<Selected>,
    mut plot_query: Query<&mut EphemerisPlot>,
    eph_time: Res<EphemeridesTime>,
    query_trajectory: Query<&Trajectory>,
    query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
    root: Query<Entity, With<SystemRoot>>,
) {
    let mut open = selected.is_some();

    if let Some(selected) = **selected {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let root = root.single();

        let Ok(trajectory) = query_trajectory.get(selected) else {
            return;
        };
        let Ok(mut plot) = plot_query.get_mut(selected) else {
            return;
        };
        let Ok((entity, name, _)) = query_hierarchy.get(selected) else {
            return;
        };

        egui::Window::new(name.as_str())
            .id("Selected".into())
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                let reference_name = plot
                    .reference
                    .and_then(|r| query_hierarchy.get(r).ok())
                    .map(|(_, name, _)| name.as_str())
                    .unwrap_or("None");
                egui::Grid::new("Ephemeris")
                    .min_col_width(150.0)
                    .show(ui, |ui| {
                        let sv = trajectory
                            .evaluate_state_vector(eph_time.current())
                            .map(|sv| {
                                sv - plot
                                    .reference
                                    .and_then(|r| query_trajectory.get(r).ok())
                                    .and_then(|t| t.evaluate_state_vector(eph_time.current()))
                                    .unwrap_or_default()
                            });
                        let position = sv.map(|sv| sv.position);
                        let velocity = sv.map(|sv| sv.velocity);
                        let distance = position.map(|pos| pos.length());
                        let speed = velocity.map(|vel| vel.length());

                        EphemerisInfo::new("Position", position)
                            .hover_text(format!("Position relative to {}", reference_name))
                            .show(ui, |ui, position| {
                                ui.label(format!("x: {:.2} km", position.x));
                                ui.label(format!("y: {:.2} km", position.y));
                                ui.label(format!("z: {:.2} km", position.z));
                            });
                        EphemerisInfo::new("Velocity", velocity)
                            .hover_text(format!("Velocity relative to {}", reference_name))
                            .show(ui, |ui, velocity| {
                                ui.label(format!("x: {:.2} km/s", velocity.x));
                                ui.label(format!("y: {:.2} km/s", velocity.y));
                                ui.label(format!("z: {:.2} km/s", velocity.z));
                            });

                        ui.end_row();

                        EphemerisInfo::new("Distance", distance)
                            .hover_text(format!("Distance relative to {}", reference_name))
                            .show(ui, |ui, distance| {
                                ui.label(format!("{:.2} km", distance));
                            });
                        EphemerisInfo::new("Speed", speed)
                            .hover_text(format!("Speed relative to {}", reference_name))
                            .show(ui, |ui, speed| {
                                ui.label(format!("{:.2} km/s", speed));
                            });
                    });

                ui.add_space(10.0);

                ui.horizontal(|ui| {
                    ui.label("Reference:");
                    egui::ComboBox::from_id_source("Reference")
                        .wrap_mode(egui::TextWrapMode::Extend)
                        .selected_text(reference_name)
                        .show_ui(ui, |ui| {
                            show_tree_query(
                                root,
                                &query_hierarchy,
                                |_| true,
                                ui,
                                |ui, _, (ref_entity, ref_name, _), _| {
                                    ui.horizontal(|ui| {
                                        ui.add_enabled_ui(entity != ref_entity, |ui| {
                                            ui.selectable_value(
                                                &mut plot.reference,
                                                Some(ref_entity),
                                                ref_name.as_str(),
                                            )
                                            .on_disabled_hover_text(
                                                "Cannot use itself as a reference",
                                            );
                                        });
                                        ui.add_space(10.0);
                                    });
                                },
                            );
                        });
                });

                ui.horizontal(|ui| {
                    ui.label("Resolution:");
                    ui.add(egui::Slider::new(&mut plot.threshold, 0.1..=10.0).logarithmic(true));
                });
            });
    }

    if selected.is_some() && !open {
        **selected = None;
    }
}

fn solar_system_explorer(
    mut contexts: EguiContexts,
    mut followed: ResMut<Followed>,
    mut selected: ResMut<Selected>,
    mut query: Query<&mut EphemerisPlot>,
    query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
    root: Query<Entity, With<SystemRoot>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    let root = root.single();

    egui::SidePanel::left("Explorer")
        .resizable(true)
        .show(ctx, |ui| {
            show_tree_query(
                root,
                &query_hierarchy,
                |_| true,
                ui,
                |ui, state, (entity, name, children), _| {
                    let has_children = children.is_some_and(|c| !c.is_empty());

                    ui.spacing_mut().item_spacing = [1.0, 3.0].into();
                    ui.add_visible_ui(has_children, |ui| {
                        state.show_toggle_button(ui, egui::collapsing_header::paint_default_icon);
                    });

                    // Camera follow toggle
                    ui.scope(|ui| {
                        let is_followed = followed.0 == Some(entity);
                        if ui.selectable_label(is_followed, "📌").clicked() {
                            followed.replace(entity);
                        }
                    });
                    if let Ok(mut plot) = query.get_mut(entity) {
                        // Plot toggle
                        ui.scope(|ui| {
                            let [r, g, b, _a] = plot.color.to_srgba().to_u8_array();
                            ui.visuals_mut().selection.bg_fill = egui::Color32::from_gray(80);
                            ui.visuals_mut().selection.stroke =
                                egui::Stroke::new(1.0, egui::Color32::from_rgb(r, g, b));

                            if ui.selectable_label(plot.enabled, "○").clicked() {
                                plot.enabled = !plot.enabled;
                            }
                        });

                        // Children plot toggle
                        ui.add_visible_ui(has_children, |ui| {
                            ui.visuals_mut().selection.bg_fill = egui::Color32::from_gray(80);

                            let plotted_count = children
                                .into_iter()
                                .flatten()
                                .filter_map(|entity| query.get(*entity).ok())
                                .filter(|p| p.enabled)
                                .count();

                            if children.is_some_and(|c| c.len() != plotted_count) {
                                let color = &mut ui.visuals_mut().selection.bg_fill;
                                *color = color.linear_multiply(0.4);
                            }

                            let any_plotted = plotted_count > 0;
                            if ui.selectable_label(any_plotted, "A").clicked() {
                                for child in children.into_iter().flatten() {
                                    if let Ok(mut plot) = query.get_mut(*child) {
                                        plot.enabled = !any_plotted;
                                    }
                                }
                            }
                        });

                        let checked = **selected == Some(entity);
                        if ui.selectable_label(checked, name.as_str()).clicked() {
                            **selected = Some(entity);
                        }
                    } else {
                        ui.label(name.as_str());
                    }
                },
            );
        });
}

#[expect(clippy::type_complexity)]
struct ParsedTextEdit<'t, T> {
    text: &'t mut String,
    parsed: &'t mut T,
    parse: Box<dyn 't + Fn(&str) -> Option<T>>,
    format: Box<dyn 't + Fn(&T) -> String>,
}

impl<'t, T> ParsedTextEdit<'t, T> {
    pub fn singleline(
        text: &'t mut String,
        parsed: &'t mut T,
        parse: impl 't + Fn(&str) -> Option<T>,
        format: impl 't + Fn(&T) -> String,
    ) -> Self {
        Self {
            text,
            parsed,
            parse: Box::new(parse),
            format: Box::new(format),
        }
    }
}

impl egui::Widget for ParsedTextEdit<'_, Epoch> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let edit = ui.text_edit_singleline(self.text);

        if let Some(val) = (self.parse)(self.text) {
            *self.parsed = val;
        } else if !edit.has_focus() {
            *self.text = (self.format)(self.parsed);
        }

        edit
    }
}

#[derive(Default, Resource)]
struct ExportSettings;

impl ExportSettings {
    #[expect(clippy::too_many_arguments)]
    fn window(
        mut contexts: EguiContexts,
        mut commands: Commands,
        mut event: EventWriter<ExportSolarSystemEvent>,
        window: Option<Res<Self>>,
        eph_time: Res<EphemeridesTime>,
        query_trajectory: Query<Entity, With<Trajectory>>,
        query_hierarchy: Query<(Entity, &Name, Option<&hierarchy::Children>)>,
        root: Query<Entity, With<SystemRoot>>,
        mut bodies: Local<Option<bevy::utils::EntityHashSet<Entity>>>,
        mut cached_exports: Local<Option<[ExportType; 2]>>,
        mut current: Local<usize>,
        mut epoch_buffer: Local<String>,
        mut start_buffer: Local<String>,
        mut end_buffer: Local<String>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let root = root.single();

        let mut open = window.is_some();
        egui::Window::new("Export").open(&mut open).show(ctx, |ui| {
            let export = cached_exports.get_or_insert_with(|| {
                [
                    ExportType::State {
                        epoch: eph_time.current(),
                    },
                    ExportType::Trajectories {
                        start: eph_time.start(),
                        end: eph_time.end(),
                    },
                ]
            });
            let bodies = bodies.get_or_insert_with(|| query_trajectory.iter().collect());

            if egui::ComboBox::from_id_source("export type")
                .show_index(ui, &mut current, export.len(), |i| export[i].to_string())
                .changed()
            {
                event.send(ExportSolarSystemEvent {
                    export: export[*current],
                    bodies: bodies.iter().copied().collect(),
                });
            };

            ui.add_space(5.0);

            ui.spacing_mut().text_edit_width = 300.0;
            let parse = |min, max| {
                move |buf: &str| Epoch::from_str(buf).ok().filter(|t| *t >= min && *t <= max)
            };
            let format_tai = |t: &Epoch| format!("{:x}", t);
            match &mut export[*current] {
                ExportType::State { epoch } => {
                    ui.horizontal(|ui| {
                        ui.label("Epoch:");
                        ui.add(ParsedTextEdit::singleline(
                            &mut epoch_buffer,
                            epoch,
                            parse(eph_time.start(), eph_time.end()),
                            format_tai,
                        ));
                    });
                }
                ExportType::Trajectories { start, end } => {
                    ui.horizontal(|ui| {
                        ui.label("Start time:");
                        ui.add(ParsedTextEdit::singleline(
                            &mut start_buffer,
                            start,
                            parse(eph_time.start(), *end),
                            format_tai,
                        ));
                    });
                    ui.horizontal(|ui| {
                        ui.label("End time:  ");
                        ui.add(ParsedTextEdit::singleline(
                            &mut end_buffer,
                            end,
                            parse(*start, eph_time.end()),
                            format_tai,
                        ))
                    });
                }
            }

            egui::ScrollArea::vertical().show(ui, |ui| {
                show_tree_query(
                    root,
                    &query_hierarchy,
                    |i| i != 0,
                    ui,
                    |ui, state, (entity, name, children), depth| {
                        let has_children = children.is_some_and(|c| !c.is_empty());
                        let has_trajectory = query_trajectory.get(entity).is_ok();
                        if has_trajectory {
                            ui.add_visible_ui(has_children, |ui| {
                                let selected_count = children
                                    .into_iter()
                                    .flatten()
                                    .filter_map(|entity| bodies.get(entity))
                                    .count();

                                if children.is_some_and(|c| c.len() != selected_count) {
                                    let color = &mut ui.visuals_mut().selection.bg_fill;
                                    *color = color.linear_multiply(0.4);
                                }

                                let any_selected = selected_count > 0;
                                if ui.selectable_label(any_selected, "A").clicked() {
                                    for entity in children.into_iter().flatten() {
                                        match any_selected {
                                            true => bodies.remove(entity),
                                            false => bodies.insert(*entity),
                                        };
                                    }
                                }
                            });
                        }

                        if has_trajectory {
                            let checked = bodies.contains(&entity);
                            if ui.selectable_label(checked, name.as_str()).clicked() {
                                match bodies.entry(entity) {
                                    bevy::utils::hashbrown::hash_set::Entry::Occupied(entry) => {
                                        entry.remove();
                                    }
                                    bevy::utils::hashbrown::hash_set::Entry::Vacant(entry) => {
                                        entry.insert();
                                    }
                                }
                            }
                        } else {
                            ui.label(name.as_str());
                        }

                        if depth == 0 {
                            state.show_toggle_button(ui, reversed_paint_default_icon);
                        }

                        ui.add_space(10.0);
                    },
                );
            });
        });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}

pub fn reversed_paint_default_icon(ui: &mut egui::Ui, openness: f32, response: &egui::Response) {
    let visuals = ui.style().interact(response);

    let rect = response.rect;

    // Draw a pointy triangle arrow:
    let rect = egui::Rect::from_center_size(
        rect.center(),
        egui::vec2(rect.width(), rect.height()) * 0.75,
    );
    let rect = rect.expand(visuals.expansion);
    let mut points = vec![rect.left_top(), rect.right_top(), rect.center_bottom()];
    use std::f32::consts::TAU;
    let rotation = egui::emath::Rot2::from_angle(egui::remap(openness, 0.0..=1.0, TAU / 4.0..=0.0));
    for p in &mut points {
        *p = rect.center() + rotation * (*p - rect.center());
    }

    ui.painter().add(egui::Shape::convex_polygon(
        points,
        visuals.fg_stroke.color,
        egui::Stroke::NONE,
    ));
}

#[derive(Default, Resource)]
struct EphemeridesDebug;

impl EphemeridesDebug {
    fn window(
        mut contexts: EguiContexts,
        mut commands: Commands,
        window: Option<Res<Self>>,
        query: Query<(&Name, &Trajectory)>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut open = window.is_some();
        egui::Window::new("Ephemerides debug")
            .open(&mut open)
            .show(ctx, |ui| {
                let queried = query
                    .iter()
                    .map(|(name, data)| (name.as_str(), data.size(), data.len(), data.end()))
                    .collect::<Vec<_>>();

                let available_height = ui.available_height();
                egui_extras::TableBuilder::new(ui)
                    .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                    .column(egui_extras::Column::exact(100.0))
                    .column(egui_extras::Column::exact(80.0))
                    .column(egui_extras::Column::exact(80.0))
                    .column(egui_extras::Column::exact(180.0))
                    .min_scrolled_height(0.0)
                    .max_scroll_height(available_height)
                    .header(20.0, |mut header| {
                        header.col(|ui| {
                            ui.strong("Body");
                        });
                        header.col(|ui| {
                            ui.strong("Size (MB)");
                        });
                        header.col(|ui| {
                            ui.strong("Segments");
                        });
                        header.col(|ui| {
                            ui.strong("End");
                        });
                    })
                    .body(|body| {
                        body.rows(16.0, queried.len() + 1, |mut row| {
                            let (name, size, count, end) = match row.index() {
                                0 => (
                                    "Total",
                                    queried.iter().map(|(_, size, ..)| size).sum(),
                                    queried.iter().map(|(.., count, _)| count).sum(),
                                    queried
                                        .iter()
                                        .map(|(.., end)| end)
                                        .min()
                                        .copied()
                                        .unwrap_or_default(),
                                ),
                                i => queried[i - 1],
                            };
                            row.col(|ui| {
                                ui.label(name.to_string());
                            });
                            row.col(|ui| {
                                ui.label(
                                    format!("{:.2}", (size as f32 / (1024.0 * 1024.0)))
                                        .separate_with_commas(),
                                );
                            });
                            row.col(|ui| {
                                ui.label(count.to_string());
                            });
                            row.col(|ui| {
                                ui.label(format!("{:x}", end));
                            });
                        })
                    });
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}

#[derive(Default, Resource)]
struct PredictionPlanner;

impl PredictionPlanner {
    #[expect(clippy::too_many_arguments)]
    fn window(
        mut contexts: EguiContexts,
        mut commands: Commands,
        window: Option<Res<Self>>,
        eph_time: Res<EphemeridesTime>,
        mut end_buffer: Local<String>,
        mut event_writer: EventWriter<ComputeEphemeridesEvent>,
        prediction: Query<(Entity, Option<Ref<PredictionTracking>>), With<SystemRoot>>,
        time: Res<Time>,
        mut time_taken: Local<Option<std::time::Duration>>,
    ) {
        // One system for now (maybe ever).
        let (root, prediction) = prediction.single();

        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut open = window.is_some();
        egui::Window::new("Prediction planner")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.spacing_mut().text_edit_width = 300.0;
                ui.horizontal(|ui| {
                    ui.label("Start time:");
                    ui.add_enabled_ui(false, |ui| {
                        ui.text_edit_singleline(&mut format!("{:x}", eph_time.start()));
                    })
                });

                let edit = ui.horizontal(|ui| {
                    ui.label("End time:  ");
                    ui.add_enabled_ui(prediction.is_none(), |ui| {
                        ui.text_edit_singleline(&mut *end_buffer)
                    })
                    .inner
                });

                let (duration, label) = match Epoch::from_str(&end_buffer) {
                    Ok(end) if prediction.is_none() && eph_time.end() < end => {
                        let duration = end - eph_time.end();
                        let label = format!("Planned prediction length: {}", duration.approx());

                        (Some(duration), label)
                    }
                    _ => {
                        if !edit.inner.has_focus() {
                            *end_buffer = format!("{:x}", eph_time.end());
                        }
                        let label = if prediction.is_some() {
                            String::from("Prediction in progress")
                        } else {
                            String::from("No planned prediction")
                        };

                        (None, label)
                    }
                };

                if duration.is_some() || prediction.as_ref().is_some_and(Ref::is_added) {
                    *time_taken = None;
                }

                if let Some(prediction) = &prediction {
                    let time_taken = time_taken.get_or_insert_with(Default::default);
                    if prediction.in_progress() {
                        *time_taken += time.delta()
                    }
                }

                ui.horizontal(|ui| {
                    ui.label(label);
                    if let Some(time_taken) = &*time_taken {
                        ui.separator();
                        ui.label(format!("Time taken: {:?}", time_taken));
                    }
                });

                ui.horizontal(|ui| match prediction {
                    None => {
                        ui.add_enabled_ui(duration.is_some(), |ui| {
                            if ui.button("Start prediction").clicked() {
                                if let Some(duration) = duration {
                                    let sync_count = (duration.total_nanoseconds()
                                        / Duration::from_days(25.0).total_nanoseconds())
                                        as usize;
                                    event_writer.send(ComputeEphemeridesEvent {
                                        root,
                                        duration,
                                        sync_count,
                                    });
                                }
                            }
                        });
                    }
                    Some(prediction) => {
                        let paused = prediction.is_paused();
                        if ui.button(if paused { "Resume" } else { "Pause" }).clicked() {
                            prediction.pause_or_resume();
                        }
                        if ui.button("Cancel").clicked() {
                            commands.entity(root).remove::<PredictionTracking>();
                        }
                        ui.add(egui::ProgressBar::new(prediction.progress()).show_percentage());
                    }
                });
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}

fn top_menu(
    mut contexts: EguiContexts,
    mut commands: Commands,
    export: Option<Res<ExportSettings>>,
    planner: Option<Res<PredictionPlanner>>,
    ephemerides: Option<Res<EphemeridesDebug>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::TopBottomPanel::top("Menu").show(ctx, |ui| {
        ui.horizontal(|ui| {
            if ui.selectable_label(false, "Load").clicked() {
                commands.dialog().pick_directory_path::<SolarSystemDir>();
            }
            menu_label(export, ui, &mut commands, "Export");
            menu_label(planner, ui, &mut commands, "Prediction planner");
            menu_label(ephemerides, ui, &mut commands, "Debug ephemerides");
        });
    });
}

fn menu_label<T>(menu: Option<Res<T>>, ui: &mut egui::Ui, commands: &mut Commands, label: &str)
where
    T: Resource + Default,
{
    if ui.selectable_label(menu.is_some(), label).clicked() {
        match menu.is_some() {
            true => commands.remove_resource::<T>(),
            false => commands.insert_resource(T::default()),
        }
    }
}

pub struct SolarSystemDir;

fn load_solar_system_state(
    asset_server: Res<AssetServer>,
    mut ev_picked: EventReader<DialogDirectoryPicked<SolarSystemDir>>,
    mut events: EventWriter<LoadSolarSystemEvent>,
) {
    for picked in ev_picked.read() {
        match LoadSolarSystemEvent::try_from_dir(&picked.path, &asset_server) {
            Ok(event) => {
                events.send(event);
            }
            Err(err) => {
                bevy::log::error!(
                    "Failed to load solar system at {}: {}",
                    picked.path.display(),
                    err
                );
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum ExportType {
    State {
        epoch: hifitime::Epoch,
    },
    Trajectories {
        start: hifitime::Epoch,
        end: hifitime::Epoch,
    },
}

impl std::fmt::Display for ExportType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportType::State { .. } => {
                write!(f, "Export state")
            }
            ExportType::Trajectories { .. } => {
                write!(f, "Export trajectories")
            }
        }
    }
}

#[derive(Event, PartialEq, PartialOrd)]
pub struct ExportSolarSystemEvent {
    pub export: ExportType,
    pub bodies: Vec<Entity>,
}

struct ExportFile;

fn export_solar_system(
    mut commands: Commands,
    mut event: EventReader<ExportSolarSystemEvent>,
    query: Query<(&Name, &EphemerisBuilder, &Trajectory)>,
) {
    if let Some(event) = event.read().next() {
        let bytes = match event.export {
            ExportType::State { epoch } => {
                #[derive(serde::Serialize)]
                struct BodyJson {
                    name: String,
                    mu: f64,
                    position: bevy::math::DVec3,
                    velocity: bevy::math::DVec3,
                }
                #[derive(serde::Serialize)]
                struct StateJson {
                    epoch: hifitime::Epoch,
                    bodies: Vec<BodyJson>,
                }

                let bodies = query
                    .iter_many(&event.bodies)
                    .map(|(name, builder, trajectory)| {
                        trajectory
                            .evaluate_state_vector(epoch)
                            .map(|state_vector| BodyJson {
                                name: name.to_string(),
                                mu: builder.mu,
                                position: state_vector.position,
                                velocity: state_vector.velocity,
                            })
                    })
                    .collect::<Option<Vec<_>>>();

                match bodies {
                    None => {
                        bevy::log::error!("Something went wrong when exporting bodies");
                        return;
                    }
                    Some(bodies) => {
                        serde_json::to_vec_pretty(&StateJson { epoch, bodies }).unwrap()
                    }
                }
            }
            ExportType::Trajectories { start, end } => {
                let trajectories = query
                    .iter_many(&event.bodies)
                    .map(|(.., trajectory)| trajectory.between(start, end))
                    .collect::<Option<Vec<_>>>();

                match trajectories {
                    None => {
                        bevy::log::error!("Something went wrong when exporting trajectories");
                        return;
                    }
                    Some(trajectories) => serde_json::to_vec(&trajectories).unwrap(),
                }
            }
        };

        commands
            .dialog()
            .add_filter("JSON", &["json"])
            .save_file::<ExportFile>(bytes);
    }
}

fn exported_solar_system(mut ev_saved: EventReader<DialogFileSaved<ExportFile>>) {
    for ev in ev_saved.read() {
        match ev.result {
            Ok(_) => bevy::log::info!("File {} successfully saved", ev.file_name),
            Err(ref err) => bevy::log::error!("Failed to save {}: {}", ev.file_name, err),
        }
    }
}
