mod fixed;
mod windows;
mod world;

pub use fixed::*;
pub use windows::*;
pub use world::*;

use crate::{load::LoadSolarSystemEvent, MainState};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContext, EguiContexts};
use bevy_file_dialog::prelude::*;
use hifitime::Epoch;
use std::str::FromStr;

pub fn is_using_pointer(query: Query<&EguiContext, With<bevy::window::PrimaryWindow>>) -> bool {
    query
        .get_single()
        .map(EguiContext::get)
        .is_ok_and(|ctx| ctx.is_pointer_over_area() || ctx.is_using_pointer())
}

pub fn is_using_keyboard(query: Query<&EguiContext, With<bevy::window::PrimaryWindow>>) -> bool {
    query
        .get_single()
        .map(EguiContext::get)
        .is_ok_and(|ctx| ctx.wants_keyboard_input())
}

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(
            FileDialogPlugin::new()
                .with_pick_directory::<SolarSystemDir>()
                .with_load_file::<ShipFile>()
                .with_save_file::<ExportSolarSystemFile>()
                .with_save_file::<ExportShipFile>(),
        )
        .add_plugins((TooltipPlugin, FixedUiPlugin, WindowsUiPlugin))
        .add_systems(Startup, setup_egui)
        .add_systems(
            Update,
            (load_solar_system_state, top_menu.before(FixedUiSet))
                .run_if(in_state(MainState::Running)),
        );
    }
}

fn setup_egui(mut contexts: EguiContexts) {
    if let Some(ctx) = contexts.try_ctx_mut() {
        let font_definitions = {
            // From egui
            let font_data = std::collections::BTreeMap::from([
                (
                    "Hack".to_owned(),
                    egui::FontData::from_static(include_bytes!(
                        "../../assets/fonts/Hack-Regular.ttf"
                    ))
                    .into(),
                ),
                (
                    "Montserrat".to_owned(),
                    egui::FontData::from_static(include_bytes!(
                        "../../assets/fonts/Montserrat-Regular.ttf"
                    ))
                    .into(),
                ),
                (
                    "NotoEmoji-Regular".to_owned(),
                    egui::FontData::from_static(include_bytes!(
                        "../../assets/fonts/NotoEmoji-Regular.ttf"
                    ))
                    .tweak(egui::FontTweak {
                        scale: 0.81, // make it smaller
                        ..Default::default()
                    })
                    .into(),
                ),
                (
                    "emoji-icon-font".to_owned(),
                    egui::FontData::from_static(include_bytes!(
                        "../../assets/fonts/emoji-icon-font.ttf"
                    ))
                    .tweak(egui::FontTweak {
                        scale: 0.90, // make it smaller
                        // probably not correct, but this does make texts look better (#2724 for details)
                        ..Default::default()
                    })
                    .into(),
                ),
            ]);

            let families = std::collections::BTreeMap::from([
                (
                    egui::FontFamily::Monospace,
                    vec![
                        "Hack".to_owned(),
                        "Montserrat".to_owned(), // fallback for √ etc
                        "NotoEmoji-Regular".to_owned(),
                        "emoji-icon-font".to_owned(),
                    ],
                ),
                (
                    egui::FontFamily::Proportional,
                    vec![
                        "Montserrat".to_owned(),
                        "NotoEmoji-Regular".to_owned(),
                        "emoji-icon-font".to_owned(),
                    ],
                ),
            ]);

            egui::FontDefinitions {
                font_data,
                families,
            }
        };

        ctx.set_fonts(font_definitions);

        ctx.style_mut(|style| {
            use egui::{Color32, CornerRadius, Shadow, Stroke};
            style.visuals.selection.bg_fill = Color32::from_rgb(0, 65, 165);
            style.visuals.window_fill = Color32::from_rgba_premultiplied(20, 20, 20, 240);
            style.visuals.window_shadow = Shadow::NONE;
            style.visuals.window_corner_radius = CornerRadius::same(8);
            style.visuals.window_stroke = Stroke::new(1.0, Color32::from_gray(60));
            style.visuals.panel_fill = Color32::from_rgba_premultiplied(20, 20, 20, 240);
            style.visuals.widgets = egui::style::Widgets {
                noninteractive: egui::style::WidgetVisuals {
                    weak_bg_fill: Color32::from_gray(20),
                    bg_fill: Color32::from_gray(20),
                    bg_stroke: Stroke::new(1.5, Color32::from_gray(60)), // separators, indentation lines
                    fg_stroke: Stroke::new(1.0, Color32::from_gray(140)), // normal text color
                    corner_radius: CornerRadius::same(4),
                    expansion: 0.0,
                },
                inactive: egui::style::WidgetVisuals {
                    weak_bg_fill: Color32::from_gray(50), // button background
                    bg_fill: Color32::from_gray(50),      // checkbox background
                    bg_stroke: Default::default(),
                    fg_stroke: Stroke::new(1.0, Color32::from_gray(140)), // button text
                    corner_radius: CornerRadius::same(4),
                    expansion: 0.0,
                },
                hovered: egui::style::WidgetVisuals {
                    weak_bg_fill: Color32::from_gray(60),
                    bg_fill: Color32::from_gray(60),
                    bg_stroke: Stroke::new(1.0, Color32::from_gray(150)), // e.g. hover over window edge or button
                    fg_stroke: Stroke::new(1.5, Color32::from_gray(180)),
                    corner_radius: CornerRadius::same(4),
                    expansion: 1.0,
                },
                active: egui::style::WidgetVisuals {
                    weak_bg_fill: Color32::from_gray(45),
                    bg_fill: Color32::from_gray(45),
                    bg_stroke: Stroke::new(1.0, Color32::WHITE),
                    fg_stroke: Stroke::new(2.0, Color32::WHITE),
                    corner_radius: CornerRadius::same(4),
                    expansion: 1.0,
                },
                open: egui::style::WidgetVisuals {
                    weak_bg_fill: Color32::from_gray(35),
                    bg_fill: Color32::from_gray(20),
                    bg_stroke: Stroke::new(1.0, Color32::from_gray(60)),
                    fg_stroke: Stroke::new(1.0, Color32::from_gray(150)),
                    corner_radius: CornerRadius::same(4),
                    expansion: 0.0,
                },
            }
        });
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

fn top_menu(
    mut contexts: EguiContexts,
    mut commands: Commands,
    export: Option<Res<ExportWindow>>,
    planner: Option<Res<PredictionPlannerWindow>>,
    debug: Option<Res<EphemeridesDebugWindow>>,
    spawner: Option<Res<ShipSpawnerWindow>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::TopBottomPanel::top("Menu")
        .exact_height(23.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.selectable_label(false, "Load").clicked() {
                    commands.dialog().pick_directory_path::<SolarSystemDir>();
                }
                menu_label(export, ui, &mut commands, "Export");
                menu_label(planner, ui, &mut commands, "Prediction planner");
                menu_label(debug, ui, &mut commands, "Ephemerides debug");
                menu_label(spawner, ui, &mut commands, "Spawn ship");
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

#[inline]
fn get_name(entity: Entity, query_name: &Query<&Name>) -> String {
    query_name
        .get(entity)
        .ok()
        .map(|name| name.to_string())
        .unwrap_or_else(|| "Unknown".to_string())
}

fn show_tree<T, I, IdSource>(
    ui: &mut egui::Ui,
    root: T,
    id_fn: impl Fn(&T) -> IdSource + Copy,
    default_open: impl Fn(usize, &T) -> bool + Copy,
    mut add_header: impl FnMut(
        &mut egui::Ui,
        &mut egui::collapsing_header::CollapsingState,
        T,
        usize,
    ) -> Option<I>,
) where
    I: Iterator<Item = T>,
    IdSource: std::hash::Hash,
{
    fn show_tree_inner<T, I, IdSource>(
        ui: &mut egui::Ui,
        parent: T,
        depth: usize,
        id_fn: impl Fn(&T) -> IdSource + Copy,
        default_open: impl Fn(usize, &T) -> bool + Copy,
        add_contents: &mut impl FnMut(
            &mut egui::Ui,
            &mut egui::collapsing_header::CollapsingState,
            T,
            usize,
        ) -> Option<I>,
    ) where
        I: Iterator<Item = T>,
        IdSource: std::hash::Hash,
    {
        let id = ui.make_persistent_id(id_fn(&parent));
        let mut state = egui::collapsing_header::CollapsingState::load_with_default_open(
            ui.ctx(),
            id,
            default_open(depth, &parent),
        );

        let header_res = ui.horizontal(|ui| add_contents(ui, &mut state, parent, depth));
        let children = header_res.inner;

        if children.is_some() {
            state.show_body_indented(&header_res.response, ui, |ui| {
                for child in children.into_iter().flatten() {
                    show_tree_inner(ui, child, depth + 1, id_fn, default_open, add_contents);
                }
            });
        }
    }

    show_tree_inner(ui, root, 0, id_fn, default_open, &mut add_header);
}

struct IdentedInfo<T> {
    text: egui::WidgetText,
    hover_text: Option<egui::WidgetText>,
    info: T,
}

impl<T> IdentedInfo<T> {
    fn new(text: impl Into<egui::WidgetText>, info: T) -> Self {
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

    fn show<R>(
        self,
        ui: &mut egui::Ui,
        content: impl FnOnce(&mut egui::Ui, T) -> R,
    ) -> egui::InnerResponse<Option<R>> {
        ui.vertical(|ui| {
            let label = ui.label(self.text.clone());
            if let Some(hover_text) = &self.hover_text {
                label.on_hover_text(hover_text.clone());
            }
            ui.indent(self.text.text(), |ui| Some(content(ui, self.info)))
        })
        .inner
    }
}

#[inline]
fn epoch_clamped_parser(min: Epoch, max: Epoch) -> impl Fn(&str) -> Option<Epoch> {
    move |buf| Epoch::from_str(buf).ok().filter(|t| *t >= min && *t <= max)
}

#[inline]
fn precision() -> hifitime::Duration {
    hifitime::Duration::from_milliseconds(100.0)
}

macro_rules! nformat {
    ($($arg:tt)*) => {
        thousands::Separable::separate_with_commas(&std::fmt::format(format_args!($($arg)*)))
    }
}
use nformat;
