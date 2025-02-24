use crate::{
    settings::{AppSettings, UserSettings},
    ui::WindowsUiSet,
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

pub struct SettingsPlugin;

impl Plugin for SettingsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            SettingsWindow::show
                .never_param_warn()
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Resource)]
pub struct SettingsWindow {
    last: UserSettings,
}

impl FromWorld for SettingsWindow {
    fn from_world(world: &mut World) -> Self {
        Self {
            last: world.get_resource_or_init::<AppSettings>().user,
        }
    }
}

impl SettingsWindow {
    fn show(
        mut commands: Commands,
        mut contexts: EguiContexts,
        window: Res<Self>,
        mut settings: ResMut<AppSettings>,
    ) {
        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut open = true;
        let mut should_close = false;
        egui::Window::new("Settings")
            .open(&mut open)
            .show(ctx, |ui| {
                if ui
                    .scope(|ui| {
                        let settings = settings.bypass_change_detection();
                        ui.heading("Graphics settings")
                            | ui.separator()
                            | ui.checkbox(&mut settings.user.fullscreen, "Fullscreen")
                            | ui.add(
                                egui::Slider::new(&mut settings.user.bloom_intensity, 0.0..=0.5)
                                    .text("Bloom intensity"),
                            )
                            | ui.add(
                                egui::Slider::new(&mut settings.user.fov, 0.05..=120.0)
                                    .logarithmic(true)
                                    .text("Field of view"),
                            )
                            | ui.add(
                                egui::Slider::new(&mut settings.user.line_width, 0.1..=4.0)
                                    .logarithmic(true)
                                    .text("Line width"),
                            )
                            | ui.checkbox(&mut settings.user.show_labels, "Show labels")
                            | ui.heading("Input settings")
                            | ui.separator()
                            | ui.add(
                                egui::Slider::new(&mut settings.user.mouse_sensitivity, 0.1..=5.0)
                                    .logarithmic(true)
                                    .text("Mouse sensitivity"),
                            )
                    })
                    .inner
                    .changed()
                {
                    settings.set_changed();
                }

                ui.horizontal(|ui| {
                    if ui.button("Discard").clicked() {
                        settings.user = window.last;
                        should_close = false;
                    }
                })
            });

        if !open || should_close {
            commands.remove_resource::<Self>();
        }
    }
}
