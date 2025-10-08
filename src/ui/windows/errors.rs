use crate::{load::LoadingErrors, ui::WindowsUiSet, MainState};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

pub struct ErrorsPlugin;

impl Plugin for ErrorsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            EguiPrimaryContextPass,
            ErrorsWindow::show
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

pub struct ErrorsWindow;

impl ErrorsWindow {
    pub fn show(mut contexts: EguiContexts, mut errors: ResMut<LoadingErrors>) {
        if errors.is_empty() {
            return;
        }

        let Ok(ctx) = contexts.ctx_mut() else {
            return;
        };

        egui::Window::new("Loading Errors")
            .fixed_size([300.0, 600.0])
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for error in errors.0.iter() {
                        ui.code(egui::RichText::new(error.to_string()).color(egui::Color32::RED));
                    }
                });
                ui.separator();
                if ui.button("Clear").clicked() {
                    errors.0.clear();
                }
            });
    }
}
