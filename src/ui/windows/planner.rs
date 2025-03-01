use crate::{
    auto_extend::AutoExtendSettings,
    dynamics::{Backward, Forward, NBodyPropagator},
    load::SystemRoot,
    prediction::{ExtendPredictionEvent, PredictionPropagator, PredictionTracker},
    time::SimulationTime,
    ui::WindowsUiSet,
    MainState,
};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use hifitime::Epoch;
use std::str::FromStr;

pub struct PredictionPlannerPlugin;

impl Plugin for PredictionPlannerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            PredictionPlannerWindow::show
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Default, Resource)]
pub struct PredictionPlannerWindow;

impl PredictionPlannerWindow {
    fn prediction_content<P>(
        ui: &mut egui::Ui,
        commands: &mut Commands,
        root: Entity,
        auto_extend: &mut AutoExtendSettings<P>,
        prediction: Option<Ref<PredictionTracker<P>>>,
        buffer: &mut String,
        parser: impl Fn(&str) -> Option<Epoch>,
        response: &egui::Response,
        default: Epoch,
        time_taken: &mut Option<std::time::Duration>,
        delta: std::time::Duration,
    ) where
        P: PredictionPropagator,
    {
        let (backward_duration, backward_label) = match parser(buffer) {
            Some(start) if prediction.is_none() => {
                let duration = (start - default).abs();
                let label = format!("Planned prediction length: {}", duration.approx());

                (Some(duration), label)
            }
            _ => {
                if !response.has_focus() {
                    *buffer = format!("{default:x}");
                }
                let label = if prediction.is_some() {
                    "Prediction in progress"
                } else {
                    "No planned prediction"
                };

                (None, label.to_string())
            }
        };

        if backward_duration.is_some() || prediction.as_ref().is_some_and(Ref::is_added) {
            *time_taken = None;
        }

        if let Some(prediction) = prediction.as_ref() {
            let time_taken = time_taken.get_or_insert_with(Default::default);
            if prediction.is_in_progress() {
                *time_taken += delta;
            }
        }

        ui.checkbox(&mut auto_extend.enabled, "Auto extend");

        ui.horizontal(|ui| {
            ui.label(backward_label);
            if let Some(time_taken) = &*time_taken {
                ui.separator();
                ui.label(format!("Time taken: {time_taken:?}"));
            }
        });

        ui.horizontal(|ui| match prediction {
            None => {
                ui.add_enabled_ui(backward_duration.is_some(), |ui| {
                    if ui.button("Start prediction").clicked() {
                        if let Some(duration) = backward_duration {
                            commands.trigger(ExtendPredictionEvent::<P>::all(duration, 1000));
                        }
                    }
                });
            }
            Some(prediction) => {
                let paused = prediction.is_paused();
                let button = egui::Button::new(if paused { "Resume" } else { "Pause" })
                    .min_size(egui::vec2(55.0, 0.0));
                if ui.add(button).clicked() {
                    prediction.pause_or_resume();
                }
                let button = egui::Button::new("Cancel").min_size(egui::vec2(55.0, 0.0));
                if ui.add(button).clicked() {
                    commands.entity(root).remove::<PredictionTracker<P>>();
                }
                ui.add(egui::ProgressBar::new(prediction.progress()).show_percentage());
            }
        });
    }

    #[expect(clippy::type_complexity)]
    fn show(
        mut contexts: EguiContexts,
        mut commands: Commands,
        window: Option<Res<Self>>,
        sim_time: Res<SimulationTime>,
        mut auto_extend_forward: ResMut<AutoExtendSettings<NBodyPropagator<Forward>>>,
        mut auto_extend_backward: ResMut<AutoExtendSettings<NBodyPropagator<Backward>>>,
        mut start_buffer: Local<String>,
        mut end_buffer: Local<String>,
        prediction: Query<
            (
                Entity,
                Option<Ref<PredictionTracker<NBodyPropagator<Forward>>>>,
                Option<Ref<PredictionTracker<NBodyPropagator<Backward>>>>,
            ),
            With<SystemRoot>,
        >,
        time: Res<Time>,
        mut forward_time: Local<Option<std::time::Duration>>,
        mut backward_time: Local<Option<std::time::Duration>>,
    ) {
        // One system for now (maybe ever).
        let (root, forward, backward) = prediction.get_single().expect("No root entity found");

        let Some(ctx) = contexts.try_ctx_mut() else {
            return;
        };

        let mut open = window.is_some();
        egui::Window::new("Prediction planner")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.spacing_mut().interact_size.x = 300.0;

                ui.heading("Backward prediction");

                let backward_edit = ui.horizontal(|ui| {
                    ui.label("Start time:");
                    ui.add_enabled_ui(backward.is_none(), |ui| {
                        ui.add(egui::TextEdit::singleline(&mut *start_buffer))
                    })
                    .inner
                });

                Self::prediction_content(
                    ui,
                    &mut commands,
                    root,
                    &mut auto_extend_backward,
                    backward,
                    &mut start_buffer,
                    |buf| Epoch::from_str(buf).ok().filter(|e| e < &sim_time.start()),
                    &backward_edit.inner,
                    sim_time.start(),
                    &mut backward_time,
                    time.delta(),
                );

                ui.separator();
                ui.heading("Forward prediction");

                let forward_edit = ui.horizontal(|ui| {
                    ui.label("End time:  ");
                    ui.add_enabled_ui(forward.is_none(), |ui| {
                        ui.add(egui::TextEdit::singleline(&mut *end_buffer))
                    })
                    .inner
                });

                Self::prediction_content(
                    ui,
                    &mut commands,
                    root,
                    &mut auto_extend_forward,
                    forward,
                    &mut end_buffer,
                    |buf| Epoch::from_str(buf).ok().filter(|e| e > &sim_time.end()),
                    &forward_edit.inner,
                    sim_time.end(),
                    &mut forward_time,
                    time.delta(),
                );
            });

        if window.is_some() && !open {
            commands.remove_resource::<Self>();
        }
    }
}
