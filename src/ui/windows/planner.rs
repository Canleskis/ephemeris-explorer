use crate::{
    MainState,
    auto_extend::AutoExtendSettings,
    dynamics::{Backward, CelestialTrajectory, Forward},
    load::SystemRoot,
    prediction::{
        ComputePrediction, PredictionContext, PredictionTracker, PropagationTarget, Synchronisation,
    },
    simulation::SimulationTime,
    ui::WindowsUiSet,
};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use ftime::Epoch;
use std::str::FromStr;

pub struct PredictionPlannerPlugin;

impl Plugin for PredictionPlannerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            EguiPrimaryContextPass,
            PredictionPlannerWindow::show
                .in_set(WindowsUiSet)
                .run_if(in_state(MainState::Running)),
        );
    }
}

#[derive(Default, Resource)]
pub struct PredictionPlannerWindow;

impl PredictionPlannerWindow {
    fn prediction_content<T>(
        ui: &mut egui::Ui,
        commands: &mut Commands,
        root: Entity,
        auto_extend: &mut AutoExtendSettings<T>,
        prediction: &PredictionContext<T>,
        tracker: Option<Ref<PredictionTracker<T>>>,
        buffer: &mut String,
        parser: impl Fn(&str) -> Option<Epoch>,
        response: &egui::Response,
        default: Epoch,
        time_taken: &mut Option<std::time::Duration>,
        delta: std::time::Duration,
    ) where
        T: PropagationTarget,
    {
        let (backward_duration, backward_label) = match parser(buffer) {
            Some(start) if tracker.is_none() => {
                let duration = (start - default).abs();
                let label = format!("Planned prediction length: {}", duration);

                (Some(duration), label)
            }
            _ => {
                if !response.has_focus() {
                    *buffer = format!("{default}");
                }
                let label = if tracker.is_some() {
                    "Prediction in progress"
                } else {
                    "No planned prediction"
                };

                (None, label.to_string())
            }
        };

        if backward_duration.is_some() || tracker.as_ref().is_some_and(Ref::is_added) {
            *time_taken = None;
        }

        if let Some(prediction) = tracker.as_ref() {
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

        ui.horizontal(|ui| match tracker {
            None => {
                ui.add_enabled_ui(backward_duration.is_some(), |ui| {
                    if ui.button("Start prediction").clicked()
                        && let Some(duration) = backward_duration
                    {
                        commands.trigger_targets(
                            ComputePrediction::<T>::extend(
                                prediction.propagator.clone(),
                                duration,
                                Synchronisation::hertz(100),
                            ),
                            root,
                        );
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
                    commands.entity(root).remove::<PredictionTracker<T>>();
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
        mut auto_extend_forward: ResMut<AutoExtendSettings<CelestialTrajectory<Forward>>>,
        mut auto_extend_backward: ResMut<AutoExtendSettings<CelestialTrajectory<Backward>>>,
        mut start_buffer: Local<String>,
        mut end_buffer: Local<String>,
        prediction: Query<
            (
                Entity,
                &PredictionContext<CelestialTrajectory<Forward>>,
                &PredictionContext<CelestialTrajectory<Backward>>,
                Option<Ref<PredictionTracker<CelestialTrajectory<Forward>>>>,
                Option<Ref<PredictionTracker<CelestialTrajectory<Backward>>>>,
            ),
            With<SystemRoot>,
        >,
        time: Res<Time>,
        mut forward_time: Local<Option<std::time::Duration>>,
        mut backward_time: Local<Option<std::time::Duration>>,
    ) {
        // One system for now (maybe ever).
        let (root, forward, backward, forward_tracker, backward_tracker) =
            prediction.single().expect("No root entity found");

        let Ok(ctx) = contexts.ctx_mut() else {
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
                    ui.add_enabled_ui(backward_tracker.is_none(), |ui| {
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
                    backward_tracker,
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
                    ui.add_enabled_ui(forward_tracker.is_none(), |ui| {
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
                    forward_tracker,
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
