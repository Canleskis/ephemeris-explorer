use crate::{simulation::SimulationTime, warp::StartWarp};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use ftime::{Duration, Epoch};
use std::str::FromStr;
use thousands::Separable;

#[derive(PartialEq, Eq, Clone, Copy, Default)]
pub enum TimeScale {
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

pub struct ComputedTimeScale {
    value: f64,
    last_update: std::time::Instant,
    sum: f64,
    count: usize,
}

impl Default for ComputedTimeScale {
    fn default() -> Self {
        Self {
            value: 0.0,
            last_update: std::time::Instant::now(),
            sum: 0.0,
            count: 0,
        }
    }
}

impl ComputedTimeScale {
    fn add(&mut self, value: f64) {
        let now = std::time::Instant::now();

        self.sum += value;
        self.count += 1;

        if now.duration_since(self.last_update) > std::time::Duration::from_millis(250) {
            if self.count > 0 {
                self.value = self.sum / self.count as f64;
            }
            self.last_update = now;
            self.sum = 0.0;
            self.count = 0;
        }
    }

    fn set_to(&mut self, value: f64) {
        self.last_update = std::time::Instant::now();
        self.value = value;
        self.sum = value;
        self.count = 1;
    }

    fn value(&self) -> f64 {
        self.value
    }
}

pub fn time_controls(
    mut commands: Commands,
    mut contexts: EguiContexts,
    diagnostics: Res<bevy::diagnostic::DiagnosticsStore>,
    mut sim_time: ResMut<SimulationTime>,
    mut buffer: Local<String>,
    mut scale: Local<TimeScale>,
    mut computed_time_scale: Local<ComputedTimeScale>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };

    const SMALLEST: f64 = 1e-1;
    const LARGEST: f64 = 1e10;

    computed_time_scale.add(sim_time.real_time_scale());

    egui::TopBottomPanel::bottom("Time").show(ctx, |ui| {
        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
            ui.spacing_mut().text_edit_width = 240.0;
            let edit = ui.add(egui::TextEdit::singleline(&mut *buffer));

            if edit.lost_focus()
                && let Ok(epoch) = Epoch::from_str(&buffer)
            {
                commands.trigger(StartWarp::new(epoch));
            }

            if !edit.has_focus() {
                *buffer = format!("{}", sim_time.current());
            }

            let pause_button = ui.add_sized(
                [40.0, 18.0],
                egui::Button::new(if sim_time.paused { "▶" } else { "⏸" }),
            );
            if pause_button.clicked() {
                sim_time.paused = !sim_time.paused;
                computed_time_scale.set_to(sim_time.effective_time_scale());
            }

            let step_up = |ts: &mut f64| *ts = (*ts * 10.0).clamp(-LARGEST, LARGEST);
            let step_down = |ts: &mut f64| *ts = (*ts / 10.0).clamp(-LARGEST, LARGEST);

            if ui.button("⏪").clicked() {
                match sim_time.time_scale {
                    m if m > -SMALLEST && m <= SMALLEST => sim_time.time_scale = -SMALLEST,
                    m if m > 0.0 => step_down(&mut sim_time.time_scale),
                    _ => step_up(&mut sim_time.time_scale),
                }
                computed_time_scale.set_to(sim_time.effective_time_scale());
            }

            ui.scope(|ui| {
                ui.spacing_mut().slider_width = ui.available_width() / 3.0;
                ui.spacing_mut().interact_size.x = 320.0;
                if round(computed_time_scale.value(), 2) != sim_time.effective_time_scale() {
                    ui.visuals_mut().override_text_color = Some(egui::Color32::RED);
                }

                let slider = egui::Slider::new(&mut sim_time.time_scale, -LARGEST..=LARGEST)
                    .logarithmic(true)
                    .fixed_decimals(1)
                    .smallest_positive(SMALLEST / 100.0)
                    .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.6 });

                let slider = match *scale {
                    TimeScale::TimePerSecond => slider
                        .suffix(" per second")
                        .custom_formatter(|_, _| {
                            let value = computed_time_scale.value();
                            format!(
                                "{}{}",
                                if value < 0.0 { "-" } else { "" },
                                (Duration::from_seconds(computed_time_scale.value()).abs())
                            )
                        })
                        .custom_parser(|text| Some(Duration::from_str(text).ok()?.as_seconds())),
                    TimeScale::Multiplier => slider.prefix("x").custom_formatter(|_, _| {
                        format!("{:.2}", computed_time_scale.value()).separate_with_commas()
                    }),
                };

                let slider = ui.add(slider);
                if slider.changed() {
                    computed_time_scale.set_to(sim_time.effective_time_scale());
                }

                if slider.is_pointer_button_down_on() {
                    egui::Tooltip::always_open(
                        slider.ctx,
                        slider.layer_id,
                        slider.id,
                        egui::PopupAnchor::Pointer,
                    )
                    .gap(12.0)
                    .show(|ui| {
                        ui.add(
                            egui::Label::new(format!("x{}", sim_time.time_scale))
                                .wrap_mode(egui::TextWrapMode::Extend),
                        );
                    });
                }
            });

            if ui.button("⏩").clicked() {
                match sim_time.time_scale {
                    m if (-SMALLEST..SMALLEST).contains(&m) => sim_time.time_scale = SMALLEST,
                    m if m > 0.0 => step_up(&mut sim_time.time_scale),
                    _ => step_down(&mut sim_time.time_scale),
                }
                computed_time_scale.set_to(sim_time.effective_time_scale());
            }

            egui::ComboBox::from_id_salt("time_scale")
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
                    ui.add(egui::Label::new(format!("FPS: {fps_smoothed:.0}")).selectable(false));
                }
            })
        });
    });
}
