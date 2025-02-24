use crate::ui::LabelSettings;

use bevy::{
    core_pipeline::bloom::Bloom,
    prelude::*,
    window::{MonitorSelection, WindowMode},
};
use big_space::camera::CameraController;
use serde::{Deserialize, Serialize};

pub struct PersistentSettingsPlugin;

impl Plugin for PersistentSettingsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AppSettings>().add_systems(
            PostUpdate,
            (
                AppSettings::apply,
                AppSettings::write.run_if(resource_changed::<AppSettings>),
            ),
        );
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct UserSettings {
    pub fullscreen: bool,
    pub bloom_intensity: f32,
    pub fov: f32,
    pub line_width: f32,
    pub show_labels: bool,
    pub mouse_sensitivity: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WindowSettings {
    pub size: Vec2,
    pub position: IVec2,
}

#[derive(Clone, Copy, Debug, PartialEq, Resource, Serialize, Deserialize)]
pub struct AppSettings {
    pub user: UserSettings,
    pub window: WindowSettings,
}

impl Default for AppSettings {
    fn default() -> Self {
        dirs::config_dir()
            .and_then(|config_dir| {
                toml::from_str(
                    &std::fs::read_to_string(config_dir.join("Ephemeris Explorer/settings.toml"))
                        .ok()?,
                )
                .ok()
            })
            .unwrap_or(Self {
                user: UserSettings {
                    fullscreen: false,
                    bloom_intensity: 0.15,
                    fov: 45.0,
                    show_labels: true,
                    line_width: 1.0,
                    mouse_sensitivity: 1.0,
                },
                window: WindowSettings {
                    size: Vec2::new(1280.0, 720.0),
                    position: IVec2::new(0, 0),
                },
            })
    }
}

impl AppSettings {
    fn apply(
        mut settings: ResMut<AppSettings>,
        label_settings: ResMut<LabelSettings>,
        gizmos_config: ResMut<GizmoConfigStore>,
        mut controller: Query<&mut CameraController>,
        mut window: Query<&mut Window>,
        mut bloom: Query<&mut Bloom>,
        mut projection: Query<(&mut PerspectiveProjection, &mut Projection)>,
    ) {
        if let Ok(mut window) = window.get_single_mut() {
            window
                .reborrow()
                .map_unchanged(|window| &mut window.mode)
                .set_if_neq(match settings.user.fullscreen {
                    true => WindowMode::BorderlessFullscreen(MonitorSelection::Current),
                    false => WindowMode::Windowed,
                });

            let current_size = window.bypass_change_detection().size();
            let current_position = match window.bypass_change_detection().position {
                WindowPosition::At(position) => position,
                _ => unreachable!(),
            };
            if window.is_changed() && !window.is_added() {
                settings.window.size = current_size;
                settings.window.position = current_position;
            }

            if current_size != settings.window.size {
                window
                    .resolution
                    .set(settings.window.size.x, settings.window.size.y);
            }
            if current_position != settings.window.position {
                window.position.set(settings.window.position);
            }
        }
        if let Ok(bloom) = bloom.get_single_mut() {
            bloom
                .map_unchanged(|bloom| &mut bloom.intensity)
                .set_if_neq(settings.user.bloom_intensity);
        }
        if let Ok((perspective, projection)) = projection.get_single_mut() {
            perspective
                .map_unchanged(|perspective| &mut perspective.fov)
                .set_if_neq(settings.user.fov.to_radians());
            projection
                .map_unchanged(|projection| match projection {
                    Projection::Perspective(perspective) => &mut perspective.fov,
                    _ => unreachable!(),
                })
                .set_if_neq(settings.user.fov.to_radians());
        }
        gizmos_config
            .map_unchanged(|c| &mut c.config_mut::<DefaultGizmoConfigGroup>().0.line_width)
            .set_if_neq(settings.user.line_width);

        if let Ok(mut controller) = controller.get_single_mut() {
            controller
                .reborrow()
                .map_unchanged(|controller| &mut controller.speed_pitch)
                .set_if_neq(settings.user.mouse_sensitivity * 2e-2);
            controller
                .map_unchanged(|controller| &mut controller.speed_yaw)
                .set_if_neq(settings.user.mouse_sensitivity * 2e-2);
        }

        label_settings
            .map_unchanged(|s| &mut s.enabled)
            .set_if_neq(settings.user.show_labels);
    }

    fn write(settings: Res<AppSettings>) {
        let Some(config_dir) = dirs::config_dir() else {
            return;
        };
        let config_dir = config_dir.join("Ephemeris Explorer");
        if !config_dir.exists() {
            std::fs::create_dir(&config_dir).unwrap();
        }
        let settings = toml::to_string_pretty(&*settings).unwrap();
        std::fs::write(config_dir.join("settings.toml"), settings).unwrap();
    }
}
