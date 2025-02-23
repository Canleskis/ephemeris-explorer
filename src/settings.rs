use bevy::{
    core_pipeline::bloom::Bloom,
    prelude::*,
    window::{MonitorSelection, WindowMode},
};
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

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Resource, Serialize, Deserialize)]
pub struct AppSettings {
    pub fullscreen: bool,
    pub bloom_intensity: f32,
    pub fov: f32,
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
                fullscreen: false,
                bloom_intensity: 0.15,
                fov: 45.0,
            })
    }
}

impl AppSettings {
    fn apply(
        settings: Res<AppSettings>,
        mut window: Query<&mut Window>,
        mut bloom: Query<&mut Bloom>,
        mut projection: Query<(&mut PerspectiveProjection, &mut Projection)>,
    ) {
        if let Ok(window) = window.get_single_mut() {
            window
                .map_unchanged(|window| &mut window.mode)
                .set_if_neq(match settings.fullscreen {
                    true => WindowMode::BorderlessFullscreen(MonitorSelection::Current),
                    false => WindowMode::Windowed,
                });
        }
        if let Ok(bloom) = bloom.get_single_mut() {
            bloom
                .map_unchanged(|bloom| &mut bloom.intensity)
                .set_if_neq(settings.bloom_intensity);
        }
        if let Ok((perspective, projection)) = projection.get_single_mut() {
            perspective
                .map_unchanged(|perspective| &mut perspective.fov)
                .set_if_neq(settings.fov.to_radians());
            projection
                .map_unchanged(|projection| match projection {
                    Projection::Perspective(perspective) => &mut perspective.fov,
                    _ => unreachable!(),
                })
                .set_if_neq(settings.fov.to_radians());
        }
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
