mod camera;
mod ephemerides;
mod floating_origin;
mod hierarchy;
mod load;
mod plot;
mod selection;
mod starlight;
mod time;
mod ui;

use crate::{
    camera::CameraPlugin,
    ephemerides::{ComputeEphemeridesEvent, EphemerisComputePlugin},
    floating_origin::FloatingOriginPlugin,
    load::{LoadSolarSystemEvent, LoadingPlugin},
    plot::{EphemerisPlotConfig, EphemerisPlotPlugin},
    selection::SelectionPlugin,
    starlight::StarLightPlugin,
    time::EphemerisTimePlugin,
    ui::UiPlugin,
};

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
#[allow(unused)]
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use hifitime::Duration;

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MainState {
    #[default]
    Loading,
    Running,
}

/// A given stellar system, planetary system or satellite system is a child of a root entity with
/// this component.
#[derive(Component)]
pub struct SystemRoot;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        #[cfg(not(target_arch = "wasm32"))]
                        resolution: bevy::window::WindowResolution::new(1920.0, 1080.0),
                        prevent_default_event_handling: false,
                        canvas: Some("#app".to_owned()),
                        visible: false,
                        title: "Orbitism".to_owned(),
                        ..default()
                    }),
                    ..default()
                })
                .disable::<TransformPlugin>(),
            bevy::diagnostic::FrameTimeDiagnosticsPlugin,
            EguiPlugin,
            CameraPlugin,
            SelectionPlugin,
            StarLightPlugin,
            FloatingOriginPlugin::default(),
            EphemerisComputePlugin,
            EphemerisPlotPlugin,
            EphemerisTimePlugin,
            UiPlugin,
            LoadingPlugin,
        ))
        .init_state::<MainState>()
        .enable_state_scoped_entities::<MainState>()
        .insert_resource(Msaa::Sample8)
        .insert_resource(EphemerisPlotConfig {
            plot_history: false,
            ..default()
        })
        .add_systems(Startup, default_solar_system)
        .add_systems(First, delay_window_visiblity)
        .add_systems(
            PostUpdate,
            shortcut_send_prediction.run_if(in_state(MainState::Running)),
        )
        .run();
}

const DEFAULT_SOLAR_SYSTEM_PATH: &str = "systems/full_solar_system_2433282.500372499";

fn default_solar_system(
    asset_server: Res<AssetServer>,
    mut events: EventWriter<LoadSolarSystemEvent>,
) {
    match LoadSolarSystemEvent::try_from_dir(
        std::path::Path::new(DEFAULT_SOLAR_SYSTEM_PATH),
        &asset_server,
    ) {
        Ok(event) => {
            events.send(event);
        }
        Err(err) => {
            bevy::log::error!("Failed to load default solar system: {}", err);
        }
    }
}

fn delay_window_visiblity(mut window: Query<&mut Window>, frames: Res<bevy::core::FrameCount>) {
    // The delay may be different for your app or system.
    if frames.0 == 3 {
        // At this point the gpu is ready to show the app so we can make the window visible.
        // Alternatively, you could toggle the visibility in Startup.
        // It will work, but it will have one white frame before it starts rendering
        window.single_mut().visible = true;
    }
}

fn shortcut_send_prediction(
    input: Res<ButtonInput<KeyCode>>,
    mut event_writer: EventWriter<ComputeEphemeridesEvent>,
    root: Query<Entity, With<SystemRoot>>,
) {
    if input.just_pressed(KeyCode::Comma) {
        let root = root.single();
        let duration = Duration::from_days(365.0 * 1.0);
        let sync_count =
            (duration.total_nanoseconds() / Duration::from_days(25.0).total_nanoseconds()) as usize;

        event_writer.send(ComputeEphemeridesEvent {
            root,
            duration,
            sync_count,
        });
    }
}
