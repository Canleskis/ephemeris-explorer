#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod analysis;
mod auto_extend;
mod camera;
mod flight_plan;
mod floating_origin;
mod hierarchy;
mod load;
mod prediction;
mod rotation;
mod selection;
mod settings;
mod starlight;
mod time;
mod ui;

use crate::{
    analysis::OrbitalAnalysisPlugin,
    auto_extend::AutoExtendPlugin,
    camera::CameraPlugin,
    flight_plan::FlightPlanPlugin,
    floating_origin::FloatingOriginPlugin,
    load::{LoadSolarSystemEvent, LoadingPlugin},
    prediction::{
        Backward, DiscreteStatesBuilder, FixedSegmentsBuilder, Forward, PredictionPlugin,
    },
    selection::SelectionPlugin,
    settings::{AppSettings, PersistentSettingsPlugin},
    starlight::StarLightPlugin,
    time::SimulationTimePlugin,
    ui::UiPlugin,
};

use bevy::prelude::*;
use bevy_egui::EguiPlugin;

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MainState {
    #[default]
    Loading,
    Running,
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        prevent_default_event_handling: false,
                        canvas: Some("#app".to_owned()),
                        visible: false,
                        title: "Ephemeris Explorer".to_owned(),
                        position: WindowPosition::At(IVec2::new(320, 180)),
                        ..default()
                    }),
                    ..default()
                })
                .disable::<TransformPlugin>(),
            bevy::diagnostic::FrameTimeDiagnosticsPlugin,
            EguiPlugin,
            PersistentSettingsPlugin,
            CameraPlugin,
            SelectionPlugin,
            StarLightPlugin,
            FloatingOriginPlugin::default(),
            SimulationTimePlugin,
            OrbitalAnalysisPlugin,
            UiPlugin,
            LoadingPlugin,
            FlightPlanPlugin,
        ))
        .add_plugins((
            PredictionPlugin::<FixedSegmentsBuilder<Forward>>::default(),
            PredictionPlugin::<FixedSegmentsBuilder<Backward>>::default(),
            PredictionPlugin::<DiscreteStatesBuilder>::default(),
            AutoExtendPlugin::<FixedSegmentsBuilder<Forward>>::default(),
            AutoExtendPlugin::<FixedSegmentsBuilder<Backward>>::default(),
        ))
        .add_plugins((
            #[cfg(debug_assertions)]
            bevy::remote::RemotePlugin::default(),
            #[cfg(debug_assertions)]
            bevy::remote::http::RemoteHttpPlugin::default(),
        ))
        .init_state::<MainState>()
        .enable_state_scoped_entities::<MainState>()
        .add_systems(Startup, (set_window_icon, default_solar_system))
        .add_systems(First, delay_window_visiblity)
        .add_systems(PreUpdate, toggle_full_screen)
        .run();
}

// Sets the icon on windows and X11
fn set_window_icon(
    windows: NonSend<bevy::winit::WinitWindows>,
    primary_window: Query<Entity, With<bevy::window::PrimaryWindow>>,
) {
    let primary_entity = primary_window.single();
    let Some(primary) = windows.get_window(primary_entity) else {
        return;
    };
    let icon_buf = std::io::Cursor::new(include_bytes!(
        "../build/macos/AppIcon.iconset/icon_256x256.png"
    ));
    if let Ok(image) = image::load(icon_buf, image::ImageFormat::Png) {
        let image = image.into_rgba8();
        let (width, height) = image.dimensions();
        let rgba = image.into_raw();
        let icon = winit::window::Icon::from_rgba(rgba, width, height).unwrap();
        primary.set_window_icon(Some(icon));
    };
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
            panic!("Failed to load default solar system: {}", err);
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

fn toggle_full_screen(kb: Res<ButtonInput<KeyCode>>, settings: Option<ResMut<AppSettings>>) {
    let Some(mut settings) = settings else {
        return;
    };
    if kb.pressed(KeyCode::AltLeft) && kb.just_pressed(KeyCode::Enter) {
        settings.user.fullscreen = !settings.user.fullscreen;
    }
}
