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
    load::{LoadSolarSystemEvent, LoadSystemPlugin},
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

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MainState {
    #[default]
    Loading,
    Running,
}

struct MainPlugins;

impl PluginGroup for MainPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        bevy::app::PluginGroupBuilder::start::<Self>()
            .add(bevy::app::PanicHandlerPlugin)
            .add(bevy::log::LogPlugin::default())
            .add(bevy::core::TaskPoolPlugin::default())
            .add(bevy::core::TypeRegistrationPlugin)
            .add(bevy::core::FrameCountPlugin)
            .add(bevy::time::TimePlugin)
            .add(bevy::hierarchy::HierarchyPlugin)
            .add(bevy::diagnostic::DiagnosticsPlugin)
            .add(bevy::input::InputPlugin)
            .add(bevy::app::ScheduleRunnerPlugin::default())
            .add(bevy::window::WindowPlugin {
                primary_window: Some(Window {
                    visible: false,
                    title: "Ephemeris Explorer".to_owned(),
                    position: WindowPosition::At(IVec2::new(320, 180)),
                    ..default()
                }),
                ..default()
            })
            .add(bevy::a11y::AccessibilityPlugin)
            .add(bevy::app::TerminalCtrlCHandlerPlugin)
            .add(bevy::asset::AssetPlugin::default())
            .add(bevy::winit::WinitPlugin::<bevy::winit::WakeUp>::default())
            .add(bevy::render::RenderPlugin::default())
            .add(bevy::render::texture::ImagePlugin::default())
            .add(bevy::render::pipelined_rendering::PipelinedRenderingPlugin)
            .add(bevy::core_pipeline::CorePipelinePlugin)
            .add(bevy::sprite::SpritePlugin::default())
            .add(bevy::text::TextPlugin)
            .add(bevy::pbr::PbrPlugin::default())
            .add(bevy::gizmos::GizmoPlugin)
            .add(bevy::state::app::StatesPlugin)
            .add(bevy::picking::input::PointerInputPlugin::default())
            .add(bevy::picking::PickingPlugin::default())
            .add(bevy::picking::InteractionPlugin)
            .add(bevy::diagnostic::FrameTimeDiagnosticsPlugin)
            .add(PersistentSettingsPlugin)
            .add(CameraPlugin)
            .add(SelectionPlugin)
            .add(StarLightPlugin)
            .add(FloatingOriginPlugin::default())
            .add(SimulationTimePlugin)
            .add(OrbitalAnalysisPlugin)
            .add(UiPlugin)
            .add(LoadSystemPlugin)
            .add(FlightPlanPlugin)
            .add(PredictionPlugin::<FixedSegmentsBuilder<Forward>>::default())
            .add(PredictionPlugin::<FixedSegmentsBuilder<Backward>>::default())
            .add(PredictionPlugin::<DiscreteStatesBuilder>::default())
            .add(AutoExtendPlugin::<FixedSegmentsBuilder<Forward>>::default())
            .add(AutoExtendPlugin::<FixedSegmentsBuilder<Backward>>::default())
    }
}

fn main() {
    App::new()
        .add_plugins((
            MainPlugins,
            #[cfg(debug_assertions)]
            bevy::remote::RemotePlugin::default(),
            #[cfg(debug_assertions)]
            bevy::remote::http::RemoteHttpPlugin::default(),
        ))
        .init_state::<MainState>()
        .enable_state_scoped_entities::<MainState>()
        .add_systems(Startup, (set_window_icon, load_initial_solar_system))
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

fn load_initial_solar_system(
    settings: Res<AppSettings>,
    asset_server: Res<AssetServer>,
    mut events: EventWriter<LoadSolarSystemEvent>,
) {
    match LoadSolarSystemEvent::try_from_dir(&settings.user.system_path, &asset_server) {
        Ok(event) => {
            events.send(event);
        }
        Err(err) => {
            panic!("Failed to load solar system: {}", err);
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
