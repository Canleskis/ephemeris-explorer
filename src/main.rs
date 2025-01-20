mod analysis;
mod auto_extend;
mod camera;
mod compound_trajectory;
mod flight_plan;
mod floating_origin;
mod hierarchy;
mod load;
mod plot;
mod prediction;
mod rotation;
mod selection;
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
    plot::TrajectoryPlotPlugin,
    prediction::{
        Backward, DiscreteStatesBuilder, FixedSegmentsBuilder, Forward, PredictionPlugin,
    },
    selection::SelectionPlugin,
    starlight::StarLightPlugin,
    time::SimulationTimePlugin,
    ui::UiPlugin,
};

use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
#[expect(unused)]
use bevy_inspector_egui::quick::WorldInspectorPlugin;

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
                        ..default()
                    }),
                    ..default()
                })
                .disable::<TransformPlugin>(),
            bevy::diagnostic::FrameTimeDiagnosticsPlugin,
            EguiPlugin,
            CameraPlugin,
            SelectionPlugin, // TODO: refactor with bevy_picking with 0.15
            StarLightPlugin,
            FloatingOriginPlugin::default(),
            TrajectoryPlotPlugin,
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
        .init_state::<MainState>()
        .enable_state_scoped_entities::<MainState>()
        .insert_resource(Msaa::Sample8)
        .add_systems(Startup, default_solar_system)
        .add_systems(PreUpdate, enter_full_screen)
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
            panic!("Failed to load default solar system: {}", err);
        }
    }
}

fn enter_full_screen(
    kb: Res<ButtonInput<KeyCode>>,
    mut query_window: Query<&mut Window, With<bevy::window::PrimaryWindow>>,
) {
    if kb.pressed(KeyCode::AltLeft) && kb.just_pressed(KeyCode::Enter) {
        let mode = &mut query_window.single_mut().mode;
        bevy::log::info!("Toggling window mode");
        match *mode {
            WindowMode::Windowed => *mode = WindowMode::BorderlessFullscreen,
            _ => *mode = WindowMode::Windowed,
        }
    }
}
