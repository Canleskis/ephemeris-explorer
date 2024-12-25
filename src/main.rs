mod analysis;
mod auto_extend;
mod camera;
mod flight_plan;
mod floating_origin;
mod hierarchy;
mod load;
mod plot;
mod prediction;
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
    plot::{TrajectoryPlotConfig, TrajectoryPlotPlugin},
    prediction::{
        Backward, DiscreteStatesBuilder, FixedSegmentsBuilder, Forward, PredictionPlugin,
    },
    selection::SelectionPlugin,
    starlight::StarLightPlugin,
    time::SimulationTimePlugin,
    ui::UiPlugin,
};

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
#[allow(unused)]
use bevy_inspector_egui::quick::WorldInspectorPlugin;

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MainState {
    #[default]
    Loading,
    Running,
}

/// All the bodies in a stellar system, planetary system or satellite system should be a child of a
/// root entity with this component.
#[derive(Component)]
pub struct SystemRoot;

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
        .insert_resource(TrajectoryPlotConfig::default())
        .add_systems(Startup, default_solar_system)
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
