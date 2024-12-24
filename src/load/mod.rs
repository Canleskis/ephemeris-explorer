pub mod solar_system;
pub mod ui;

pub use solar_system::*;
pub use ui::*;

use crate::{
    analysis::SphereOfInfluence,
    camera::{CanFollow, Followed, OrbitCamera},
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    floating_origin::{BigReferenceFrameBundle, BigSpaceRootBundle, FloatingOrigin, GridCell},
    hierarchy,
    plot::{PlotPoints, TrajectoryPlot},
    prediction::{
        Backward, DiscreteStates, DiscreteStatesBuilder, ExtendPredictionEvent, FixedSegments,
        FixedSegmentsBuilder, Forward, Mu, PredictionTracker, Trajectory, DIV,
    },
    selection::Clickable,
    starlight::Star,
    time::{BoundsTime, SimulationTime},
    ui::Labelled,
    MainState, SystemRoot,
};

use bevy::asset::{LoadedFolder, RecursiveDependencyLoadState};
use bevy::core_pipeline::Skybox;
use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, SubStates)]
#[source(MainState = MainState::Loading)]
pub enum LoadingStage {
    #[default]
    Assets,
    Ephemerides,
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, SubStates)]
#[source(LoadingStage = LoadingStage::Ephemerides)]
pub enum EphemeridesLoadingStage {
    #[default]
    Bodies,
    Ships,
}

pub struct LoadingPlugin;

impl Plugin for LoadingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(First, delay_window_visiblity);
        app.add_plugins(LoadingScreenPlugin)
            .add_sub_state::<LoadingStage>()
            .add_sub_state::<EphemeridesLoadingStage>()
            .add_plugins(LoadSolarSytemPlugin)
            .add_event::<LoadSolarSystemEvent>()
            .add_systems(First, handle_load_events);

        app.add_systems(
            Update,
            text_on_load::<Image>.run_if(in_state(LoadingStage::Assets)),
        )
        .add_systems(
            First,
            finish_assets_loading
                .after(handle_load_events)
                .run_if(
                    in_state(LoadingStage::Assets)
                        .and_then(bodies_loaded)
                        .and_then(eph_settings_loaded)
                        .and_then(hierarchy_loaded)
                        .and_then(skybox_loaded)
                        .and_then(ships_loaded),
                )
                .chain(),
        )
        .add_systems(
            OnExit(LoadingStage::Assets),
            (spawn_bodies, spawn_ships, setup_camera, default_follow).chain(),
        );

        app.add_systems(
            OnEnter(EphemeridesLoadingStage::Bodies),
            compute_ephemerides_bodies,
        )
        .add_systems(
            Update,
            ephemerides_bodies_progress.run_if(in_state(EphemeridesLoadingStage::Bodies)),
        )
        .add_systems(
            OnEnter(EphemeridesLoadingStage::Ships),
            (compute_ephemerides_ships, apply_deferred).chain(),
        )
        .add_systems(
            Update,
            ephemerides_ships_progress.run_if(in_state(EphemeridesLoadingStage::Ships)),
        )
        .add_systems(
            Update,
            bypass_ephemerides_loading.run_if(in_state(LoadingStage::Ephemerides)),
        );

        bevy::asset::load_internal_binary_asset!(
            app,
            Handle::default(),
            "../../assets/fonts/Montserrat-Regular.ttf",
            |bytes: &[u8], _path: String| { Font::try_from_bytes(bytes.to_vec()).unwrap() }
        );
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

#[derive(Event, Clone)]
pub struct LoadSolarSystemEvent {
    pub state: Handle<SolarSystem>,
    pub eph_settings: Handle<EphemeridesSettings>,
    pub hierarchy_tree: Handle<HierarchyTree>,
    pub skybox: Handle<Image>,
    pub ship: Handle<LoadedFolder>,
}

impl LoadSolarSystemEvent {
    pub fn try_from_dir(
        dir: &std::path::Path,
        asset_server: &AssetServer,
    ) -> std::io::Result<Self> {
        // We can't really check if the files exist for now.
        // Hopefully in the future we can use the asset server to check if the files exist.
        Ok(Self {
            state: asset_server.load(dir.join("state.json")),
            eph_settings: asset_server.load(dir.join("ephemeris.json")),
            hierarchy_tree: asset_server.load(dir.join("hierarchy.json")),
            skybox: asset_server.load(dir.join("skybox.png")),
            ship: asset_server.load_folder(dir.join("ships")),
        })
    }
}

pub fn handle_load_events(
    mut commands: Commands,
    mut events: EventReader<LoadSolarSystemEvent>,
    mut next_state: ResMut<NextState<MainState>>,
) {
    for event in events.read() {
        commands.insert_resource(UniqueAssetHandle(event.state.clone()));
        commands.insert_resource(UniqueAssetHandle(event.eph_settings.clone()));
        commands.insert_resource(UniqueAssetHandle(event.hierarchy_tree.clone()));
        commands.insert_resource(SkyboxHandle {
            reconfigured: false,
            handle: event.skybox.clone(),
        });
        commands.insert_resource(ShipsFolderHandle {
            handle: event.ship.clone(),
        });

        next_state.set(MainState::Loading);
    }
}

fn finish_assets_loading(world: &mut World) {
    world
        .resource_mut::<NextState<LoadingStage>>()
        .set(LoadingStage::Ephemerides);
}

fn bodies_loaded(solar_system: UniqueAsset<SolarSystem>) -> bool {
    matches!(
        solar_system.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed
    )
}

fn eph_settings_loaded(eph_settings: UniqueAsset<EphemeridesSettings>) -> bool {
    matches!(
        eph_settings.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed
    )
}

fn hierarchy_loaded(hierarchy: UniqueAsset<HierarchyTree>) -> bool {
    matches!(
        hierarchy.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed
    )
}

fn skybox_loaded(skybox: Res<SkyboxHandle>, asset_server: Res<AssetServer>) -> bool {
    matches!(
        asset_server.recursive_dependency_load_state(&skybox.handle),
        RecursiveDependencyLoadState::Failed
    ) || skybox.reconfigured
}

fn ships_loaded(folder: Res<ShipsFolderHandle>, asset_server: Res<AssetServer>) -> bool {
    matches!(
        asset_server.recursive_dependency_load_state(&folder.handle),
        RecursiveDependencyLoadState::Loaded
    )
}

#[derive(Component)]
pub struct Rotating {
    #[expect(unused)]
    right_ascension: f64,
    #[expect(unused)]
    declination: f64,
    reference_epoch: Epoch,
    reference_rotation: f64,
    rotation_rate: f64,
}

impl Rotating {
    pub fn at(&self, epoch: Epoch) -> bevy::math::DQuat {
        let dt = (epoch - self.reference_epoch).to_unit(hifitime::Unit::Day);
        bevy::math::DQuat::from_rotation_z(self.reference_rotation + dt * self.rotation_rate)
    }
}

fn spawn_bodies(
    mut commands: Commands,
    solar_system: UniqueAsset<SolarSystem>,
    hierachy_tree: UniqueAsset<HierarchyTree>,
    eph_settings: UniqueAsset<EphemeridesSettings>,
    visuals: Res<Assets<BodyVisuals>>,
) {
    // We can assume all assets are loaded at this point and just unwrap everything.
    let system = solar_system.get().unwrap();
    let hierachy_tree = hierachy_tree.get().unwrap();
    let eph_settings = eph_settings.get().unwrap();
    let dt = eph_settings.dt;

    // Hierarchy should always have one root element.
    let (name, tree) = hierachy_tree.first().unwrap();
    let root = commands
        .spawn((
            StateScoped(MainState::Running),
            Name::new(name.to_string()),
            BigSpaceRootBundle::default(),
            SystemRoot,
            CanFollow {
                min_distance: 1e3,
                max_distance: 1e10,
            },
        ))
        .id();

    let mut massive_states = Vec::new();

    spawn_from_hierarchy(
        0,
        root,
        &mut commands,
        &mut massive_states,
        root,
        tree,
        eph_settings,
        system,
        &visuals,
    );

    #[expect(clippy::too_many_arguments)]
    fn spawn_from_hierarchy(
        depth: usize,
        root: Entity,
        commands: &mut Commands,
        massive_states: &mut Vec<(Entity, DVec3, DVec3, f64, usize)>,
        parent: Entity,
        tree: &HierarchyTree,
        eph_settings: &EphemeridesSettings,
        solar_system: &SolarSystem,
        visuals: &Assets<BodyVisuals>,
    ) {
        for (name, tree) in tree.iter() {
            let body = solar_system.bodies.get(name).unwrap();
            let visual = visuals.get(&body.visuals).unwrap();
            let settings = eph_settings.settings.get(name).unwrap();

            let radius = ((visual.radii.x + visual.radii.y + visual.radii.z) / 3.0) as f32;
            let soi = massive_states
                .iter()
                .find(|(entity, ..)| *entity == parent)
                .map(|(_, _, parent_position, parent_mu, _)| {
                    SphereOfInfluence::approximate(
                        body.position.distance(*parent_position),
                        body.mu,
                        *parent_mu,
                    )
                })
                .unwrap_or(SphereOfInfluence::Fixed(f64::INFINITY));

            let mut entity = commands.spawn((
                // No state scope needed as it is always a child of the root.
                Name::new(name.clone()),
                BigReferenceFrameBundle {
                    transform: Transform::from_translation(body.position.as_vec3()),
                    ..default()
                },
                Clickable {
                    radius,
                    index: depth + 1,
                },
                CanFollow {
                    min_distance: radius as f64 * 1.05,
                    max_distance: 5e10,
                },
                Labelled {
                    style: TextStyle {
                        font_size: 15.0,
                        color: Color::WHITE,
                        ..default()
                    },
                    offset: Vec2::new(0.0, radius) * 1.1,
                    index: depth + 1,
                },
                Mu(body.mu),
                Trajectory::new(FixedSegments::new(
                    solar_system.epoch,
                    eph_settings.dt * DIV as i64 * settings.count as i64,
                )),
                BoundsTime,
                soi,
                TrajectoryPlot {
                    enabled: depth <= 1,
                    color: visual.orbit.color,
                    start: Epoch::from_tai_duration(-Duration::MAX),
                    end: Epoch::from_tai_duration(Duration::MAX),
                    threshold: 0.5,
                    max_points: 10_000,
                    reference: Some(parent),
                },
                PlotPoints::default(),
            ));

            massive_states.push((
                entity.id(),
                body.velocity,
                body.position,
                body.mu,
                settings.degree,
            ));

            entity.with_children(|parent| {
                let mut child = parent.spawn((
                    PbrBundle {
                        // The mesh is a sphere of `radius`, so we scale based on it.
                        transform: Transform::from_scale(visual.radii.as_vec3() / radius),
                        mesh: visual.mesh.clone(),
                        material: visual.material.clone(),
                        ..default()
                    },
                    Rotating {
                        right_ascension: visual.right_ascension,
                        declination: visual.declination,
                        reference_epoch: visual.rotation_reference_epoch,
                        reference_rotation: visual.rotation_reference,
                        rotation_rate: visual.rotation_rate,
                    },
                ));

                if let Some(color) = visual.light {
                    child.insert((
                        Star {
                            color,
                            illuminance: 120_000.0,
                        },
                        bevy::pbr::NotShadowCaster,
                    ));
                }
            });

            let child = entity.id();
            commands.entity(root).add_child(child);
            commands.add(hierarchy::AddChild { parent, child });

            spawn_from_hierarchy(
                depth + 1,
                root,
                commands,
                massive_states,
                child,
                tree,
                eph_settings,
                solar_system,
                visuals,
            );
        }
    }

    commands.entity(root).insert((
        FixedSegmentsBuilder::new(Forward::new(dt), system.epoch, massive_states.clone()),
        FixedSegmentsBuilder::new(Backward::new(dt), system.epoch, massive_states),
    ));

    commands.insert_resource(SimulationTime::new(system.epoch));
}

fn spawn_ships(
    mut commands: Commands,
    folder: Res<ShipsFolderHandle>,
    assets: Res<Assets<Ship>>,
    folder_assets: Res<Assets<LoadedFolder>>,
    query_names: Query<(Entity, &Name)>,
    root: Query<Entity, With<SystemRoot>>,
) {
    let root = root.single();
    let find_by_name = |reference| {
        query_names
            .iter()
            .find(|(_, name)| name.as_str() == reference)
            .map(|(entity, _)| entity)
    };

    let Some(folder) = folder_assets.get(&folder.handle) else {
        return;
    };

    for ship in folder
        .handles
        .iter()
        .filter_map(|handle| assets.get(&handle.clone().try_typed().ok()?))
    {
        let radius = 1.0;

        let end = ship
            .burns
            .last()
            .map(|burn| burn.start + burn.duration)
            .map(|end| end + ((end - ship.start) / 5).round(Duration::from_hours(1.0)))
            .filter(|end| *end > ship.start + Duration::from_days(1.0))
            .unwrap_or(ship.start + Duration::from_days(1.0));

        let mut entity = commands.spawn_empty();
        entity.insert((
            // No state scope needed as it is always a child of the root.
            Name::new(ship.name.clone()),
            BigReferenceFrameBundle {
                transform: Transform::from_translation(ship.position.as_vec3()),
                ..default()
            },
            Clickable { radius, index: 99 },
            CanFollow {
                min_distance: radius as f64 * 1.05,
                max_distance: 5e10,
            },
            Labelled {
                style: TextStyle {
                    font_size: 15.0,
                    color: Color::WHITE,
                    ..default()
                },
                offset: Vec2::new(0.0, radius) * 1.1,
                index: 99,
            },
            Trajectory::new(DiscreteStates::new(
                ship.start,
                ship.velocity,
                ship.position,
            )),
            DiscreteStatesBuilder::new(entity.id(), ship.start, ship.velocity, ship.position),
            FlightPlan::with(
                end,
                10_000,
                ship.burns
                    .iter()
                    .map(|burn| {
                        let (reference, frame) = burn
                            .reference
                            .as_ref()
                            .and_then(find_by_name)
                            .map(|entity| (entity, BurnFrame::Frenet))
                            .unwrap_or((root, BurnFrame::Cartesian));
                        Burn::with(
                            burn.start,
                            burn.duration,
                            burn.acceleration,
                            reference,
                            frame,
                        )
                    })
                    .collect(),
            ),
            TrajectoryPlot {
                enabled: true,
                color: LinearRgba::GREEN.into(),
                start: Epoch::from_tai_duration(-Duration::MAX),
                end: Epoch::from_tai_duration(Duration::MAX),
                threshold: 0.5,
                max_points: 10_000,
                reference: None,
            },
            PlotPoints::default(),
        ));

        let child = entity.id();
        commands.entity(root).add_child(child);
    }
}

fn setup_camera(
    mut commands: Commands,
    skybox: Res<SkyboxHandle>,
    root: Query<Entity, With<SystemRoot>>,
) {
    commands
        .spawn((
            // No state scope needed as it is always a child of the root.
            Camera3dBundle {
                camera: Camera {
                    hdr: true,
                    ..default()
                },
                projection: Projection::Perspective(PerspectiveProjection {
                    fov: 45f32.to_radians(),
                    near: 0.001,
                    ..default()
                }),
                exposure: bevy::render::camera::Exposure { ev100: 12.0 },
                ..default()
            },
            PerspectiveProjection::default(),
            Skybox {
                image: skybox.handle.clone(),
                brightness: 1000.0,
            },
            FloatingOrigin,
            GridCell::default(),
            OrbitCamera::default().with_distance(5e4),
            big_space::camera::CameraController::default()
                .with_speed_bounds([1e-17, 10e35])
                .with_smoothness(0.0, 0.0)
                .with_speed(1.0)
                .with_speed_pitch(0.02)
                .with_speed_yaw(0.02)
                .with_speed_roll(0.5),
            bevy::core_pipeline::bloom::BloomSettings::NATURAL,
            IsDefaultUiCamera,
        ))
        .set_parent(root.single());
}

fn default_follow(mut commands: Commands, query: Query<(Option<Entity>, &Name)>) {
    for (entity, name) in &query {
        if name.contains("Earth") {
            commands.insert_resource(Followed(entity));
        }
    }
}

fn compute_ephemerides_bodies(mut commands: Commands, sim_time: Res<SimulationTime>) {
    let target_forward = Epoch::from_gregorian_tai_hms(1952, 1, 1, 0, 0, 0);
    let target_backward = Epoch::from_gregorian_tai_hms(1948, 1, 1, 0, 0, 0);
    let sync_count = 100;

    commands.trigger(ExtendPredictionEvent::<FixedSegmentsBuilder<Forward>>::all(
        target_forward - sim_time.current(),
        sync_count,
    ));
    commands.trigger(
        ExtendPredictionEvent::<FixedSegmentsBuilder<Backward>>::all(
            sim_time.current() - target_backward,
            sync_count,
        ),
    );
}

fn ephemerides_bodies_progress(
    mut state: ResMut<NextState<EphemeridesLoadingStage>>,
    mut query: Query<&mut Text, With<LoadingText>>,
    forward: Query<&PredictionTracker<FixedSegmentsBuilder<Forward>>>,
    backward: Query<&PredictionTracker<FixedSegmentsBuilder<Backward>>>,
) {
    if forward.is_empty() && backward.is_empty() {
        state.set(EphemeridesLoadingStage::Ships);
        return;
    }

    let forward_progress = forward
        .get_single()
        .map(PredictionTracker::progress)
        .unwrap_or(1.0);
    let backward_progress = backward
        .get_single()
        .map(PredictionTracker::progress)
        .unwrap_or(1.0);
    let progress = (forward_progress + backward_progress) / 2.0;
    for mut text in query.iter_mut() {
        text.sections[0].value = format!("Computing ephemerides (1/2): {:.0}%", progress * 100.0);
    }
}

fn compute_ephemerides_ships(
    mut commands: Commands,
    query_flight_path: Query<Entity, With<FlightPlan>>,
) {
    for entity in query_flight_path.iter() {
        commands.trigger_targets(FlightPlanChanged, entity);
    }
}

fn ephemerides_ships_progress(
    mut state: ResMut<NextState<MainState>>,
    mut query: Query<&mut Text, With<LoadingText>>,
    query_tracker: Query<&PredictionTracker<DiscreteStatesBuilder>>,
) {
    if query_tracker.is_empty() {
        state.set(MainState::Running);
        return;
    }

    let total_progress = query_tracker
        .iter()
        .map(PredictionTracker::progress)
        .sum::<f32>()
        / query_tracker.iter().len().max(1) as f32;
    for mut text in query.iter_mut() {
        text.sections[0].value = format!(
            "Computing ephemerides (2/2): {:.0}%",
            total_progress * 100.0
        );
    }
}

#[expect(unused)]
fn true_for_frames(frames: usize) -> impl Fn(Local<usize>) -> bool {
    move |mut completed: Local<usize>| {
        *completed += 1;
        *completed > frames
    }
}

fn bypass_ephemerides_loading(
    kb: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<NextState<MainState>>,
) {
    if kb.just_pressed(KeyCode::Escape) {
        state.set(MainState::Running);
    }
}
