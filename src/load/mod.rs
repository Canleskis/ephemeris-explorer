pub mod solar_system;
pub mod ui;

pub use solar_system::*;
pub use ui::*;

use crate::{
    MainState,
    analysis::{SphereOfInfluence, under_soi},
    camera::{BigSpaceCameraController, CanFollow, Followed, OrbitCamera},
    dynamics::{
        Backward, Bodies, CubicHermiteSplineSamples, DEFAULT_PARAMS, Forward, LeastSquaresFit, Mu,
        NBodyPropagator, SpacecraftPropagator, StateVector, UniformSpline,
    },
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged},
    floating_origin::{BigGridBundle, BigSpaceRootBundle, FloatingOrigin, GridCell},
    hierarchy::AddOrbit,
    prediction::{ExtendPredictionEvent, PredictionContext, PredictionTracker, Trajectory},
    rotation::Rotating,
    selection::Selectable,
    simulation::{BoundsTime, SimulationTime},
    starlight::Star,
    ui::{Labelled, PlotConfig, PlotSource},
};

use bevy::asset::RecursiveDependencyLoadState;
use bevy::core_pipeline::Skybox;
use bevy::platform::hash::FixedState;
use bevy::prelude::*;
use ephemeris::{DIV, EvaluateTrajectory};
use ftime::Duration;
use std::hash::{BuildHasher, Hash};

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, SubStates)]
#[source(MainState = MainState::Loading)]
pub enum LoadingStage {
    #[default]
    Assets,
    Spawn,
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, SubStates)]
#[source(LoadingStage = LoadingStage::Spawn)]
pub enum SpawnStage {
    #[default]
    Bodies,
    Ships,
}

#[derive(Default)]
pub struct LoadSystemPlugin;

impl Plugin for LoadSystemPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(LoadingScreenPlugin)
            .add_sub_state::<LoadingStage>()
            .add_sub_state::<SpawnStage>()
            .add_plugins(LoadSolarSystemPlugin)
            .add_event::<LoadSolarSystemEvent>()
            .add_systems(First, handle_load_events);

        app.add_systems(
            Update,
            text_on_load::<Image>.run_if(in_state(LoadingStage::Assets)),
        )
        .add_systems(OnEnter(MainState::Loading), setup_loading_errors)
        .add_systems(
            First,
            (
                finish_assets_loading.after(handle_load_events).run_if(
                    in_state(LoadingStage::Assets)
                        .and(bodies_loaded)
                        .and(eph_settings_loaded)
                        .and(hierarchy_loaded)
                        .and(skybox_loaded)
                        .and(ships_loaded),
                ),
                agregate_asset_errors,
            )
                .chain(),
        );

        app.add_systems(
            OnEnter(SpawnStage::Bodies),
            (
                spawn_loaded_bodies,
                setup_camera,
                default_follow,
                compute_ephemerides_bodies,
            )
                .chain(),
        )
        .add_systems(
            Update,
            ephemerides_bodies_progress.run_if(in_state(SpawnStage::Bodies)),
        );
        app.add_observer(spawn_ship)
            .add_systems(OnEnter(SpawnStage::Ships), spawn_loaded_ships)
            .add_systems(
                Update,
                ephemerides_ships_progress.run_if(in_state(SpawnStage::Ships)),
            );

        #[cfg(debug_assertions)]
        app.add_systems(
            Update,
            bypass_ephemerides_loading.run_if(in_state(LoadingStage::Spawn)),
        );

        bevy::asset::load_internal_binary_asset!(
            app,
            Handle::default(),
            "../../assets/fonts/Montserrat-Medium.ttf",
            |bytes: &[u8], _: String| { Font::try_from_bytes(bytes.to_vec()).unwrap() }
        );
    }
}

#[derive(Event, Clone, Debug)]
pub struct LoadSolarSystemEvent {
    pub path: std::path::PathBuf,
    pub state: Handle<SolarSystemState>,
    pub eph_settings: Handle<EphemeridesSettings>,
    pub hierarchy_tree: Handle<HierarchyTree>,
    pub skybox: Handle<Image>,
    pub ships: Vec<Handle<Ship>>,
}

impl LoadSolarSystemEvent {
    pub fn try_from_dir(
        dir: &std::path::Path,
        asset_server: &AssetServer,
    ) -> std::io::Result<Self> {
        Ok(Self {
            path: dir.to_path_buf(),
            state: asset_server.load(dir.join("state.json")),
            eph_settings: asset_server.load(dir.join("ephemeris.json")),
            hierarchy_tree: asset_server.load(dir.join("hierarchy.json")),
            skybox: asset_server.load(dir.join("skybox.png")),
            ships: std::fs::read_dir(dir.join("ships"))
                .into_iter()
                .flatten()
                .flatten()
                .filter(|entry| entry.path().extension() == Some("json".as_ref()))
                .map(|entry| asset_server.load(entry.path()))
                .collect(),
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
        commands.insert_resource(ShipsHandles {
            handles: event.ships.clone(),
        });

        next_state.set(MainState::Loading);
    }
}

fn finish_assets_loading(world: &mut World) {
    world
        .resource_mut::<NextState<LoadingStage>>()
        .set(LoadingStage::Spawn);
}

fn bodies_loaded(solar_system: UniqueAsset<SolarSystemState>) -> bool {
    matches!(
        solar_system.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed(_)
    )
}

fn eph_settings_loaded(eph_settings: UniqueAsset<EphemeridesSettings>) -> bool {
    matches!(
        eph_settings.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed(_)
    )
}

fn hierarchy_loaded(hierarchy: UniqueAsset<HierarchyTree>) -> bool {
    matches!(
        hierarchy.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed(_)
    )
}

fn skybox_loaded(skybox: Res<SkyboxHandle>, asset_server: Res<AssetServer>) -> bool {
    matches!(
        asset_server.recursive_dependency_load_state(&skybox.handle),
        RecursiveDependencyLoadState::Failed(_)
    ) || skybox.reconfigured
}

fn ships_loaded(folder: Res<ShipsHandles>, asset_server: Res<AssetServer>) -> bool {
    folder.handles.iter().all(|handle| {
        matches!(
            asset_server.load_state(handle),
            bevy::asset::LoadState::Loaded | bevy::asset::LoadState::Failed(_)
        )
    })
}

#[derive(Clone, Debug)]
pub enum LoadingError {
    AssetLoadError(bevy::asset::AssetLoadError),
    MissingBody(String),
    MissingEphemeridesSettings(String),
    MissingReference { ship: String, reference: String },
}
impl std::fmt::Display for LoadingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AssetLoadError(err) => write!(f, "Asset load error: {err}"),
            Self::MissingBody(name) => write!(f, "Missing body in solar system state: {name}"),
            Self::MissingEphemeridesSettings(name) => {
                write!(f, "Missing ephemerides settings for body: {name}")
            }
            Self::MissingReference { ship, reference } => {
                write!(f, "Reference \"{reference}\" for \"{ship}\" does not exist")
            }
        }
    }
}
impl std::error::Error for LoadingError {}

#[derive(Default, Resource, Deref, DerefMut)]
pub struct LoadingErrors(pub Vec<LoadingError>);

fn setup_loading_errors(mut commands: Commands) {
    commands.insert_resource(LoadingErrors::default());
}

fn agregate_asset_errors(
    mut events: EventReader<bevy::asset::UntypedAssetLoadFailedEvent>,
    mut errors: ResMut<LoadingErrors>,
) {
    errors.extend(
        events
            .read()
            .map(|event| LoadingError::AssetLoadError(event.error.clone())),
    );
}

fn spawn_loaded_bodies(
    mut commands: Commands,
    solar_system: UniqueAsset<SolarSystemState>,
    hierachy_tree: UniqueAsset<HierarchyTree>,
    eph_settings: UniqueAsset<EphemeridesSettings>,
    visuals: Res<Assets<BodyVisuals>>,
    mut errors: ResMut<LoadingErrors>,
) {
    // We can assume all assets are loaded at this point and just unwrap everything.
    let empty = HierarchyTree::empty();
    let hierachy_tree = hierachy_tree.get().unwrap_or(&empty);

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

    let empty = SolarSystemState::default();
    let system = solar_system.get().unwrap_or(&empty);

    let empty = EphemeridesSettings::default();
    let settings = eph_settings.get().unwrap_or(&empty);

    let mut spawn_data = Vec::new();

    enum MissingDataError {
        State(String),
        EphemeridesSettings(String),
    }

    #[expect(clippy::type_complexity)]
    fn spawn_from_hierarchy(
        depth: usize,
        root: Entity,
        commands: &mut Commands,
        spawn_data: &mut Vec<(Entity, (StateVector, f64, Duration, LeastSquaresFit))>,
        parent: Entity,
        tree: &HierarchyTree,
        eph_settings: &EphemeridesSettings,
        solar_system: &SolarSystemState,
        visuals: &Assets<BodyVisuals>,
    ) -> Vec<MissingDataError> {
        let mut errors = Vec::new();
        for (name, tree) in tree.iter() {
            let body = match solar_system.bodies.get(name) {
                Some(body) => body,
                None => {
                    errors.push(MissingDataError::State(name.clone()));
                    continue;
                }
            };
            let settings = match eph_settings.settings.get(name) {
                Some(settings) => settings,
                None => {
                    errors.push(MissingDataError::EphemeridesSettings(name.clone()));
                    continue;
                }
            };
            let visual = visuals.get(&body.visuals).unwrap();

            let radius = ((visual.radii.x + visual.radii.y + visual.radii.z) / 3.0) as f32;
            let soi = spawn_data
                .iter()
                .find(|(entity, ..)| *entity == parent)
                .map(|(_, (parent_sv, parent_mu, ..))| {
                    SphereOfInfluence::approximate(
                        body.position.distance(parent_sv.position),
                        body.mu,
                        *parent_mu,
                    )
                })
                .unwrap_or(SphereOfInfluence::Fixed(f64::INFINITY));

            let sample_period = eph_settings.dt * settings.count as f64;

            let mut entity = commands.spawn_empty();
            entity.insert((
                // No state scope needed as it is always a child of the root.
                Name::new(name.clone()),
                BigGridBundle {
                    transform: Transform::from_translation(body.position.as_vec3()),
                    ..default()
                },
                Selectable {
                    radius,
                    index: depth + 1,
                },
                CanFollow {
                    min_distance: radius as f64 * 1.05,
                    max_distance: 5e10,
                },
                Labelled {
                    font: TextFont::from_font_size(12.0),
                    colour: TextColor(Color::WHITE),
                    offset: Vec2::new(0.0, radius) * 1.1,
                    index: depth + 1,
                },
                Mu(body.mu),
                Trajectory::new(UniformSpline::new(
                    solar_system.epoch,
                    sample_period * DIV as f64,
                )),
                BoundsTime,
                soi,
            ));

            spawn_data.push((
                entity.id(),
                (
                    StateVector::new(body.position, body.velocity),
                    body.mu,
                    sample_period,
                    LeastSquaresFit {
                        degree: settings.degree,
                    },
                ),
            ));

            entity.with_children(|cmds| {
                cmds.spawn((
                    Name::new(name.to_string()),
                    PlotSource(cmds.target_entity()),
                    PlotConfig {
                        enabled: depth <= 1,
                        color: visual.orbit.color,
                        start: solar_system.epoch - Duration::from_years(1000.0),
                        end: solar_system.epoch + Duration::from_years(1000.0),
                        threshold: 0.5,
                        max_points: 10_000,
                        reference: Some(parent),
                    },
                ));

                let mut child = cmds.spawn((
                    Transform::from_scale(visual.radii.as_vec3() / radius),
                    Mesh3d(visual.mesh.clone()),
                    MeshMaterial3d(visual.material.clone()),
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

            let entity = entity.id();
            commands.entity(root).add_child(entity);
            commands.queue(AddOrbit {
                orbiting: parent,
                body: entity,
            });

            errors.extend(spawn_from_hierarchy(
                depth + 1,
                root,
                commands,
                spawn_data,
                entity,
                tree,
                eph_settings,
                solar_system,
                visuals,
            ))
        }

        errors
    }

    let spawn_errors = spawn_from_hierarchy(
        0,
        root,
        &mut commands,
        &mut spawn_data,
        root,
        tree,
        settings,
        system,
        &visuals,
    );

    for error in spawn_errors {
        match error {
            MissingDataError::State(name) => {
                errors.push(LoadingError::MissingBody(name));
            }
            MissingDataError::EphemeridesSettings(name) => {
                errors.push(LoadingError::MissingEphemeridesSettings(name));
            }
        }
    }

    let (entities, states): (Vec<_>, Vec<_>) = spawn_data.into_iter().collect();
    let dt = settings.dt;
    commands.entity(root).insert((
        PredictionContext::new(
            entities.clone(),
            NBodyPropagator::new(Forward::new(dt), system.epoch, states.clone()),
        ),
        PredictionContext::new(
            entities,
            NBodyPropagator::new(Backward::new(dt), system.epoch, states),
        ),
    ));

    commands.insert_resource(SimulationTime::new(system.epoch));
}

fn spawn_loaded_ships(
    mut commands: Commands,
    folder: Res<ShipsHandles>,
    assets: Res<Assets<Ship>>,
) {
    for ship in folder
        .handles
        .iter()
        .filter_map(|handle| assets.get(handle))
    {
        commands.trigger(SpawnShip(ship.clone()));
    }
}

#[derive(Event)]
pub struct SpawnShip(pub Ship);

pub fn find_by_name(query: &Query<(Entity, &Name), With<Mu>>, reference: &str) -> Option<Entity> {
    query
        .iter()
        .find(|(_, name)| name.as_str() == reference)
        .map(|(entity, _)| entity)
}

fn spawn_ship(
    trigger: Trigger<SpawnShip>,
    mut commands: Commands,
    query_name: Query<(Entity, &Name), With<Mu>>,
    query_soi: Query<(Entity, &Trajectory, &SphereOfInfluence)>,
    query_context: Query<(Entity, &Trajectory, &Mu)>,
    root: Single<Entity, With<SystemRoot>>,
    mut errors: ResMut<LoadingErrors>,
) {
    let ship = &trigger.event().0;

    let radius = 0.01;

    let parent = under_soi(
        query_soi.iter().filter_map(|(entity, trajectory, soi)| {
            Some((entity, trajectory.position(ship.start)?, soi))
        }),
        ship.position,
    );

    let context = query_context
        .iter()
        .map(|(e, traj, mu)| (e, (traj.clone(), *mu)))
        .collect();

    let mut entity = match trigger.target() {
        entity if entity == Entity::PLACEHOLDER => commands.spawn_empty(),
        entity => commands.entity(entity),
    };
    let id = entity.id();

    let mapped_burns: Result<Vec<_>, &str> = ship
        .burns
        .iter()
        .map(|burn| {
            let (reference, frame) = match &burn.reference {
                Some(reference) => (
                    find_by_name(&query_name, reference).ok_or(reference.as_str())?,
                    BurnFrame::Relative,
                ),
                None => (*root, BurnFrame::Inertial),
            };

            Ok(Burn::with(
                burn.start,
                burn.duration,
                burn.acceleration,
                reference,
                frame,
            ))
        })
        .collect();

    let mapped_burns = match mapped_burns {
        Ok(burns) => burns,
        Err(missing) => {
            errors.push(LoadingError::MissingReference {
                ship: ship.name.to_string(),
                reference: missing.to_string(),
            });
            return;
        }
    };

    let hash = FixedState::default().hash_one(&ship.name);
    let color = LinearRgba::new(
        (hash & 0xFF) as f32 / 255.0,
        ((hash >> 4) & 0xFF) as f32 / 255.0,
        ((hash >> 8) & 0xFF) as f32 / 255.0,
        1.0,
    );

    entity
        .insert((
            // No state scope needed as it is always a child of the root.
            Name::new(ship.name.to_string()),
            BigGridBundle {
                transform: Transform::from_translation(ship.position.as_vec3()),
                ..default()
            },
            Selectable { radius, index: 99 },
            CanFollow {
                min_distance: radius as f64 * 1.05,
                max_distance: 5e10,
            },
            Labelled {
                font: TextFont::from_font_size(12.0),
                colour: TextColor(Color::WHITE),
                offset: Vec2::new(0.0, radius) * 1.1,
                index: 99,
            },
            Trajectory::new(CubicHermiteSplineSamples::new(
                ship.start,
                ship.position,
                ship.velocity,
            )),
            PredictionContext::new(
                vec![id],
                SpacecraftPropagator::new(
                    ship.start,
                    StateVector::new(ship.position, ship.velocity),
                    DEFAULT_PARAMS,
                    Bodies(context),
                ),
            ),
            FlightPlan::new(ship.end, DEFAULT_PARAMS.n_max as usize, mapped_burns),
        ))
        .with_child((
            Name::new(ship.name.to_string()),
            PlotSource(id),
            PlotConfig {
                enabled: true,
                color: color.into(),
                start: ship.start - Duration::from_years(1000.0),
                end: ship.start + Duration::from_years(1000.0),
                threshold: 0.5,
                max_points: 10_000,
                reference: parent,
            },
        ));

    commands.entity(*root).add_child(id);
    commands.queue(AddOrbit {
        orbiting: parent.unwrap_or(*root),
        body: id,
    });

    commands.trigger_targets(FlightPlanChanged, id);
}

fn setup_camera(
    mut commands: Commands,
    skybox: Res<SkyboxHandle>,
    root: Single<Entity, With<SystemRoot>>,
) {
    commands
        .spawn((
            // No state scope needed as it is always a child of the root.
            bevy_egui::PrimaryEguiContext,
            Camera3d::default(),
            Camera {
                hdr: true,
                ..default()
            },
            Projection::Perspective(PerspectiveProjection {
                near: 0.001,
                ..default()
            }),
            bevy::render::camera::Exposure { ev100: 12.0 },
            bevy::core_pipeline::bloom::Bloom::NATURAL,
            Msaa::Sample4,
            Skybox {
                image: skybox.handle.clone(),
                brightness: 1000.0,
                rotation: Quat::IDENTITY,
            },
            FloatingOrigin,
            GridCell::default(),
            OrbitCamera::default().with_distance(5e4),
            BigSpaceCameraController::default()
                .with_speed_bounds([1e-17, 10e35])
                .with_smoothness(0.0, 0.0)
                .with_speed(1.0)
                .with_speed_pitch(0.02)
                .with_speed_yaw(0.02)
                .with_speed_roll(0.5),
            IsDefaultUiCamera,
        ))
        .insert(ChildOf(*root));
}

fn default_follow(mut commands: Commands, query: Query<(Option<Entity>, &Name)>) {
    for (entity, name) in &query {
        if name.contains("Earth") {
            commands.insert_resource(Followed(entity));
        }
    }
}

fn compute_ephemerides_bodies(mut commands: Commands) {
    let min_steps = 1000;

    commands.trigger(ExtendPredictionEvent::<NBodyPropagator<Forward>>::all(
        Duration::from_days(365.0 * 2.0),
        min_steps,
    ));
    commands.trigger(ExtendPredictionEvent::<NBodyPropagator<Backward>>::all(
        Duration::from_days(365.0 * 2.0),
        min_steps,
    ));
}

fn ephemerides_bodies_progress(
    mut state: ResMut<NextState<SpawnStage>>,
    mut query: Query<&mut Text, With<LoadingText>>,
    forward: Query<&PredictionTracker<NBodyPropagator<Forward>>>,
    backward: Query<&PredictionTracker<NBodyPropagator<Backward>>>,
) {
    if forward.is_empty() && backward.is_empty() {
        state.set(SpawnStage::Ships);
        return;
    }

    let forward_progress = forward
        .single()
        .map(PredictionTracker::progress)
        .unwrap_or(1.0);
    let backward_progress = backward
        .single()
        .map(PredictionTracker::progress)
        .unwrap_or(1.0);
    let progress = (forward_progress + backward_progress) / 2.0;
    for mut text in query.iter_mut() {
        **text = format!("Computing ephemerides (1/2): {:.0}%", progress * 100.0);
    }
}

fn ephemerides_ships_progress(
    mut state: ResMut<NextState<MainState>>,
    mut query: Query<&mut Text, With<LoadingText>>,
    query_tracker: Query<
        Option<&PredictionTracker<SpacecraftPropagator>>,
        With<PredictionContext<SpacecraftPropagator>>,
    >,
) {
    if query_tracker.iter().all(|t| t.is_none()) {
        state.set(MainState::Running);
        return;
    }

    let mut total_progress = 1.0f32;
    // Using flatten crashes the compiler randomly.
    #[expect(clippy::manual_flatten)]
    for tracker in query_tracker.iter() {
        if let Some(tracker) = tracker {
            total_progress = total_progress.min(tracker.progress());
        }
    }

    for mut text in query.iter_mut() {
        **text = format!(
            "Computing ephemerides (2/2): {:.0}%",
            total_progress * 100.0
        );
    }
}

#[allow(unused)]
fn bypass_ephemerides_loading(
    kb: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<NextState<MainState>>,
) {
    if kb.just_pressed(KeyCode::Escape) {
        state.set(MainState::Running);
    }
}

#[expect(unused)]
fn true_for_frames(frames: usize) -> impl Fn(Local<usize>) -> bool {
    move |mut completed: Local<usize>| {
        *completed += 1;
        *completed > frames
    }
}
