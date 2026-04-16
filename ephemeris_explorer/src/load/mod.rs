pub mod solar_system;
pub mod ui;

use integration::{AdaptiveMethodParams, ratio::Ratio};
pub use solar_system::*;
pub use ui::*;

use crate::{
    MainState,
    analysis::{OrbitPlotConfig, OrbitPlotReference, OrbitTarget, SoiTransitionsAnalysis},
    camera::{CameraController, CanFollow, Followed, OrbitCamera},
    dynamics::{
        AbsTol, Backward, Bodies, CelestialTrajectory, CubicHermiteSpline, Forward,
        GravitationalBody, LeastSquaresFit, Mu, NBodyPropagator, SpacecraftTrajectory,
        SphereOfInfluence, StateVector, Trajectory, UniformSpline,
    },
    flight_plan::{Burn, BurnFrame, FlightPlan, FlightPlanChanged, FlightPlanDependency},
    floating_origin::{BigGridBundle, BigSpaceRootBundle, CellCoord, FloatingOrigin},
    prediction::{
        ComputePrediction, PredictionController, PredictionControllerOf, PredictionPropagator,
        PredictionTracker, Synchronisation,
    },
    rotation::Rotating,
    selection::Selectable,
    simulation::{BoundsTime, SimulationTime},
    starlight::Star,
    ui::{Id, Labelled, PlotBound},
};

use bevy::prelude::*;
use bevy::{
    asset::RecursiveDependencyLoadState, core_pipeline::Skybox, platform::hash::FixedState,
};
use ephemeris::DIV;
use ftime::{Duration, Epoch};
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
            .add_message::<LoadSolarSystem>()
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
            "../../assets/fonts/Lexend-Regular.ttf",
            |bytes: &[u8], _: String| { Font::try_from_bytes(bytes.to_vec()).unwrap() }
        );
    }
}

#[derive(Message, Clone, Debug)]
pub struct LoadSolarSystem {
    pub path: std::path::PathBuf,
    pub state: Handle<SolarSystemState>,
    pub eph_settings: Handle<EphemeridesSettings>,
    pub skybox: Handle<SkyboxImage>,
    pub ships: Vec<Handle<Ship>>,
}

impl LoadSolarSystem {
    pub fn try_from_dir(
        dir: &std::path::Path,
        asset_server: &AssetServer,
    ) -> std::io::Result<Self> {
        Ok(Self {
            path: dir.to_path_buf(),
            state: asset_server.load(dir.join("state.json")),
            eph_settings: asset_server.load(dir.join("ephemeris.json")),
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
    mut events: MessageReader<LoadSolarSystem>,
    mut state: ResMut<NextState<MainState>>,
) {
    for event in events.read() {
        commands.insert_resource(UniqueAssetHandle(event.state.clone()));
        commands.insert_resource(UniqueAssetHandle(event.eph_settings.clone()));
        commands.insert_resource(UniqueAssetHandle(event.skybox.clone()));
        commands.insert_resource(ShipsHandles {
            handles: event.ships.clone(),
        });

        NextState::set_if_neq(&mut state, MainState::Loading);
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

fn skybox_loaded(skybox: UniqueAsset<SkyboxImage>) -> bool {
    matches!(
        skybox.recursive_dependency_load_state(),
        RecursiveDependencyLoadState::Loaded | RecursiveDependencyLoadState::Failed(_)
    )
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
    MissingInterpolationParameter(String),
    MissingReference { ship: String, reference: String },
}
impl std::fmt::Display for LoadingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AssetLoadError(err) => write!(f, "Asset load error: {err}"),
            Self::MissingBody(name) => write!(f, "Missing body in solar system state: {name}"),
            Self::MissingInterpolationParameter(name) => {
                write!(f, "Missing interpolation parameter for body: {name}")
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
    mut events: MessageReader<bevy::asset::UntypedAssetLoadFailedEvent>,
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
    ephemerides: UniqueAsset<EphemeridesSettings>,
    visuals: Res<Assets<BodyVisuals>>,
    mut errors: ResMut<LoadingErrors>,
) {
    // We can assume all assets are loaded at this point and unwrap.

    let empty = SolarSystemState::default();
    let system = solar_system.get().unwrap_or(&empty);

    let root = commands
        .spawn((
            DespawnOnExit(MainState::Running),
            Name::new(system.name.clone()),
            BigSpaceRootBundle::default(),
            SystemRoot,
            CanFollow {
                min_distance: 1e3,
                max_distance: 1e10,
            },
        ))
        .id();

    let empty = EphemeridesSettings::default();
    let ephemerides = ephemerides.get().unwrap_or(&empty);

    let mut sorted_bodies = system.bodies.iter().collect::<Vec<_>>();
    sorted_bodies.sort_by(|(_, a), (_, b)| b.mu.total_cmp(&a.mu));
    let mut info = indexmap::IndexMap::<_, (SphereOfInfluence, _)>::new();
    for &(name, body) in sorted_bodies.iter() {
        let (soi, depth) = sorted_bodies
            .iter()
            .zip(info.values())
            .filter(|((_, other_body), (soi, _))| {
                body.position.distance(other_body.position) < soi.radius
            })
            .map(|(&(_, other_body), (_, depth))| {
                (
                    SphereOfInfluence::approximate(
                        body.position.distance(other_body.position),
                        body.mu,
                        other_body.mu,
                    ),
                    depth + 1,
                )
            })
            .min_by(|(a, _), (b, _)| a.radius.total_cmp(&b.radius))
            .unwrap_or((SphereOfInfluence::INFINITY, 0));

        info.insert(name.clone(), (soi, depth));
    }

    let mut entities = vec![];
    let mut states = vec![];

    for (id, (name, body)) in system.bodies.iter().enumerate() {
        let interpolation = match ephemerides.interpolation.get(name) {
            Some(interpolation) => interpolation,
            None => {
                errors.push(LoadingError::MissingInterpolationParameter(name.clone()));
                continue;
            }
        };
        let visual = visuals.get(&body.visuals).unwrap();

        let radius = ((visual.radii.x + visual.radii.y + visual.radii.z) / 3.0) as f32;
        let (soi, depth) = info.swap_remove(name).unwrap();

        let sample_period = ephemerides.dt * interpolation.count as f64;

        let mut entity = commands.spawn((
            // No state scope needed as it is always a child of the root.
            Name::new(name.clone()),
            Id(id as u64),
            Visibility::default(),
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
            (
                Mu(body.mu),
                Trajectory::from(UniformSpline::new(system.epoch, sample_period * DIV as f64)),
                PredictionController::<CelestialTrajectory<Forward>>::new(root),
                PredictionController::<CelestialTrajectory<Backward>>::new(root),
            ),
            BoundsTime,
            FlightPlanDependency,
            soi,
            SoiTransitionsAnalysis::Static,
            OrbitPlotConfig {
                enabled: depth <= 1,
                start: Epoch::MIN,
                end: Epoch::MAX,
                color: visual.orbit.color,
                bound: PlotBound::Start,
                resolution: 0.5,
                max_points_per_segment: 10_000,
                reference: OrbitPlotReference::Primary,
            },
        ));
        entity.with_children(|cmds| {
            let mut child = cmds.spawn((
                Transform::from_scale(visual.radii.as_vec3() / radius),
                Visibility::default(),
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
                    bevy::light::NotShadowCaster,
                ));
            }
        });

        entities.push(entity.id());
        states.push((
            StateVector::new(body.position, body.velocity),
            body.mu,
            sample_period,
            LeastSquaresFit {
                degree: interpolation.degree,
            },
        ));

        let entity = entity.id();
        commands.entity(root).add_child(entity);
    }

    let dt = ephemerides.dt;
    commands.entity(root).insert((
        PredictionPropagator::<CelestialTrajectory<Forward>>(NBodyPropagator::with(
            Forward::new(dt),
            system.epoch,
            states.clone(),
        )),
        PredictionControllerOf::<CelestialTrajectory<Forward>>::new(entities.clone()),
        PredictionPropagator::<CelestialTrajectory<Backward>>(NBodyPropagator::with(
            Backward::new(dt),
            system.epoch,
            states,
        )),
        PredictionControllerOf::<CelestialTrajectory<Backward>>::new(entities),
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
        commands.trigger(SpawnShip {
            ship: ship.clone(),
            entity: None,
        });
    }
}

#[derive(Event)]
pub struct SpawnShip {
    pub ship: Ship,
    pub entity: Option<Entity>,
}

impl SpawnShip {
    #[inline]
    pub fn new(ship: Ship) -> Self {
        Self { ship, entity: None }
    }

    #[inline]
    pub fn with_entity(mut self, entity: Entity) -> Self {
        self.entity = Some(entity);
        self
    }
}

pub fn find_by_name(query: &Query<(Entity, &Name), With<Mu>>, reference: &str) -> Option<Entity> {
    query
        .iter()
        .find(|(_, name)| name.as_str() == reference)
        .map(|(entity, _)| entity)
}

const fn ratio_f64(ratio: Ratio<u16>) -> f64 {
    ratio.numerator() as f64 / ratio.denominator() as f64
}

pub const INITIAL_ADAPTIVE_PARAMS: AdaptiveMethodParams<f64, AbsTol, f64> =
    AdaptiveMethodParams::with(
        60.0,
        f64::MAX,
        // Tolerance of 1 m and 1 m/s.
        AbsTol {
            position: 1e-3,
            velocity: 1e-3,
        },
        // Const workaround instead of using `AdaptiveMethodParams::new`.
        ratio_f64(integration::DEFAULT_FAC_MIN),
        ratio_f64(integration::DEFAULT_FAC_MAX),
        ratio_f64(integration::DEFAULT_FAC),
        1_000_000,
    );

fn spawn_ship(
    trigger: On<SpawnShip>,
    mut commands: Commands,
    query_name: Query<(Entity, &Name), With<Mu>>,
    query_context: Query<(Entity, &Trajectory, &Mu, &SphereOfInfluence)>,
    query_ids: Query<&Id>,
    root: Single<Entity, With<SystemRoot>>,
    mut errors: ResMut<LoadingErrors>,
) {
    let ship = &trigger.event().ship;

    let radius = 0.01;

    let bodies = Bodies(
        query_context
            .iter()
            .map(|(e, traj, mu, soi)| (e, GravitationalBody::new(traj.clone(), *mu, *soi)))
            .collect(),
    );

    let mut entity = match trigger.event().entity {
        Some(entity) => commands.entity(entity),
        None => commands.spawn_empty(),
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
    )
    .into();

    let flight_plan = FlightPlan::new(
        ship.end,
        ship.integrator,
        AdaptiveMethodParams {
            tol: AbsTol {
                position: ship.tolerance,
                velocity: ship.tolerance,
            },
            ..INITIAL_ADAPTIVE_PARAMS
        },
        mapped_burns,
        bodies,
    );

    entity.insert((
        // No state scope needed as this is always a child of the root.
        Name::new(ship.name.to_string()),
        Id(query_ids.iter().map(|id| id.0 + 1).max().unwrap_or(0)),
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
        Trajectory::from(CubicHermiteSpline::new(
            ship.start,
            ship.position,
            ship.velocity,
        )),
        PredictionPropagator::<SpacecraftTrajectory>(
            flight_plan
                .build_propagator_at(ship.start, StateVector::new(ship.position, ship.velocity)),
        ),
        PredictionController::<SpacecraftTrajectory>::new(id),
        PredictionControllerOf::<SpacecraftTrajectory>::new(vec![id]),
        flight_plan,
        SoiTransitionsAnalysis::Dynamic,
        OrbitPlotConfig {
            enabled: true,
            start: Epoch::MIN,
            end: Epoch::MAX,
            color,
            bound: PlotBound::Start,
            resolution: 0.5,
            max_points_per_segment: 10_000,
            reference: OrbitPlotReference::Primary,
        },
        OrbitTarget(None),
    ));

    commands.entity(*root).add_child(id);

    commands.trigger(FlightPlanChanged(id));
}

fn setup_camera(
    mut commands: Commands,
    skybox: UniqueAsset<SkyboxImage>,
    root: Single<Entity, With<SystemRoot>>,
) {
    commands
        .spawn((
            // No state scope needed as this is always a child of the root.
            bevy_egui::PrimaryEguiContext,
            Camera3d::default(),
            Camera {
                order: 1,
                ..default()
            },
            Projection::Perspective(PerspectiveProjection {
                near: 0.001,
                ..default()
            }),
            bevy::camera::Exposure { ev100: 12.0 },
            bevy::post_process::bloom::Bloom::NATURAL,
            Msaa::Sample4,
            Skybox {
                image: skybox.get().unwrap().0.clone(),
                brightness: 1000.0,
                rotation: Quat::IDENTITY,
            },
            FloatingOrigin,
            CellCoord::default(),
            OrbitCamera::default().with_distance(5e4),
            CameraController::default()
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

fn default_follow(mut commands: Commands, query: Query<(Entity, &Name)>) {
    for (entity, name) in &query {
        if name.as_str() == "Earth" {
            commands.insert_resource(Followed(Some(entity)));
        }
    }
}

fn compute_ephemerides_bodies(mut commands: Commands, root: Single<Entity, With<SystemRoot>>) {
    let duration = Duration::from_days(365.0 * 2.0);
    let synchronisation = Synchronisation::hertz(100);

    commands.trigger(ComputePrediction::<CelestialTrajectory<Forward>>::extend(
        *root,
        duration,
        synchronisation,
    ));
    commands.trigger(ComputePrediction::<CelestialTrajectory<Backward>>::extend(
        *root,
        duration,
        synchronisation,
    ));
}

fn ephemerides_bodies_progress(
    mut state: ResMut<NextState<SpawnStage>>,
    mut query: Query<&mut Text, With<LoadingText>>,
    forward: Query<&PredictionTracker<CelestialTrajectory<Forward>>>,
    backward: Query<&PredictionTracker<CelestialTrajectory<Backward>>>,
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
        Option<&PredictionTracker<SpacecraftTrajectory>>,
        With<PredictionPropagator<SpacecraftTrajectory>>,
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
