pub mod solar_system;
pub mod ui;

pub use solar_system::*;
pub use ui::*;

use crate::{
    camera::{CanFollow, OrbitCamera},
    ephemerides::{EphemeridesComputeSettings, EphemerisBuilder, Trajectory, MAX_DIV},
    floating_origin::{
        BigReferenceFrameBundle, BigSpace, BigSpaceRootBundle, FloatingOrigin, GridCell,
    },
    hierarchy,
    plot::EphemerisPlot,
    selection::{Clickable, Followed},
    starlight::Star,
    time::EphemeridesTime,
    ui::{Labelled, OrbitalPeriod},
    GameState, SystemRoot,
};

use bevy::asset::RecursiveDependencyLoadState;
use bevy::core_pipeline::Skybox;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};

pub struct LoadingPlugin;

impl Plugin for LoadingPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(LoadSolarSytemPlugin)
            .add_event::<LoadSolarSystemEvent>()
            .add_systems(
                First,
                (
                    handle_load_events,
                    finish_loading.run_if(
                        in_state(GameState::Loading)
                            .and_then(bodies_loaded)
                            .and_then(eph_settings_loaded)
                            .and_then(hierarchy_loaded)
                            .and_then(skybox_loaded),
                    ),
                )
                    .chain(),
            )
            .add_systems(
                OnEnter(GameState::Running),
                (spawn_bodies, setup_camera, default_follow).chain(),
            );
        app.add_plugins(LoadingScreenPlugin).add_systems(
            Update,
            update_loading_text::<Image>.run_if(in_state(GameState::Loading)),
        );
    }
}

#[derive(Event, Clone)]
pub struct LoadSolarSystemEvent {
    pub state: Handle<SolarSystem>,
    pub eph_settings: Handle<EphemeridesSettings>,
    pub hierarchy_tree: Handle<HierarchyTree>,
    pub skybox: Handle<Image>,
}

impl LoadSolarSystemEvent {
    pub fn try_from_dir(
        dir: &std::path::Path,
        asset_server: &AssetServer,
    ) -> std::io::Result<Self> {
        let state_file = dir.join("state.json");
        let eph_settings_file = dir.join("ephemeris.json");
        let hierarchy_tree_file = dir.join("hierarchy.json");
        let skybox_file = dir.join("skybox.png");

        // We can't really check if the files exist for now.
        // Hopefully in the future we can use the asset server to check if the files exist.

        Ok(Self {
            state: asset_server.load(state_file),
            eph_settings: asset_server.load(eph_settings_file),
            hierarchy_tree: asset_server.load(hierarchy_tree_file),
            skybox: asset_server.load(skybox_file),
        })
    }
}

pub fn handle_load_events(
    mut commands: Commands,
    mut events: EventReader<LoadSolarSystemEvent>,
    mut next_state: ResMut<NextState<GameState>>,
) {
    for event in events.read() {
        commands.insert_resource(UniqueAssetHandle(event.state.clone()));
        commands.insert_resource(UniqueAssetHandle(event.eph_settings.clone()));
        commands.insert_resource(UniqueAssetHandle(event.hierarchy_tree.clone()));
        commands.insert_resource(SkyboxHandle {
            reconfigured: false,
            handle: event.skybox.clone(),
        });

        next_state.set(GameState::Loading);
    }
}

fn finish_loading(world: &mut World) {
    world
        .resource_mut::<NextState<GameState>>()
        .set(GameState::Running);

    world.run_schedule(StateTransition)
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
    let solar_system = solar_system.get().unwrap();
    let hierachy_tree = hierachy_tree.get().unwrap();
    let eph_settings = eph_settings.get().unwrap();

    // Hierarchy should always have one root element.
    let (name, tree) = hierachy_tree.first().unwrap();
    let root = commands
        .spawn((
            StateScoped(GameState::Running),
            Name::new(name.to_string()),
            BigSpaceRootBundle::default(),
            SystemRoot,
            CanFollow {
                min_distance: 1e3,
                max_distance: 1e10,
            },
        ))
        .id();

    spawn_from_hierarchy(
        0,
        root,
        &mut commands,
        root,
        tree,
        eph_settings,
        solar_system,
        &visuals,
    );

    #[expect(clippy::too_many_arguments)]
    fn spawn_from_hierarchy(
        depth: usize,
        root: Entity,
        commands: &mut Commands,
        parent: Entity,
        tree: &HierarchyTree,
        eph_settings: &EphemeridesSettings,
        solar_system: &SolarSystem,
        visuals: &Assets<BodyVisuals>,
    ) {
        for (name, tree) in tree.iter() {
            let body = solar_system.bodies.get(name).unwrap();
            let ephemeris_settings = eph_settings.settings.get(name).unwrap();
            let visual = visuals.get(&body.visuals).unwrap();

            let radius = ((visual.radii.x + visual.radii.y + visual.radii.z) / 3.0) as f32;
            let granule = eph_settings.granule(name, MAX_DIV - 1);

            // println!("{}: {}", name, granule);

            let mut entity = commands.spawn((
                // No state scope needed as it is always a child of the root.
                Name::new(name.clone()),
                BigReferenceFrameBundle {
                    transform: Transform::from_translation(body.position.as_vec3()),
                    ..default()
                },
                Clickable { radius },
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
                OrbitalPeriod(Some(Duration::default())),
                Trajectory::new(solar_system.epoch, granule),
                EphemerisBuilder::new(
                    ephemeris_settings.degree,
                    body.position,
                    body.velocity,
                    body.mu,
                ),
                EphemerisPlot {
                    enabled: depth <= 1,
                    color: visual.orbit.color,
                    start: Epoch::default(),
                    end: Epoch::default() + Duration::MAX,
                    threshold: 0.5,
                    reference: Some(parent),
                },
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
                child,
                tree,
                eph_settings,
                solar_system,
                visuals,
            );
        }
    }

    commands.insert_resource(EphemeridesTime::new(solar_system.epoch));
    commands.insert_resource(EphemeridesComputeSettings {
        dt: eph_settings.dt,
    });
}

fn setup_camera(
    mut commands: Commands,
    skybox: Res<SkyboxHandle>,
    root: Query<Entity, With<BigSpace>>,
) {
    let root = root.single();
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
                    ..default()
                }),
                exposure: bevy::render::camera::Exposure { ev100: 12.0 },
                ..default()
            },
            Skybox {
                image: skybox.handle.clone(),
                brightness: 1000.0,
            },
            FloatingOrigin,
            GridCell::default(),
            OrbitCamera::default().with_distance(2e4),
            big_space::camera::CameraController::default()
                .with_speed_bounds([1e-17, 10e35])
                .with_smoothness(0.0, 0.0)
                .with_speed(1.0),
            bevy::core_pipeline::bloom::BloomSettings::NATURAL,
        ))
        .set_parent(root);
}

fn default_follow(mut commands: Commands, query: Query<(Option<Entity>, &Name)>) {
    for (entity, name) in &query {
        if name.contains("Earth") {
            commands.insert_resource(Followed(entity));
        }
    }
}
