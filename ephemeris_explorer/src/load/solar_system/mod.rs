pub mod loaders;

pub use loaders::*;

use crate::{MainState, flight_plan::IntegrationMethod};

use bevy::math::DVec3;
use bevy::prelude::*;
use ftime::{Duration, Epoch};

#[derive(Resource)]
pub struct UniqueAssetHandle<T: Asset>(pub Handle<T>);

#[derive(bevy::ecs::system::SystemParam)]
pub struct UniqueAsset<'w, T: Asset> {
    handle: Res<'w, UniqueAssetHandle<T>>,
    server: Res<'w, AssetServer>,
    assets: Res<'w, Assets<T>>,
}

impl<T: Asset> UniqueAsset<'_, T> {
    pub fn handle(&self) -> &Handle<T> {
        &self.handle.0
    }

    pub fn get(&self) -> Option<&T> {
        self.assets.get(&self.handle.0)
    }

    pub fn load_state(&self) -> bevy::asset::LoadState {
        self.server.load_state(&self.handle.0)
    }

    pub fn recursive_dependency_load_state(&self) -> bevy::asset::RecursiveDependencyLoadState {
        self.server.recursive_dependency_load_state(&self.handle.0)
    }
}

#[derive(bevy::ecs::system::SystemParam)]
pub struct UniqueAssetMut<'w, T: Asset> {
    handle: Res<'w, UniqueAssetHandle<T>>,
    server: Res<'w, AssetServer>,
    assets: ResMut<'w, Assets<T>>,
}

impl<T: Asset> UniqueAssetMut<'_, T> {
    pub fn get(&self) -> Option<&T> {
        self.assets.get(&self.handle.0)
    }

    pub fn get_mut(&mut self) -> Option<&mut T> {
        self.assets.get_mut(&self.handle.0)
    }

    pub fn load_state(&self) -> bevy::asset::LoadState {
        self.server.load_state(&self.handle.0)
    }

    pub fn recursive_dependency_load_state(&self) -> bevy::asset::RecursiveDependencyLoadState {
        self.server.recursive_dependency_load_state(&self.handle.0)
    }
}

pub struct LoadSolarSystemPlugin;

impl Plugin for LoadSolarSystemPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<BodyVisuals>()
            .init_asset::<SolarSystemState>()
            .init_asset::<EphemeridesSettings>()
            .init_asset::<Ship>()
            .init_asset::<SkyboxImage>()
            .register_asset_loader(BodyVisualsLoader)
            .register_asset_loader(SolarSystemStateLoader)
            .register_asset_loader(EphemeridesSettingsLoader)
            .register_asset_loader(ShipLoader)
            .register_asset_loader(SkyboxLoader(bevy::image::ImageLoader::new(
                bevy::image::CompressedImageFormats::NONE,
            )))
            .insert_resource(ClearColor(Color::BLACK))
            .insert_resource(GlobalAmbientLight {
                color: Color::NONE,
                brightness: 0.0,
                affects_lightmapped_meshes: true,
            })
            .add_systems(
                PreUpdate,
                handle_missing_visuals.run_if(in_state(MainState::Loading)),
            );

        let mut default_visuals = BodyVisuals::default();
        default_visuals.mesh = app
            .world_mut()
            .resource_mut::<AssetServer>()
            .add(Mesh::from(Sphere {
                radius: default_visuals.radii.x as f32,
            }));

        _ = app
            .world_mut()
            .resource_mut::<Assets<BodyVisuals>>()
            .insert(Handle::default().id(), default_visuals);
    }
}

#[derive(Debug, Resource)]
pub struct ShipsHandles {
    pub handles: Vec<Handle<Ship>>,
}

#[derive(Debug)]
pub struct OrbitVisuals {
    pub color: Color,
}

impl Default for OrbitVisuals {
    fn default() -> Self {
        OrbitVisuals {
            color: Color::Srgba(Srgba {
                red: 0.3,
                green: 0.3,
                blue: 0.3,
                alpha: 1.0,
            }),
        }
    }
}

#[derive(Asset, TypePath, Debug)]
pub struct BodyVisuals {
    pub radii: DVec3,
    pub material: Handle<StandardMaterial>,
    pub mesh: Handle<Mesh>,
    pub light: Option<Color>,
    pub orbit: OrbitVisuals,
    pub right_ascension: f64,
    pub declination: f64,
    pub rotation_reference_epoch: Epoch,
    pub rotation_reference: f64,
    pub rotation_rate: f64,
}

impl Default for BodyVisuals {
    fn default() -> Self {
        BodyVisuals {
            radii: DVec3::splat(100.0),
            material: Handle::default(),
            mesh: Handle::default(),
            light: None,
            orbit: OrbitVisuals::default(),
            right_ascension: 0.0,
            declination: 0.0,
            rotation_reference_epoch: Epoch::default(),
            rotation_reference: 0.0,
            rotation_rate: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Body {
    pub mu: f64,
    pub position: DVec3,
    pub velocity: DVec3,
    pub visuals: Handle<BodyVisuals>,
}

#[derive(Debug, Default, Asset, TypePath)]
pub struct SolarSystemState {
    pub name: String,
    pub bodies: indexmap::IndexMap<String, Body>,
    pub epoch: Epoch,
}

fn handle_missing_visuals(
    asset_server: Res<AssetServer>,
    mut solar_system: UniqueAssetMut<SolarSystemState>,
    mut events: MessageWriter<AssetEvent<BodyVisuals>>,
) {
    if let Some(solar_system) = solar_system.get_mut() {
        for body in solar_system.bodies.values_mut() {
            if matches!(
                asset_server.load_state(&body.visuals),
                bevy::asset::LoadState::Failed(_)
            ) {
                body.visuals = Handle::default();
                events.write(AssetEvent::LoadedWithDependencies {
                    id: body.visuals.id(),
                });
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct InterpolationParameters {
    pub degree: usize,
    pub count: usize,
}

#[derive(Debug, Default, Asset, TypePath)]
pub struct EphemeridesSettings {
    pub dt: Duration,
    pub interpolation:
        bevy::platform::collections::hash_map::HashMap<String, InterpolationParameters>,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct BurnDe {
    pub start: Epoch,
    pub duration: Duration,
    pub acceleration: DVec3,
    pub reference: Option<String>,
}

#[derive(Clone, Asset, TypePath, Debug, serde::Deserialize)]
pub struct Ship {
    pub name: String,
    pub integrator: IntegrationMethod,
    pub tolerance: f64,
    pub start: Epoch,
    pub end: Epoch,
    pub position: DVec3,
    pub velocity: DVec3,
    pub burns: Vec<BurnDe>,
}

impl Ship {
    pub fn new(
        name: String,
        integrator: IntegrationMethod,
        tolerance: f64,
        start: Epoch,
        end: Epoch,
        position: DVec3,
        velocity: DVec3,
        burns: Vec<BurnDe>,
    ) -> Self {
        Ship {
            name,
            integrator,
            tolerance,
            start,
            end,
            position,
            velocity,
            burns,
        }
    }
}

#[derive(Clone, Asset, TypePath, Debug)]
pub struct SkyboxImage(pub Handle<Image>);

/// All the bodies in a gravitationally bound system should be a child of a root entity with this
/// component.
#[derive(Component)]
pub struct SystemRoot;
