pub mod loaders;

pub use loaders::*;

use crate::MainState;

use bevy::prelude::*;
use bevy::{asset::LoadedFolder, math::DVec3};
use hifitime::{Duration, Epoch};

#[derive(Resource)]
pub struct UniqueAssetHandle<T: Asset>(pub Handle<T>);

#[derive(bevy::ecs::system::SystemParam)]
pub struct UniqueAsset<'w, T: Asset> {
    handle: Res<'w, UniqueAssetHandle<T>>,
    server: Res<'w, AssetServer>,
    assets: Res<'w, Assets<T>>,
}

impl<T: Asset> UniqueAsset<'_, T> {
    pub fn get(&self) -> Option<&T> {
        self.assets.get(&self.handle.0)
    }

    #[expect(unused)]
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
    #[expect(unused)]
    pub fn get(&self) -> Option<&T> {
        self.assets.get(&self.handle.0)
    }

    pub fn get_mut(&mut self) -> Option<&mut T> {
        self.assets.get_mut(&self.handle.0)
    }

    #[expect(unused)]
    pub fn load_state(&self) -> bevy::asset::LoadState {
        self.server.load_state(&self.handle.0)
    }

    #[expect(unused)]
    pub fn recursive_dependency_load_state(&self) -> bevy::asset::RecursiveDependencyLoadState {
        self.server.recursive_dependency_load_state(&self.handle.0)
    }
}

pub struct LoadSolarSytemPlugin;

impl Plugin for LoadSolarSytemPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<BodyVisuals>()
            .init_asset::<SolarSystem>()
            .init_asset::<HierarchyTree>()
            .init_asset::<EphemeridesSettings>()
            .init_asset::<Ship>()
            .register_asset_loader(BodyVisualsLoader)
            .register_asset_loader(SolarSystemLoader)
            .register_asset_loader(HierarchyTreeLoader)
            .register_asset_loader(EphemeridesSettingsLoader)
            .register_asset_loader(ShipLoader)
            .insert_resource(ClearColor(Color::BLACK))
            .insert_resource(AmbientLight {
                color: Color::NONE,
                brightness: 0.0,
            })
            .add_systems(
                PreUpdate,
                (reconfigure_skybox_image, handle_missing_visuals)
                    .run_if(in_state(MainState::Loading)),
            );

        let mut default_visuals = BodyVisuals::default();
        default_visuals.mesh = app
            .world_mut()
            .resource_mut::<AssetServer>()
            .add(Mesh::from(Sphere {
                radius: default_visuals.radii.x as f32,
            }));

        app.world_mut()
            .resource_mut::<Assets<BodyVisuals>>()
            .insert(Handle::default().id(), default_visuals);
    }
}

#[derive(Resource)]
pub struct SkyboxHandle {
    pub reconfigured: bool,
    pub handle: Handle<Image>,
}

fn reconfigure_skybox_image(mut images: ResMut<Assets<Image>>, mut skybox: ResMut<SkyboxHandle>) {
    if skybox.reconfigured {
        return;
    }

    if let Some(image) = images.get_mut(&skybox.handle) {
        // NOTE: PNGs do not have any metadata that could indicate they contain a cubemap texture,
        // so they appear as one texture. The following code reconfigures the texture as necessary.
        if image.texture_descriptor.array_layer_count() == 1 {
            image.reinterpret_stacked_2d_as_array(image.height() / image.width());
            image.texture_view_descriptor =
                Some(bevy::render::render_resource::TextureViewDescriptor {
                    dimension: Some(bevy::render::render_resource::TextureViewDimension::Cube),
                    ..default()
                });
        }

        skybox.reconfigured = true;
    }
}

#[derive(Debug, Resource)]
pub struct ShipsFolderHandle {
    pub handle: Handle<LoadedFolder>,
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

#[derive(Debug)]
pub struct Body {
    pub mu: f64,
    pub position: DVec3,
    pub velocity: DVec3,
    pub visuals: Handle<BodyVisuals>,
}

#[derive(Asset, TypePath, Debug)]
pub struct SolarSystem {
    pub bodies: bevy::utils::HashMap<String, Body>,
    pub epoch: Epoch,
}

fn handle_missing_visuals(
    asset_server: Res<AssetServer>,
    mut solar_system: UniqueAssetMut<SolarSystem>,
    mut writer: EventWriter<AssetEvent<BodyVisuals>>,
) {
    if let Some(solar_system) = solar_system.get_mut() {
        for body in solar_system.bodies.values_mut() {
            if matches!(
                asset_server.load_state(&body.visuals),
                bevy::asset::LoadState::Failed(_)
            ) {
                body.visuals = Handle::default();
                writer.send(AssetEvent::LoadedWithDependencies {
                    id: body.visuals.id(),
                });
            }
        }
    }
}

#[derive(Debug)]
pub struct EphemerisSettings {
    pub degree: usize,
    pub count: usize,
}

#[derive(Asset, TypePath, Debug)]
pub struct EphemeridesSettings {
    pub dt: Duration,
    pub settings: bevy::utils::HashMap<String, EphemerisSettings>,
}

#[derive(Asset, TypePath, Deref, DerefMut, Debug, serde::Deserialize)]
pub struct HierarchyTree(pub indexmap::IndexMap<String, HierarchyTree>);

#[derive(Clone, Debug, serde::Deserialize)]
pub struct Burn {
    pub start: Epoch,
    pub duration: Duration,
    pub acceleration: DVec3,
    pub reference: Option<String>,
}

#[derive(Clone, Asset, TypePath, Debug, serde::Deserialize)]
pub struct Ship {
    pub name: String,
    pub start: Epoch,
    pub position: DVec3,
    pub velocity: DVec3,
    pub burns: Vec<Burn>,
}

/// All the bodies in a stellar system, planetary system or satellite system should be a child of a
/// root entity with this component.
#[derive(Component)]
pub struct SystemRoot;
