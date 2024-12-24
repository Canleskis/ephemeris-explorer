use super::{
    Body, BodyVisuals, EphemeridesSettings, EphemerisSettings, HierarchyTree, OrbitVisuals, Ship,
    SolarSystem,
};

use bevy::asset::{AssetPath, AsyncReadExt};
use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};
use serde::Deserialize;
use thiserror::Error;

// pub const DISTANCE_SCALE: f64 = 1e-3; // Meters
pub const DISTANCE_SCALE: f64 = 1.0; // Kilometers

pub struct BodyVisualsLoader;

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum BodyLoaderError {
    /// An [IO Error](std::io::Error)
    #[error("Could not read the file: {0}")]
    Io(#[from] std::io::Error),
    /// A [conversion Error](std::str::Utf8Error)
    #[error("Could not interpret as UTF-8: {0}")]
    FormatError(#[from] std::str::Utf8Error),
    /// A [TOML Error](serde_toml::de::Error)
    #[error("Could not parse TOML: {0}")]
    TomlError(#[from] toml::de::Error),
}

impl bevy::asset::AssetLoader for BodyVisualsLoader {
    type Asset = BodyVisuals;

    type Settings = ();

    type Error = BodyLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut bevy::asset::io::Reader,
        _: &'a (),
        ctx: &'a mut bevy::asset::LoadContext,
    ) -> impl bevy::utils::ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        #[derive(Deserialize, Clone)]
        #[serde(default)]
        struct PhysicalToml {
            radii: DVec3,
            right_ascension: f64,
            declination: f64,
            rotation_reference_epoch: Epoch,
            rotation_reference: f64,
            rotation_rate: f64,
        }

        impl Default for PhysicalToml {
            fn default() -> Self {
                PhysicalToml {
                    radii: DVec3::splat(100.0),
                    right_ascension: 0.0,
                    declination: 0.0,
                    rotation_reference_epoch: Epoch::from_gregorian_hms(
                        2000,
                        1,
                        1,
                        12,
                        0,
                        0,
                        hifitime::TimeScale::TDB,
                    ),
                    rotation_reference: 0.0,
                    rotation_rate: 0.0,
                }
            }
        }

        #[derive(Deserialize, Clone)]
        #[serde(default)]
        struct MaterialToml {
            base_color: String,
            base_color_texture: Option<AssetPath<'static>>,
            emissive: String,
            emissive_intensity: f32,
            emissive_texture: Option<AssetPath<'static>>,
            perceptual_roughness: f32,
            metallic_roughness_texture: Option<AssetPath<'static>>,
        }

        impl Default for MaterialToml {
            fn default() -> Self {
                MaterialToml {
                    base_color: "#ffffff".to_owned(),
                    base_color_texture: None,
                    emissive: "#00000000".to_owned(),
                    emissive_intensity: 1.0,
                    emissive_texture: None,
                    perceptual_roughness: 0.5,
                    metallic_roughness_texture: None,
                }
            }
        }

        #[derive(Deserialize, Asset, TypePath, Debug)]
        struct OrbitVisualsToml {
            color: String,
        }

        impl Default for OrbitVisualsToml {
            fn default() -> Self {
                OrbitVisualsToml {
                    color: "#ffffff".to_owned(),
                }
            }
        }

        #[derive(Deserialize, Asset, TypePath, Debug)]
        struct LightToml {
            color: String,
        }

        #[derive(Deserialize, Default)]
        #[serde(default)]
        struct BodyVisualsToml {
            physical: PhysicalToml,
            material: MaterialToml,
            orbit: OrbitVisualsToml,
            light: Option<LightToml>,
        }

        async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;
            let toml = toml::from_str::<BodyVisualsToml>(std::str::from_utf8(&bytes)?)?;

            let material = StandardMaterial {
                base_color: Srgba::hex(toml.material.base_color).unwrap().into(),
                base_color_texture: toml.material.base_color_texture.map(|p| ctx.load(p)),
                emissive: (Srgba::hex(toml.material.emissive).unwrap()
                    * toml.material.emissive_intensity)
                    .into(),
                emissive_texture: toml.material.emissive_texture.map(|p| ctx.load(p)),
                perceptual_roughness: toml.material.perceptual_roughness,
                metallic_roughness_texture: toml
                    .material
                    .metallic_roughness_texture
                    .map(|p| ctx.load(p)),
                alpha_mode: AlphaMode::Opaque,
                ..default()
            };
            let material = ctx.add_labeled_asset("material".to_owned(), material);

            let radii = toml.physical.radii * DISTANCE_SCALE;
            let mesh = bevy::render::mesh::SphereMeshBuilder {
                sphere: Sphere {
                    radius: ((radii.x + radii.y + radii.z) / 3.0) as _,
                },
                kind: bevy::render::mesh::SphereKind::Uv {
                    sectors: 144,
                    stacks: 72,
                },
            }
            .build();
            let mesh = ctx.add_labeled_asset("mesh".to_owned(), mesh);

            let light = toml
                .light
                .map(|light| Srgba::hex(light.color).unwrap().into());

            let orbit = OrbitVisuals {
                color: Srgba::hex(toml.orbit.color).unwrap().into(),
            };

            Ok(BodyVisuals {
                radii,
                material,
                mesh,
                light,
                orbit,
                right_ascension: toml.physical.right_ascension,
                declination: toml.physical.declination,
                rotation_reference_epoch: toml.physical.rotation_reference_epoch,
                rotation_reference: toml.physical.rotation_reference,
                rotation_rate: toml.physical.rotation_rate,
            })
        }
    }

    fn extensions(&self) -> &[&str] {
        &["visuals.toml"]
    }
}

pub struct SolarSystemLoader;

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum SolarSystemLoaderError {
    /// An [IO Error](std::io::Error)
    #[error("Could not read the file: {0}")]
    Io(#[from] std::io::Error),
    /// A [JSON Error](serde_json::error::Error)
    #[error("Could not parse the JSON: {0}")]
    JsonError(#[from] serde_json::error::Error),
}

impl bevy::asset::AssetLoader for SolarSystemLoader {
    type Asset = SolarSystem;

    type Settings = ();

    type Error = SolarSystemLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut bevy::asset::io::Reader,
        _: &'a (),
        ctx: &'a mut bevy::asset::LoadContext,
    ) -> impl bevy::utils::ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        #[derive(Deserialize)]
        pub struct BodyJson {
            pub name: String,
            pub mu: f64,
            pub position: DVec3,
            pub velocity: DVec3,
        }

        #[derive(Deserialize)]
        pub struct SolarSytemJson {
            epoch: Epoch,
            pub bodies: Vec<BodyJson>,
        }

        async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;
            let json = serde_json::from_slice::<SolarSytemJson>(&bytes)?;

            let mut bodies = bevy::utils::HashMap::with_capacity(json.bodies.len());

            for body in json.bodies {
                let label = body.name.to_lowercase();

                bodies.insert(
                    body.name,
                    Body {
                        mu: body.mu * DISTANCE_SCALE * DISTANCE_SCALE * DISTANCE_SCALE,
                        position: body.position * DISTANCE_SCALE,
                        velocity: body.velocity * DISTANCE_SCALE,
                        visuals: ctx.load(format!("visuals/{}.visuals.toml", &label)),
                    },
                );
            }

            Ok(SolarSystem {
                bodies,
                epoch: json.epoch,
            })
        }
    }

    fn extensions(&self) -> &[&str] {
        &["json"]
    }
}

pub struct EphemeridesSettingsLoader;

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum EphemeridesSettingsLoaderError {
    /// An [IO Error](std::io::Error)
    #[error("Could not read the file: {0}")]
    Io(#[from] std::io::Error),
    /// A [JSON Error](serde_json::error::Error)
    #[error("Could not parse the JSON: {0}")]
    JsonError(#[from] serde_json::error::Error),
}

impl bevy::asset::AssetLoader for EphemeridesSettingsLoader {
    type Asset = EphemeridesSettings;

    type Settings = ();

    type Error = EphemeridesSettingsLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut bevy::asset::io::Reader,
        _: &'a (),
        _: &'a mut bevy::asset::LoadContext,
    ) -> impl bevy::utils::ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        #[derive(Deserialize)]
        struct EphemerisSettingsJson {
            pub degree: usize,
            pub count: usize,
        }

        #[derive(Deserialize)]
        struct EphemeridesSettingsJson {
            dt: Duration,
            settings: bevy::utils::HashMap<String, EphemerisSettingsJson>,
        }

        async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;
            let json = serde_json::from_slice::<EphemeridesSettingsJson>(&bytes)?;

            let settings = EphemeridesSettings {
                dt: json.dt,
                settings: json
                    .settings
                    .into_iter()
                    .map(|(name, setting)| {
                        (
                            name,
                            EphemerisSettings {
                                degree: setting.degree,
                                count: setting.count,
                            },
                        )
                    })
                    .collect(),
            };

            Ok(settings)
        }
    }

    fn extensions(&self) -> &[&str] {
        &["toml"]
    }
}

pub struct HierarchyTreeLoader;

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum HierarchyTreeLoaderError {
    /// An [IO Error](std::io::Error)
    #[error("Could not read the file: {0}")]
    Io(#[from] std::io::Error),
    /// A [JSON Error](serde_json::error::Error)
    #[error("Could not parse the JSON: {0}")]
    JsonError(#[from] serde_json::error::Error),
}

impl bevy::asset::AssetLoader for HierarchyTreeLoader {
    type Asset = HierarchyTree;

    type Settings = ();

    type Error = HierarchyTreeLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut bevy::asset::io::Reader,
        _: &'a (),
        _: &'a mut bevy::asset::LoadContext,
    ) -> impl bevy::utils::ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;
            let json = serde_json::from_slice(&bytes)?;

            Ok(HierarchyTree(json))
        }
    }

    fn extensions(&self) -> &[&str] {
        &["json"]
    }
}

pub struct ShipLoader;

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum ShipLoaderError {
    /// An [IO Error](std::io::Error)
    #[error("Could not read the file: {0}")]
    Io(#[from] std::io::Error),
    /// A [JSON Error](serde_json::error::Error)
    #[error("Could not parse the JSON: {0}")]
    JsonError(#[from] serde_json::error::Error),
}

impl bevy::asset::AssetLoader for ShipLoader {
    type Asset = Ship;

    type Settings = ();

    type Error = ShipLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut bevy::asset::io::Reader,
        _: &'a (),
        _: &'a mut bevy::asset::LoadContext,
    ) -> impl bevy::utils::ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;

            Ok(serde_json::from_slice(&bytes)?)
        }
    }

    fn extensions(&self) -> &[&str] {
        &["json"]
    }
}
