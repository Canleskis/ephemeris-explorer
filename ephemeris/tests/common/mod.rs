#![allow(dead_code)]

use std::hash::{Hash, Hasher};

use ftime::{Duration, Epoch};
use horizons_solar_system::{SolarSystem, SolarSystemObject};

const CACHE_PATH: &str = "fetch_cache";

pub fn fetch_solar_system_cached(bodies: &[SolarSystemObject], start: Epoch) -> SolarSystem {
    let mut hasher = std::hash::DefaultHasher::new();
    (bodies, start).hash(&mut hasher);
    let hash = hasher.finish();

    // Check cache
    let cache_path = std::path::Path::new(CACHE_PATH).join(hash.to_string());
    if let Ok(mut file) = std::fs::File::open(&cache_path) {
        println!("Loading data from cache...");
        return bincode::serde::decode_from_std_read(&mut file, bincode::config::standard())
            .expect("Failed to read cache");
    }

    println!("Fetching data from JPL Horizons...");
    let system = horizons_solar_system::fetch_solar_system(
        bodies,
        start,
        start + Duration::from_seconds(1.0),
        Duration::from_days(1.0),
    )
    .unwrap()
    .remove(0);

    // Write to cache
    std::fs::create_dir_all(CACHE_PATH).expect("Failed to create cache directory");
    let mut file = std::fs::File::create(cache_path).expect("Failed to create cache file");
    bincode::serde::encode_into_std_write(&system, &mut file, bincode::config::standard())
        .expect("Failed to write cache");

    system
}

fn to_jde_tai_days(epoch: Epoch) -> f64 {
    const JD_TAI_EPOCH: f64 = 2436204.5;
    epoch.as_offset_seconds() / ftime::SEC_PER_DAY + JD_TAI_EPOCH
}

pub fn fetch_solar_systems_cached(
    bodies: &[SolarSystemObject],
    start: Epoch,
    end: Epoch,
    step: Duration,
) -> std::collections::BTreeMap<Epoch, SolarSystem> {
    let mut systems = std::collections::BTreeMap::<Epoch, SolarSystem>::new();

    let mut hasher = std::hash::DefaultHasher::new();
    (bodies, start, end, step).hash(&mut hasher);
    let hash = hasher.finish();

    // Check cache
    let cache_path = std::path::Path::new(CACHE_PATH).join(hash.to_string());
    if cache_path.is_dir() {
        println!("Loading data from cache...");
        for entry in std::fs::read_dir(cache_path).unwrap().flatten() {
            let mut file = std::fs::File::open(entry.path()).unwrap();
            let system: SolarSystem =
                bincode::serde::decode_from_std_read(&mut file, bincode::config::standard())
                    .expect("Failed to read cache");
            systems.insert(system.epoch, system);
        }
        return systems;
    }

    println!("Fetching data from JPL Horizons...");
    systems = horizons_solar_system::fetch_solar_system(bodies, start, end, step)
        .unwrap()
        .into_iter()
        .map(|system| (system.epoch, system))
        .collect();

    // Write to cache
    let cache_path = std::path::Path::new(CACHE_PATH).join(hash.to_string());
    std::fs::create_dir_all(&cache_path).expect("Failed to create cache directory");
    for system in systems.values() {
        let jd = to_jde_tai_days(system.epoch);
        let mut file = std::fs::File::create(cache_path.join(jd.to_string()))
            .expect("Failed to create cache file");
        bincode::serde::encode_into_std_write(system, &mut file, bincode::config::standard())
            .expect("Failed to write cache");
    }

    systems
}
