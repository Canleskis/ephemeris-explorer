use std::hash::{Hash, Hasher};

use bevy_math::DVec3;
use ephemeris::{NBodyProblem, NewtonianGravity};
use hifitime::{Duration, Epoch};
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;

const CACHE_PATH: &str = "fetch_cache";

fn fetch_solar_systems_cached(
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
    // for &body in bodies {
    //     let data = horizons_solar_system::fetch_body(body, start, end, step)
    //         .expect("Failed to fetch data");

    //     println!("Successfully fetched data for {body:?}");
    //     for (epoch, body) in data {
    //         systems
    //             .entry(epoch)
    //             .or_insert_with(|| SolarSystem {
    //                 epoch,
    //                 bodies: Vec::new(),
    //             })
    //             .bodies
    //             .push(body);
    //     }
    // }
    systems = horizons_solar_system::fetch_solar_system(BODIES, start, end, step)
        .unwrap()
        .into_iter()
        .map(|system| (system.epoch, system))
        .collect();

    // Write to cache
    let cache_path = std::path::Path::new(CACHE_PATH).join(hash.to_string());
    _ = std::fs::create_dir_all(&cache_path).ok();
    for system in systems.values() {
        let jd = system.epoch.to_jde_tt_days();
        let mut file = std::fs::File::create(cache_path.join(jd.to_string()))
            .expect("Failed to create cache file");
        bincode::serde::encode_into_std_write(system, &mut file, bincode::config::standard())
            .expect("Failed to write cache");
    }

    systems
}

fn barycenter(system: &SolarSystem) -> DVec3 {
    let total_mass: f64 = system.bodies.iter().map(|b| b.mu).sum();
    system
        .bodies
        .iter()
        .map(|b| b.position * b.mu)
        .sum::<DVec3>()
        / total_mass
}

fn n_body_problem_from_solar_system(system: &SolarSystem) -> NBodyProblem<DVec3> {
    let ((positions, velocities), mus): ((Vec<DVec3>, Vec<DVec3>), Vec<f64>) = system
        .bodies
        .iter()
        .map(|body| ((body.position, body.velocity), body.mu))
        .collect();

    NBodyProblem {
        time: system.epoch.to_tai_seconds(),
        bound: f64::INFINITY,
        state: SecondOrderState {
            y: positions,
            dy: velocities,
        },
        ode: NewtonianGravity {
            gravitational_parameters: mus,
        },
    }
}

#[rustfmt::skip]
const BODIES: &[SolarSystemObject] = {
    use SolarSystemObject::*;
    &[
        Sun,
            Mercury,
            Venus,
            Earth,
                Moon,
            Mars,
            JupiterBarycenter,
            SaturnBarycenter,
            UranusBarycenter,
            NeptuneBarycenter,
    ]
};

#[test]
fn jpl_comparison() -> Result<(), Box<dyn std::error::Error>> {
    let start = Epoch::from_gregorian_str("2000-01-01T00:00:00 TAI")?;
    let end = Epoch::from_gregorian_str("2001-01-01T00:00:00 TAI")?;
    let compare_step = Duration::from_hours(12.0);

    let delta = Duration::from_hours(6.0);

    let systems = fetch_solar_systems_cached(BODIES, start, end, compare_step);
    let end = *systems.last_key_value().unwrap().0;

    let (initial_epoch, initial_state) = systems.first_key_value().unwrap();
    println!("Barycenter: {}", barycenter(initial_state));

    let method: QuinlanTremaine12 = QuinlanTremaine12::new(delta.to_seconds());
    let nbody = n_body_problem_from_solar_system(initial_state);
    let mut integrator = method.integrate(nbody);

    let mut max_errors = std::collections::HashMap::<SolarSystemObject, f64>::new();

    let mut t = *initial_epoch;
    loop {
        integrator.step()?;
        t += delta;

        if let Some(real_system) = systems.get(&t) {
            println!("{t}:");
            for (i, body) in BODIES.iter().enumerate() {
                let real_position = real_system.bodies[i].position;
                let integrated_position = integrator.problem.state.y[i];
                let abs_error = integrated_position.distance(real_position);

                let max_error = max_errors.entry(*body).or_insert(0.0);
                *max_error = (*max_error).max(abs_error);

                println!("  {body}: {abs_error} km error");
            }
        }

        if t >= end {
            break;
        }
    }

    println!("{:#?}", &max_errors);

    assert!(max_errors[&SolarSystemObject::Sun] < 1.0);
    assert!(max_errors[&SolarSystemObject::JupiterBarycenter] < 1.0);
    assert!(max_errors[&SolarSystemObject::SaturnBarycenter] < 1.0);
    assert!(max_errors[&SolarSystemObject::UranusBarycenter] < 1.0);
    assert!(max_errors[&SolarSystemObject::NeptuneBarycenter] < 1.0);
    // The inner-planet absolute errors can reach 100s of km because the model currently only uses
    // Newtonian point-mass gravity, neglecting oblateness and relativistic corrections.
    assert!(max_errors[&SolarSystemObject::Mercury] < 200.0);
    assert!(max_errors[&SolarSystemObject::Venus] < 100.0);
    assert!(max_errors[&SolarSystemObject::Earth] < 100.0);
    assert!(max_errors[&SolarSystemObject::Moon] < 100.0);
    assert!(max_errors[&SolarSystemObject::Mars] < 100.0);

    Ok(())
}
