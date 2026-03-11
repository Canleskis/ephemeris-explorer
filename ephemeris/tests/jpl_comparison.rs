use ephemeris::{NBodyProblem, NewtonianGravity};
use ftime::Duration;
use glam::DVec3;
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;

mod common;

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
        time: system.epoch.as_offset_seconds(),
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
    let start = "2000-01-01 00:00:00".parse()?;
    let end = "2001-01-01 00:00:00".parse()?;
    let compare_step = Duration::from_hours(12.0);

    let h = Duration::from_hours(6.0);

    let systems = common::fetch_solar_systems_cached(BODIES, start, end, compare_step);
    let end = *systems.last_key_value().unwrap().0;

    let (initial_epoch, initial_state) = systems.first_key_value().unwrap();
    println!("Barycenter: {}", barycenter(initial_state));

    let method: QuinlanTremaine12<_> =
        QuinlanTremaine12::new(FixedMethodParams::new(h.as_seconds()));
    let nbody = n_body_problem_from_solar_system(initial_state);
    let mut integrator = method.integrate(nbody);

    let mut max_errors = std::collections::HashMap::<SolarSystemObject, f64>::new();

    let mut t = *initial_epoch;
    loop {
        integrator.step()?;
        t += h;

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
