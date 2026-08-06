mod common;

use common::DoubleDVec3;
use ephemeris::{AccelerationPairedAssign, GravityModel, GravitySource};
use ftime::{Duration, Epoch, Unit};
use glam::DVec3;
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;
use particular::{
    gravity::newtonian::AccelerationPaired,
    prelude::{Mass, Position},
};

// Double precision for point-mass acceleration is enough. Implemented to work with double-double.
#[derive(Position, Mass)]
pub struct PointMass {
    position: DVec3,
    mu: f64,
}

impl AccelerationPairedAssign for PointMass {
    type Output = DoubleDVec3;

    #[inline]
    fn acceleration_paired_assign(
        &self,
        other: &Self,
        a1: &mut Self::Output,
        a2: &mut Self::Output,
    ) {
        let (ai, aj) = self.acceleration_paired(other, &0.0);
        a1.h += ai;
        a2.h += aj;
    }
}

impl GravitySource for PointMass {
    type Vector = DoubleDVec3;

    type Scalar = f64;

    #[inline]
    fn init(position: Self::Vector, mu: Self::Scalar) -> Self {
        Self {
            position: position.h,
            mu,
        }
    }
}

pub type NewtonianGravity = ephemeris::NewtonianGravity<PointMass>;
pub type NBodyProblem = ephemeris::NBodyProblem<DoubleDVec3, NewtonianGravity>;

fn n_body_from_solar_system(system: &SolarSystem) -> NBodyProblem {
    let ((positions, velocities), mus): ((Vec<_>, Vec<_>), Vec<f64>) = system
        .bodies
        .iter()
        .map(|body| {
            (
                (
                    DoubleDVec3::new(body.position),
                    DoubleDVec3::new(body.velocity),
                ),
                body.mu,
            )
        })
        .collect();

    NBodyProblem {
        time: system.epoch.as_offset_seconds(),
        bound: f64::INFINITY,
        state: SecondOrderState {
            y: positions,
            dy: velocities,
        },
        ode: GravityModel::new(NewtonianGravity::new(mus)),
    }
}

#[derive(Debug)]
enum ConvergenceError {
    IntegrationError(StepError),
    NoResults,
    Overstep,
}
impl std::fmt::Display for ConvergenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConvergenceError::IntegrationError(e) => write!(f, "Integration error: {e}"),
            ConvergenceError::NoResults => write!(f, "No convergence results"),
            ConvergenceError::Overstep => write!(f, "Overstepped integration bounds"),
        }
    }
}
impl std::error::Error for ConvergenceError {}
impl From<StepError> for ConvergenceError {
    #[inline]
    fn from(e: StepError) -> Self {
        ConvergenceError::IntegrationError(e)
    }
}

fn convergence<M>(problem: &NBodyProblem, h: Duration) -> Result<Duration, ConvergenceError>
where
    M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
    M::Integrator: Integrator<NBodyProblem>,
{
    fn convergence_results<M>(
        problem: &NBodyProblem,
        h: Duration,
    ) -> Result<Vec<(Duration, SecondOrderState<f64>)>, ConvergenceError>
    where
        M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
        M::Integrator: Integrator<NBodyProblem>,
    {
        let mut h = h.as_seconds();
        let n = problem.state.y.len();
        let integration = M::new(FixedMethodParams::new(h / 2.0))
            .integrate(problem.clone())
            .with_solout(EveryStepSolout);
        let (_, truth) = integration.solve()?;

        let mut errors = vec![];

        loop {
            let integration = M::new(FixedMethodParams::new(h))
                .integrate(problem.clone())
                .with_solout(EveryStepSolout);
            let (end, state) = integration.solve()?;

            let mut max_position_errors = vec![0.0; n];
            let mut max_velocity_errors = vec![0.0; n];
            for i in 0..n {
                let position_error = (state.y[i] - truth.y[i]).as_dvec3().length() * 1e3;
                let velocity_error = (state.dy[i] - truth.dy[i]).as_dvec3().length() * 1e3;

                max_position_errors[i] = position_error.max(max_position_errors[i]);
                max_velocity_errors[i] = velocity_error.max(max_velocity_errors[i]);
                println!("{}: {}", BODIES[i], max_position_errors[i]);
            }

            if (problem.bound - end).abs() != 0.0 {
                return Err(ConvergenceError::Overstep);
            }
            let max_position_error = max_position_errors
                .iter()
                .copied()
                .max_by(f64::total_cmp)
                .unwrap_or(0.0);
            let max_velocity_error = max_velocity_errors
                .iter()
                .copied()
                .max_by(f64::total_cmp)
                .unwrap_or(0.0);

            errors.push((
                Duration::from_seconds(h),
                SecondOrderState::new(max_position_error, max_velocity_error),
            ));

            if max_position_error > 10.0 || max_velocity_error > 1.0 {
                break;
            }

            h *= 2.0;
        }

        Ok(errors)
    }

    match convergence_results::<M>(problem, h) {
        Ok(results) => match dbg!(&results).get(results.len().saturating_sub(2)) {
            Some((last_h, _)) => Ok(*last_h),
            None => Err(ConvergenceError::NoResults),
        },
        Err(e) => Err(e),
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
                Phobos,
                Deimos,
            Ceres,
            Jupiter,
                Io,
                Europa,
                Ganymede,
                Callisto,
            Saturn,
                Mimas,
                Enceladus,
                Tethys,
                Dione,
                Rhea,
                Titan,
                Iapetus,
            Uranus,
                Ariel,
                Umbriel,
                Titania,
                Oberon,
                Miranda,
            Neptune,
                Triton,
            Pluto,
                Charon,
    ]
};

/// Test to find the time step that integrates the solar system over one year with position errors
/// below 10 meters for all bodies.
#[test]
fn solar_system_convergence() -> Result<(), Box<dyn std::error::Error>> {
    let start = "2000-01-01 00:00:00".parse()?;
    let mut n_body_problem =
        n_body_from_solar_system(&common::fetch_solar_system_cached(BODIES, start));
    let end = "2001-01-01 00:00:00".parse::<Epoch>()?;
    n_body_problem.bound = end.as_offset_seconds();

    use Unit::*;

    assert_eq!(
        convergence::<QuinlanTremaine12<_>>(&n_body_problem, 10.0 * Minute)?,
        10.0 * Minute
    );
    assert_eq!(
        convergence::<Stormer13<_>>(&n_body_problem, 75.0 * Second)?,
        5.0 * Minute
    );
    assert_eq!(
        convergence::<BlanesMoan14A<_>>(&n_body_problem, 75.0 * Second)?,
        10.0 * Minute
    );

    Ok(())
}
