use std::f64;
use std::hash::{Hash, Hasher};
use std::ops::{Add, Div, Mul, Neg, Sub};

use bevy_math::DVec3;
use ephemeris::StateVector;
use hifitime::{Duration, Epoch, Unit};
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationPaired;

const CACHE_PATH: &str = "fetch_cache";

fn fetch_solar_system_cached(bodies: &[SolarSystemObject], start: Epoch) -> SolarSystem {
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
    let mut file = std::fs::File::create(cache_path).expect("Failed to create cache file");
    bincode::serde::encode_into_std_write(&system, &mut file, bincode::config::standard())
        .expect("Failed to write cache");

    system
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Double<T> {
    value: T,
    error: T,
}

impl<T> Double<T> {
    #[inline]
    fn new(value: T) -> Self
    where
        T: Default,
    {
        Self {
            value,
            error: T::default(),
        }
    }

    #[inline]
    fn two_sum(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Copy,
    {
        let value = a + b;
        let v = value - a;
        let error = (a - (value - v)) + (b - v);

        Self { value, error }
    }

    #[inline]
    fn two_sub(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Neg<Output = T> + Copy,
    {
        Self::two_sum(a, -b)
    }

    #[inline]
    fn fast_two_sum(a: T, b: T) -> Self
    where
        T: Add<Output = T> + Sub<Output = T> + Copy,
    {
        let value = a + b;
        let error = b - (value - a);

        Self { value, error }
    }
}

impl<T> Add for Double<T>
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self::Output {
        let sum = Double::two_sum(self.value, other.value);
        Double::fast_two_sum(sum.value, (sum.error + self.error) + other.error)
    }
}

impl<T> Sub for Double<T>
where
    T: Add<Output = T> + Sub<Output = T> + Neg<Output = T> + Copy,
{
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self::Output {
        let sum = Double::two_sub(self.value, other.value);
        Double::fast_two_sum(sum.value, (sum.error + self.error) - other.error)
    }
}

impl Mul<Double<DVec3>> for f64 {
    type Output = Double<DVec3>;

    #[inline]
    fn mul(self, rhs: Double<DVec3>) -> Self::Output {
        Double {
            value: rhs.value * self,
            error: rhs.error * self,
        }
    }
}

impl Mul<f64> for Double<DVec3> {
    type Output = Double<DVec3>;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Double {
            value: self.value * rhs,
            error: self.error * rhs,
        }
    }
}

impl Div<f64> for Double<DVec3> {
    type Output = Double<DVec3>;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Double {
            value: self.value / rhs,
            error: self.error / rhs,
        }
    }
}

#[derive(Clone, Debug)]
pub struct NewtonianGravity {
    pub gravitational_parameters: Vec<f64>,
}

impl SecondOrderODE<f64, Vec<Double<DVec3>>> for NewtonianGravity {
    #[inline]
    fn eval(
        &mut self,
        _: f64,
        y: &Vec<Double<DVec3>>,
        ddy: &mut Vec<Double<DVec3>>,
    ) -> Result<(), EvalFailed> {
        ddy.fill(Double::new(DVec3::ZERO));
        for i in 0..y.len() {
            let mut output_i = DVec3::ZERO;
            let p_i = (y[i].value, self.gravitational_parameters[i]);

            for j in (i + 1)..y.len() {
                let p_j = (y[j].value, self.gravitational_parameters[j]);
                let computed = p_i.acceleration_paired(&p_j, &0.0);
                output_i += computed.0;
                ddy[j].value += computed.1;
            }

            ddy[i].value += output_i;
        }

        Ok(())
    }
}

impl FirstOrderODE<f64, Vec<StateVector<Double<DVec3>>>> for NewtonianGravity {
    #[inline]
    fn eval(
        &mut self,
        _: f64,
        y: &Vec<StateVector<Double<DVec3>>>,
        dy: &mut Vec<StateVector<Double<DVec3>>>,
    ) -> Result<(), EvalFailed> {
        for (y, dy) in y.iter().zip(dy.iter_mut()) {
            dy.position = y.velocity;
            dy.velocity = Double::new(DVec3::ZERO);
        }
        for i in 0..y.len() {
            let mut output_i = DVec3::ZERO;
            let p_i = (y[i].position.value, self.gravitational_parameters[i]);

            for j in (i + 1)..y.len() {
                let p_j = (y[j].position.value, self.gravitational_parameters[j]);
                let computed = p_i.acceleration_paired(&p_j, &0.0);
                output_i += computed.0;
                dy[j].velocity.value += computed.1;
            }

            dy[i].velocity.value += output_i;
        }

        Ok(())
    }
}

pub type NBodyProblem<V = Vec<Double<DVec3>>> =
    ODEProblem<f64, SecondOrderState<V>, NewtonianGravity>;

fn n_body_from_solar_system(system: &SolarSystem) -> NBodyProblem {
    let ((positions, velocities), mus): ((Vec<_>, Vec<_>), Vec<f64>) = system
        .bodies
        .iter()
        .map(|body| {
            (
                (Double::new(body.position), Double::new(body.velocity)),
                body.mu,
            )
        })
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
        let mut h = h.to_seconds();
        let n = problem.state.y.len();
        let mut integrator = M::new(FixedMethodParams::new(h / 2.0)).integrate(problem.clone());
        let truth = integrator.solve()?.1;

        let mut errors = vec![];

        loop {
            let mut integrator = M::new(FixedMethodParams::new(h)).integrate(problem.clone());
            let (_, state) = integrator.solve()?;

            let mut max_error = SecondOrderState::new(0.0, 0.0);
            for i in 0..n {
                let position_error = (state.y[i].value - truth.y[i].value).length() * 1e3;
                let velocity_error = (state.dy[i].value - truth.dy[i].value).length() * 1e3;

                max_error.y = position_error.max(max_error.y);
                max_error.dy = velocity_error.max(max_error.dy);
            }

            if (integrator.problem.bound - integrator.problem.time).abs() != 0.0 {
                return Err(ConvergenceError::Overstep);
            }

            errors.push((Duration::from_seconds(h), max_error));

            if max_error.y > 10.0 || max_error.dy > 1.0 {
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
            Vesta,
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
                // Naiad,
                // Thalassa,
                // Despina,
                // Galatea,
                // Larissa,
                // Proteus,
            Pluto,
                Charon,
            Eris,
    ]
};

/// Test to find the time step that integrates the solar system over one year with position errors
/// below 10 meters for all bodies.
#[test]
fn solar_system_convergence() -> Result<(), Box<dyn std::error::Error>> {
    let start = Epoch::from_gregorian_str("2000-01-01T00:00:00 TAI")?;
    let end = Epoch::from_gregorian_str("2001-01-01T00:00:00 TAI")?;

    let mut n_body_problem = n_body_from_solar_system(&fetch_solar_system_cached(BODIES, start));
    n_body_problem.bound = end.to_tai_seconds();

    use Unit::*;
    type Starter<T> = Substepper<4, BlanesMoan6B<T>>;

    assert_eq!(
        convergence::<QuinlanTremaine12<_, Starter<_>>>(&n_body_problem, 75 * Second)?,
        10 * Minute
    );
    assert_eq!(
        convergence::<Stormer13<_, Starter<_>>>(&n_body_problem, 75 * Second)?,
        5 * Minute
    );
    assert_eq!(
        convergence::<BlanesMoan14A<_>>(&n_body_problem, 75 * Second)?,
        10 * Minute
    );

    Ok(())
}
