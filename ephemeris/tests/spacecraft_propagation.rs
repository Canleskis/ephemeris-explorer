use ephemeris::{
    AccelerationModel, BoundedTrajectory, EvaluateTrajectory, Forward, Frame,
    InterpolationAlgorithm, NBodyPropagatorError, Polynomial, Propagation, PropagationEnvironment,
    Transform, eval_slice_horner,
};
use ftime::{Duration, Epoch};
use glam::{DMat3, DVec3};
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

mod common;

#[derive(Clone, Copy, Debug)]
pub struct LeastSquaresFit {
    pub degree: usize,
}

impl InterpolationAlgorithm<DVec3> for LeastSquaresFit {
    type Error = ();

    // Credit to the poly_it crate (https://github.com/SkyeC0re/polyit-rs)
    #[inline]
    fn interpolate(&self, ts: &[f64], xs: &[DVec3]) -> Result<Polynomial<DVec3>, Self::Error> {
        debug_assert!(ts.len() == xs.len());
        let data_len = ts.len();
        let (mut d_0, gamma_0, mut b_0) = ts.iter().zip(xs.iter()).fold(
            (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO),
            |mut acc, (xi, yi)| {
                acc.0 += yi;
                acc.1 += DVec3::ONE;
                acc.2 += xi;
                acc
            },
        );

        if gamma_0 == DVec3::ZERO {
            return Err(());
        }
        // Impossible for `data_len` to be zero at this stage.
        let degree = self.degree.min(data_len - 1);

        b_0 /= gamma_0;
        d_0 /= gamma_0;

        let mut p_data = smallvec::SmallVec::new_const();

        if degree == 0 {
            p_data.push(d_0);
            return Ok(Polynomial::new(p_data));
        }

        for _ in 0..(degree + 1) {
            p_data.push(DVec3::ZERO);
        }
        let mut p_km1 = p_data.clone();
        let mut p_km1 = p_km1.as_mut_slice();
        let mut p_k = p_data.clone();
        let mut p_k = p_k.as_mut_slice();
        let p_data_slice = p_data.as_mut_slice();
        p_data_slice[0] = d_0;
        // Safety: `p_data` has `deg + 1` elements.
        *unsafe { p_k.get_unchecked_mut(0) } = DVec3::ONE;

        let mut gamma_k = gamma_0;
        let mut b_k = b_0;
        let mut minus_c_k = DVec3::ZERO;
        let mut kp1 = 1;

        loop {
            // Overwrite $p_{k-1}$ with $p_{k+1}$
            for i in 0..kp1 {
                // Safety: `kp1 <= deg + 1`.
                let p_km1i = unsafe { p_km1.get_unchecked_mut(i) };
                *p_km1i = minus_c_k * *p_km1i - b_k * unsafe { p_k.get_unchecked(i) };
            }

            for im1 in 0..kp1 {
                // Safety: `kp1 <= deg`.
                let p_km1i = unsafe { p_km1.get_unchecked_mut(im1 + 1) };
                *p_km1i += unsafe { p_k.get_unchecked(im1) };
            }

            let (mut d_kp1, gamma_kp1, mut b_kp1) = ts.iter().zip(xs.iter()).fold(
                (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO),
                |mut acc, (xi, yi)| {
                    // Safety: `kp1 <= deg`.
                    let px = eval_slice_horner(unsafe { p_km1.get_unchecked(0..(kp1 + 1)) }, xi);
                    let wipx = px;
                    acc.0 += yi * wipx;

                    let wipxpx = wipx * px;
                    acc.1 += wipxpx;
                    acc.2 += xi * wipxpx;

                    acc
                },
            );

            if gamma_kp1 == DVec3::ZERO {
                break;
            }

            d_kp1 /= gamma_kp1;

            for i in 0..(kp1 + 1) {
                // Safety: `kp1 <= deg`.
                let pi = unsafe { p_data_slice.get_unchecked_mut(i) };
                *pi += d_kp1 * unsafe { p_km1.get_unchecked(i) };
            }

            if kp1 == degree {
                break;
            }

            // Delay the remaining work left for $b_{k+1}$ until it is certain to be needed.
            b_kp1 /= gamma_kp1;

            kp1 += 1;
            b_k = b_kp1;
            minus_c_k = -(gamma_kp1 / gamma_k);
            gamma_k = gamma_kp1;

            // Reorient the offsets
            std::mem::swap(&mut p_k, &mut p_km1);
        }
        let mut p = Polynomial::new(p_data);
        p.trim();
        Ok(p)
    }
}

type StateVector = ephemeris::StateVector<DVec3>;
type NBodyProblem = ephemeris::NBodyProblem<DVec3>;
type NBodyPropagator<M> = ephemeris::NBodyPropagator<Forward, DVec3, M, LeastSquaresFit>;
type UniformSpline = ephemeris::UniformSpline<DVec3>;

#[inline]
fn propagate_solar_system<M>(
    system: &SolarSystem,
    parameters: &[(Duration, LeastSquaresFit)],
    h: Duration,
    end: Epoch,
) -> Result<Vec<UniformSpline>, NBodyPropagatorError>
where
    M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
    M::Integrator: Integrator<NBodyProblem> + IntegratorState<Time = f64>,
{
    let ((positions, velocities), mus) = system
        .bodies
        .iter()
        .map(|body| ((body.position, body.velocity), body.mu))
        .unzip();

    let mut propagation = Propagation::new(NBodyPropagator::<M>::new(
        Forward::new(h),
        system.epoch,
        positions,
        velocities,
        mus,
        parameters.to_vec(),
    ));
    propagation.propagate(end)?;

    Ok(propagation.into_inner().1)
}

fn generate_celestial_bodies<M>(
    system: &SolarSystem,
    parameters: &[(Duration, LeastSquaresFit)],
    h: Duration,
    end: Epoch,
) -> Result<CelestialBodies, NBodyPropagatorError>
where
    M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
    M::Integrator: Integrator<NBodyProblem> + IntegratorState<Time = f64>,
{
    let trajectories = propagate_solar_system::<M>(system, parameters, h, end)?;

    Ok(CelestialBodies {
        bodies: system
            .bodies
            .iter()
            .zip(trajectories)
            .zip(BODIES)
            .map(|((body, trajectory), object)| {
                (
                    *object,
                    CelestialBody {
                        trajectory,
                        mu: body.mu,
                    },
                )
            })
            .collect(),
    })
}

#[derive(Clone)]
struct CelestialBody {
    trajectory: UniformSpline,
    mu: f64,
}

impl CelestialBody {
    #[inline]
    fn acceleration_at(&self, t: Epoch, position: &DVec3) -> Option<DVec3> {
        let body_position = self.trajectory.position(t)?;
        Some((body_position, self.mu).acceleration_at::<false>(position, &0.0))
    }
}

#[derive(Clone)]
struct CelestialBodies {
    bodies: indexmap::IndexMap<SolarSystemObject, CelestialBody>,
}

impl AccelerationModel for &CelestialBodies {
    type Vector = DVec3;

    #[inline]
    fn acceleration(&self, t: Epoch, state: &StateVector) -> Option<Self::Vector> {
        let mut acc = DVec3::ZERO;
        for body in self.bodies.values() {
            acc += body.acceleration_at(t, &state.position)?;
        }
        Some(acc)
    }
}

impl PropagationEnvironment for &CelestialBodies {
    #[inline]
    fn max_time(&self) -> Epoch {
        self.bodies
            .values()
            .fold(Epoch::MAX, |end, body| end.min(body.trajectory.end()))
    }
}

#[derive(Clone, Copy, Default)]
pub struct AbsTol(pub StateVector);

impl Tolerance<[StateVector; 1]> for AbsTol {
    type Output = f64;

    #[inline]
    fn err_over_tol(&mut self, _: &[StateVector; 1], [e]: &[StateVector; 1]) -> Self::Output {
        let atol = self.0;
        (e.position / atol.position)
            .abs()
            .max_element()
            .max((e.velocity / atol.velocity).abs().max_element())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TNB(pub DMat3);

impl TNB {
    pub const IDENTITY: Self = Self(DMat3::IDENTITY);

    #[inline]
    pub fn new(sv: StateVector) -> Self {
        let x = sv.velocity.normalize_or_zero();
        let y = sv.position.cross(sv.velocity).normalize_or_zero();
        let z = x.cross(y).normalize_or_zero();
        Self(DMat3::from_cols(x, z, y))
    }
}

impl Transform<DVec3> for TNB {
    #[inline]
    fn to_inertial(&self, v: &DVec3) -> DVec3 {
        self.0.mul_vec3(*v)
    }
}

#[derive(Default)]
enum ReferenceFrame {
    /// Relative to a reference trajectory.
    Relative(SolarSystemObject),
    #[default]
    Inertial,
}

impl ReferenceFrame {
    #[inline]
    fn relative(reference: SolarSystemObject) -> Self {
        Self::Relative(reference)
    }

    #[expect(unused)]
    #[inline]
    fn inertial() -> Self {
        Self::Inertial
    }
}

impl Frame<DVec3, &CelestialBodies> for ReferenceFrame {
    type Transform = TNB;

    #[inline]
    fn transform(
        &self,
        t: Epoch,
        sv: &StateVector,
        environment: &&CelestialBodies,
    ) -> Option<Self::Transform> {
        match self {
            ReferenceFrame::Relative(reference) => Some(TNB::new(
                *sv - environment.bodies[reference].trajectory.state_vector(t)?,
            )),
            ReferenceFrame::Inertial => Some(TNB::IDENTITY),
        }
    }
}

type ConstantThrust = ephemeris::ConstantThrust<DVec3, ReferenceFrame>;
type Timeline = ephemeris::Timeline<DVec3, ReferenceFrame>;
type SpacecraftPropagator<'a> = ephemeris::SpacecraftPropagator<
    [StateVector; 1],
    ReferenceFrame,
    &'a CelestialBodies,
    Verner87<f64, AbsTol, f64>,
>;

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

const DELTA: Duration = Duration::from_hours(6.0);

#[rustfmt::skip]
const INTERPOLATION_PARAMETERS: &[(Duration, LeastSquaresFit)] = &[
    (Duration::from_hours(72.0), LeastSquaresFit { degree: 6 }), // Sun
        (Duration::from_hours(12.0), LeastSquaresFit { degree: 7 }), // Mercury
        (Duration::from_hours(60.0), LeastSquaresFit { degree: 7 }), // Venus
        (Duration::from_hours(18.0), LeastSquaresFit { degree: 7 }), // Earth
            (Duration::from_hours(6.0), LeastSquaresFit { degree: 6 }),  // Moon
        (Duration::from_hours(72.0), LeastSquaresFit { degree: 7 }), // Mars
        (Duration::from_hours(150.0), LeastSquaresFit { degree: 7 }), // JupiterBarycenter
        (Duration::from_hours(150.0), LeastSquaresFit { degree: 6 }), // SaturnBarycenter
        (Duration::from_hours(150.0), LeastSquaresFit { degree: 6 }), // UranusBarycenter
        (Duration::from_hours(150.0), LeastSquaresFit { degree: 5 }), // NeptuneBarycenter
];

const fn ratio_f64(ratio: Ratio<u16>) -> f64 {
    ratio.numerator() as f64 / ratio.denominator() as f64
}

pub const ADAPTIVE_PARAMETERS: AdaptiveMethodParams<f64, AbsTol, f64> = AdaptiveMethodParams::with(
    60.0,
    f64::MAX,
    AbsTol(StateVector {
        position: DVec3::splat(1e-3),
        velocity: DVec3::splat(1e-3),
    }),
    ratio_f64(integration::DEFAULT_FAC_MIN),
    ratio_f64(integration::DEFAULT_FAC_MAX),
    ratio_f64(integration::DEFAULT_FAC),
    1_000_000,
);

#[derive(Debug)]
struct OutOfBoundsEval;
impl std::fmt::Display for OutOfBoundsEval {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "evaluated out of the bounds of the trajectory")
    }
}
impl std::error::Error for OutOfBoundsEval {}

// Test for a spacecraft performing a series of burns, starting from an Earth orbit and eventually entering Mars' orbit.
#[test]
fn spacecraft_propagation() -> Result<(), Box<dyn std::error::Error>> {
    let celestial_bodies = generate_celestial_bodies::<QuinlanTremaine12<_>>(
        &common::fetch_solar_system_cached(BODIES, "1950-01-01 00:00:00".parse()?),
        INTERPOLATION_PARAMETERS,
        DELTA,
        "1952-01-01 00:00:00".parse()?,
    )?;

    let initial_time = "1950-01-01 00:00:00".parse()?;
    let initial_state = StateVector {
        position: DVec3::new(-27204249.668775786, 132947582.43848978, 57641619.74241204),
        velocity: DVec3::new(-22.253599106181895, -5.189518219791726, -2.2515617105336263),
    };

    let timeline = Timeline::new(vec![
        (
            "1950-01-01 00:15:15".parse()?,
            "1950-01-01 00:15:15".parse::<Epoch>()? + "5 min 15 s".parse()?,
            ConstantThrust::new(
                DVec3::new(0.0, 0.0, 10.0) / 1e3,
                ReferenceFrame::relative(SolarSystemObject::Earth),
            ),
        ),
        (
            "1950-01-01 00:43:10".parse()?,
            "1950-01-01 00:43:10".parse::<Epoch>()? + "6 min 30 s".parse()?,
            ConstantThrust::new(
                DVec3::new(9.97, -2.31, 0.3) / 1e3,
                ReferenceFrame::relative(SolarSystemObject::Sun),
            ),
        ),
        (
            "1950-02-28 04:12:25".parse()?,
            "1950-02-28 04:12:25".parse::<Epoch>()? + "1 min".parse()?,
            ConstantThrust::new(
                DVec3::new(0.51, -0.1, -6.53) / 1e3,
                ReferenceFrame::relative(SolarSystemObject::Mars),
            ),
        ),
        (
            "1950-07-27 15:44:05".parse()?,
            "1950-07-27 15:44:05".parse::<Epoch>()? + "5 min 10 s".parse()?,
            ConstantThrust::new(
                DVec3::new(-10.0, 0.0, 0.0) / 1e3,
                ReferenceFrame::relative(SolarSystemObject::Mars),
            ),
        ),
    ]);
    let mut propagation = Propagation::new(SpacecraftPropagator::new(
        initial_time,
        initial_state,
        ADAPTIVE_PARAMETERS,
        timeline,
        &celestial_bodies,
    ));

    let end = "1951-01-01 00:00:00".parse()?;
    propagation.propagate(end)?;
    let [spacecraft_trajectory] = propagation.trajectories();

    let distance_from = |t, body: SolarSystemObject| {
        Ok::<_, OutOfBoundsEval>(
            spacecraft_trajectory
                .position(t)
                .ok_or(OutOfBoundsEval)?
                .distance(
                    celestial_bodies.bodies[&body]
                        .trajectory
                        .position(t)
                        .ok_or(OutOfBoundsEval)?,
                ),
        )
    };

    assert!(distance_from("1950-01-01 00:00:00".parse()?, SolarSystemObject::Earth)? < 10_000.0);
    assert!(distance_from("1950-01-01 00:15:00".parse()?, SolarSystemObject::Earth)? < 10_000.0);
    // Spacecraft enters and stays in Mars' orbit.
    assert!(distance_from("1950-07-27 15:45:00".parse()?, SolarSystemObject::Mars)? < 10_000.0);
    assert!(distance_from("1951-01-01 00:00:00".parse()?, SolarSystemObject::Mars)? < 10_000.0);

    Ok(())
}
