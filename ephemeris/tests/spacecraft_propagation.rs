mod common;

use ephemeris::{
    AccelerationModel, AccelerationPairedAssign, BoundedPropagator, BoundedTrajectory,
    CubicHermiteSplineSolout, EvaluateTrajectory, ExtendSpline, Forward, Frame, GravityModel,
    GravitySource, InterpolationSample, InterpolationTolerance, NBodyPropagatorError,
    PropagationContext, QuinticHermite, QuinticHermiteSpline, Transform,
};
use ftime::{Duration, Epoch};
use glam::{DMat3, DVec3};
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;
use particular::{
    gravity::newtonian::{AccelerationAt, AccelerationPaired},
    prelude::{Mass, Position},
};

#[derive(Position, Mass)]
pub struct PointMass {
    position: DVec3,
    mu: f64,
}

impl AccelerationPairedAssign for PointMass {
    type Output = DVec3;

    #[inline]
    fn acceleration_paired_assign(
        &self,
        other: &Self,
        a1: &mut Self::Output,
        a2: &mut Self::Output,
    ) {
        let (ai, aj) = self.acceleration_paired(other, &0.0);
        *a1 += ai;
        *a2 += aj;
    }
}

impl GravitySource for PointMass {
    type Vector = DVec3;

    type Scalar = f64;

    #[inline]
    fn init(position: Self::Vector, mu: Self::Scalar) -> Self {
        Self { position, mu }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct QuinticHermiteSample {
    pub time: Epoch,
    pub position: DVec3,
    pub velocity: DVec3,
    pub acceleration: DVec3,
}

impl QuinticHermiteSample {
    #[inline]
    pub fn new(time: Epoch, position: DVec3, velocity: DVec3, acceleration: DVec3) -> Self {
        Self {
            time,
            position,
            velocity,
            acceleration,
        }
    }
}

impl InterpolationSample<DVec3> for QuinticHermiteSample {
    type Trajectory = QuinticHermiteSpline<DVec3>;

    #[inline]
    fn new(time: Epoch, position: DVec3, velocity: DVec3, acceleration: DVec3) -> Self {
        Self {
            time,
            position,
            velocity,
            acceleration,
        }
    }
}

impl ExtendSpline<Forward, QuinticHermiteSample> for QuinticHermiteSpline<DVec3> {
    #[inline]
    fn push_at_bound(&mut self, sample: QuinticHermiteSample) {
        self.push_back(
            sample.time,
            sample.position,
            sample.velocity,
            sample.acceleration,
        );
    }
}

#[derive(Clone, Copy)]
pub struct MidDistance(f64);

impl MidDistance {
    #[inline]
    pub fn new(tolerance: f64) -> Self {
        Self(tolerance * tolerance)
    }
}

impl InterpolationTolerance<QuinticHermiteSample> for MidDistance {
    #[inline]
    fn is_valid(&mut self, samples: &[QuinticHermiteSample]) -> bool {
        let len = samples.len();
        if len < 3 {
            return true;
        }

        let first = samples[0];
        let last = samples[len - 1];
        let hermite = QuinticHermite::new(
            (
                first.time.as_offset_seconds(),
                last.time.as_offset_seconds(),
            ),
            (first.position, last.position),
            (first.velocity, last.velocity),
            (first.acceleration, last.acceleration),
        );

        let mid = len / 2;
        let mid_sample = &samples[mid];

        hermite
            .eval(mid_sample.time.as_offset_seconds())
            .distance_squared(mid_sample.position)
            <= self.0
    }
}

type StateVector = ephemeris::StateVector<DVec3>;
type ForceModel = ephemeris::NewtonianGravity<PointMass>;
type NBodyProblem = ephemeris::NBodyProblem<DVec3, ForceModel>;
type SplineInterpolators<D> =
    ephemeris::SplineInterpolators<D, QuinticHermiteSample, DVec3, MidDistance>;
type NBodyPropagator<M> =
    ephemeris::NBodyPropagator<DVec3, ForceModel, M, SplineInterpolators<Forward>>;

#[inline]
fn propagate_solar_system<M>(
    system: &SolarSystem,
    h: Duration,
    end: Epoch,
) -> Result<Vec<QuinticHermiteSpline<DVec3>>, NBodyPropagatorError>
where
    M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
    M::Integrator: Integrator<NBodyProblem> + IntegratorState<Time = f64>,
{
    let ((positions, velocities), mus): ((Vec<_>, _), _) = system
        .bodies
        .iter()
        .map(|body| ((body.position, body.velocity), body.mu))
        .unzip();

    let mut problem = NBodyProblem::new(
        system.epoch.as_offset_seconds(),
        f64::INFINITY,
        SecondOrderState::new(positions, velocities),
        GravityModel::new(ForceModel::new(mus)).cache_evaluations(true),
    );

    let solout = SplineInterpolators::new(&mut problem, MidDistance::new(1e-3));
    let mut propagator = NBodyPropagator::<M>::new_with_solout::<Forward>(h, problem, solout);

    propagator.propagate(end)
}

fn generate_celestial_bodies<M>(
    system: &SolarSystem,
    h: Duration,
    end: Epoch,
) -> Result<CelestialBodies, NBodyPropagatorError>
where
    M: NewMethod<FixedMethodParams<f64>> + Method<NBodyProblem>,
    M::Integrator: Integrator<NBodyProblem> + IntegratorState<Time = f64>,
{
    let trajectories = propagate_solar_system::<M>(system, h, end)?;

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
    trajectory: QuinticHermiteSpline<DVec3>,
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

impl PropagationContext for &CelestialBodies {
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
        context: &&CelestialBodies,
    ) -> Option<Self::Transform> {
        match self {
            ReferenceFrame::Relative(reference) => Some(TNB::new(
                *sv - context.bodies[reference].trajectory.state_vector(t)?,
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
    CubicHermiteSplineSolout,
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
    let mut propagator = SpacecraftPropagator::new(
        initial_time,
        initial_state,
        ADAPTIVE_PARAMETERS,
        timeline,
        &celestial_bodies,
        CubicHermiteSplineSolout,
    );

    let end = "1951-01-01 00:00:00".parse()?;
    let spacecraft_trajectory = propagator.propagate(end)?;

    let distance_from = |body: SolarSystemObject, t| {
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

    assert!(distance_from(SolarSystemObject::Earth, "1950-01-01 00:00:00".parse()?)? < 10_000.0);
    assert!(distance_from(SolarSystemObject::Earth, "1950-01-01 00:15:00".parse()?)? < 10_000.0);
    // Spacecraft enters and stays in Mars' orbit.
    assert!(distance_from(SolarSystemObject::Mars, "1950-07-27 15:45:00".parse()?)? < 10_000.0);
    assert!(distance_from(SolarSystemObject::Mars, "1951-01-01 00:00:00".parse()?)? < 10_000.0);

    Ok(())
}
