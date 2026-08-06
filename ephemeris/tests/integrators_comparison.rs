mod common;

use std::ops::{Add, Div, Mul, Sub};

use common::DoubleDVec3;
use double_double as dd;
use ephemeris::{AccelerationPairedAssign, GravityModel, GravitySource};
use ftime::{Epoch, Unit};
use glam::DVec3;
use horizons_solar_system::{SolarSystem, SolarSystemObject};
use integration::prelude::*;
use particular::{
    gravity::newtonian::AccelerationPaired,
    prelude::{Mass, Position},
};

#[derive(Clone, Copy, Debug, Default)]
pub struct DoubleDVec3Sloppy {
    pub h: DVec3,
    pub l: DVec3,
}

impl DoubleDVec3Sloppy {
    #[inline]
    pub fn new(h: DVec3) -> Self {
        Self { h, l: DVec3::ZERO }
    }

    #[inline]
    pub fn as_dvec3(&self) -> DVec3 {
        self.h
    }
}

impl From<DoubleDVec3Sloppy> for (DVec3, DVec3) {
    #[inline]
    fn from(value: DoubleDVec3Sloppy) -> Self {
        (value.h, value.l)
    }
}

impl From<(DVec3, DVec3)> for DoubleDVec3Sloppy {
    #[inline]
    fn from((h, l): (DVec3, DVec3)) -> Self {
        Self { h, l }
    }
}

impl Add for DoubleDVec3Sloppy {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        dd::sloppy::dw_add_dw(self.into(), rhs.into()).into()
    }
}

impl Sub for DoubleDVec3Sloppy {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        dd::sloppy::dw_sub_dw(self.into(), rhs.into()).into()
    }
}

impl Mul<f64> for DoubleDVec3Sloppy {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        dd::naive::dw_mul(self.into(), rhs).into()
    }
}

impl Div<f64> for DoubleDVec3Sloppy {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        dd::naive::dw_div(self.into(), rhs).into()
    }
}

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

#[derive(Position, Mass)]
pub struct PointMass2 {
    position: DVec3,
    mu: f64,
}

impl AccelerationPairedAssign for PointMass2 {
    type Output = DoubleDVec3Sloppy;

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

impl GravitySource for PointMass2 {
    type Vector = DoubleDVec3Sloppy;

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
pub type NewtonianGravity2 = ephemeris::NewtonianGravity<PointMass2>;
pub type NBodyProblem = ephemeris::NBodyProblem<DoubleDVec3, NewtonianGravity>;
pub type NBodyProblem2 = ephemeris::NBodyProblem<DoubleDVec3Sloppy, NewtonianGravity2>;

fn n_body_from(system: &SolarSystem) -> NBodyProblem {
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

fn n_body2_from(system: &SolarSystem) -> NBodyProblem2 {
    let ((positions, velocities), mus): ((Vec<_>, Vec<_>), Vec<f64>) = system
        .bodies
        .iter()
        .map(|body| {
            (
                (
                    DoubleDVec3Sloppy::new(body.position),
                    DoubleDVec3Sloppy::new(body.velocity),
                ),
                body.mu,
            )
        })
        .collect();

    NBodyProblem2 {
        time: system.epoch.as_offset_seconds(),
        bound: f64::INFINITY,
        state: SecondOrderState {
            y: positions,
            dy: velocities,
        },
        ode: GravityModel::new(NewtonianGravity2::new(mus)),
    }
}

#[derive(Debug)]
enum ComparisonError {
    IntegrationError(StepError),
    TimeDiverged,
}
impl std::fmt::Display for ComparisonError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ComparisonError::IntegrationError(e) => write!(f, "Integration error: {e}"),
            ComparisonError::TimeDiverged => write!(f, "The integrators time diverged"),
        }
    }
}
impl std::error::Error for ComparisonError {}
impl From<StepError> for ComparisonError {
    #[inline]
    fn from(e: StepError) -> Self {
        ComparisonError::IntegrationError(e)
    }
}

fn compare<M1, M2>(
    mut i1: Integration<NBodyProblem, M1>,
    mut i2: Integration<NBodyProblem2, M2>,
) -> Result<SecondOrderState<Vec<f64>>, ComparisonError>
where
    M1: Method<NBodyProblem>,
    M1::Integrator: Integrator<NBodyProblem>,
    M2: Method<NBodyProblem2>,
    M2::Integrator: Integrator<NBodyProblem2>,
{
    i1.advance_to_bound()?;
    i2.advance_to_bound()?;

    if i1.problem.as_ref().time != i2.problem.as_ref().time {
        return Err(ComparisonError::TimeDiverged);
    }

    let s1 = &i1.problem.as_ref().state;
    let s2 = &i2.problem.as_ref().state;

    let n = s1.y.len();
    let mut errors =
        SecondOrderState::new(vec![Default::default(); n], vec![Default::default(); n]);
    for i in 0..n {
        errors.y[i] = (s1.y[i].h - s2.y[i].h).length();
        errors.dy[i] = (s1.dy[i].h - s2.dy[i].h).length();
    }

    Ok(errors)
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

#[test]
fn integrators_comparison() -> Result<(), Box<dyn std::error::Error>> {
    let start = "2000-01-01 00:00:00".parse()?;
    let end = "2100-01-01 00:00:00".parse::<Epoch>()?;

    let mut n_body_problem = n_body_from(&common::fetch_solar_system_cached(BODIES, start));
    n_body_problem.bound = end.as_offset_seconds();

    let mut n_body_problem2 = n_body2_from(&common::fetch_solar_system_cached(BODIES, start));
    n_body_problem2.bound = end.as_offset_seconds();

    use Unit::*;

    let method = QuinlanTremaine12::<_>::new(FixedMethodParams::new((10.0 * Minute).as_seconds()));

    let errors = compare(
        method.integrate(n_body_problem),
        method.integrate(n_body_problem2),
    )?;

    let mut max_y = None;
    for (&name, (y, dy)) in BODIES.iter().zip(std::iter::zip(errors.y, errors.dy)) {
        let (max_name, max_y) = max_y.get_or_insert((name, y));
        if y > *max_y {
            *max_name = name;
            *max_y = y;
        }

        println!("{name}:");
        println!("    Position error: {:.4} m", y * 1e3);
        println!("    Velocity error: {:.4} m/s", dy * 1e3);
    }

    println!();
    if let Some((max_name, max_y)) = max_y {
        println!("Max position error: {max_name} with: {} m", max_y * 1e3);
    }

    Ok(())
}
