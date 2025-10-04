use crate::prediction::Trajectory;

use bevy::math::{DMat3, DVec3};
use bevy::prelude::*;
use ephemeris::{
    eval_slice_horner, AccelerationModel, BoundedTrajectory, EvaluateTrajectory, Interpolation,
    Polynomial, PropagationContext, Transform,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

pub type StateVector = ephemeris::StateVector<DVec3>;

pub type UniformSpline = ephemeris::UniformSpline<DVec3>;
pub type CubicHermiteSplineSamples = ephemeris::CubicHermiteSplineSamples<DVec3>;

#[derive(Clone, Copy, Debug)]
pub struct LeastSquaresFit {
    pub degree: usize,
}

impl Interpolation<DVec3> for LeastSquaresFit {
    // Credit to the poly_it crate (https://github.com/SkyeC0re/polyit-rs)
    #[inline]
    fn interpolate(&self, ts: &[f64], xs: &[DVec3]) -> Option<Polynomial<DVec3>> {
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
            return None;
        }
        // Impossible for `data_len` to be zero at this stage.
        let degree = self.degree.min(data_len - 1);

        b_0 /= gamma_0;
        d_0 /= gamma_0;

        let mut p_data = smallvec::SmallVec::new_const();

        if degree == 0 {
            p_data.push(d_0);
            return Some(Polynomial::new(p_data));
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
        Some(p)
    }
}

pub type NBodyPropagator<D> =
    ephemeris::NBodyPropagator<D, DVec3, QuinlanTremaine12<f64>, LeastSquaresFit>;

pub use ephemeris::{Backward, Forward};

#[derive(Clone, Copy, Debug)]
pub struct TNB(pub DMat3);

impl Default for TNB {
    #[inline]
    fn default() -> Self {
        Self(DMat3::IDENTITY)
    }
}

impl From<StateVector> for TNB {
    #[inline]
    fn from(sv: StateVector) -> Self {
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

pub type ReferenceFrame = ephemeris::ReferenceFrame<Trajectory, TNB>;

pub type ConstantThrust = ephemeris::ConstantThrust<DVec3, ReferenceFrame>;

#[derive(Clone, Copy, Debug, Component, Deref, DerefMut)]
pub struct Mu(pub f64);

#[derive(Clone)]
pub struct Bodies(pub bevy::ecs::entity::EntityHashMap<(Trajectory, Mu)>);

impl AccelerationModel for Bodies {
    type Vector = DVec3;

    #[inline]
    fn acceleration(&self, t: Epoch, state: &StateVector) -> Option<Self::Vector> {
        let mut acc = DVec3::ZERO;
        for (traj, mu) in self.0.values() {
            let position = traj.position(t)?;
            acc += (position, **mu).acceleration_at::<false>(&state.position, &0.0);
        }
        Some(acc)
    }
}

impl PropagationContext for Bodies {
    #[inline]
    fn max_time(&self) -> Epoch {
        self.0
            .values()
            .fold(Epoch::from_offset(Duration::MAX), |end, (traj, _)| {
                end.min(traj.end())
            })
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

pub type SpacecraftPropagator =
    ephemeris::SpacecraftPropagator<Bodies, DVec3, ReferenceFrame, Verner87<f64, AbsTol, f64>>;

const fn ratio_f64(ratio: Ratio<u16>) -> f64 {
    ratio.numerator() as f64 / ratio.denominator() as f64
}

pub const DEFAULT_PARAMS: AdaptiveMethodParams<f64, AbsTol, f64> = AdaptiveMethodParams::with(
    f64::MAX,
    f64::MAX,
    AbsTol(StateVector {
        position: DVec3::splat(1e-7),
        velocity: DVec3::splat(1e-5),
    }),
    ratio_f64(integration::FAC_MIN_DEFAULT),
    ratio_f64(integration::FAC_MAX_DEFAULT),
    ratio_f64(integration::FAC_DEFAULT),
    1_000_000,
);
