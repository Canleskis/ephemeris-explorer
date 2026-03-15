use crate::{
    dynamics::{PredictionTrajectory, Trajectory},
    prediction::PropagationTarget,
};

use bevy::ecs::query::QueryData;
use bevy::math::DVec3;
use bevy::prelude::*;
use ephemeris::{InterpolationAlgorithm, Polynomial, eval_slice_horner};
use integration::prelude::*;

pub use ephemeris::{Backward, Forward};
pub type UniformSpline = ephemeris::UniformSpline<DVec3>;

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

pub type NBodyPropagator<D> =
    ephemeris::NBodyPropagator<D, DVec3, QuinlanTremaine12<f64>, LeastSquaresFit>;

#[derive(QueryData)]
#[query_data(mutable)]
pub struct CelestialTrajectory<D> {
    pub trajectory: &'static mut Trajectory,
    marker: std::marker::PhantomData<D>,
}

impl PropagationTarget for CelestialTrajectory<Forward> {
    type Propagator = NBodyPropagator<Forward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_, '_>, propagated: UniformSpline) {
        match &mut *item.trajectory.write() {
            PredictionTrajectory::UniformSpline(trajectory) => {
                Self::Propagator::join(trajectory, propagated)
            }
            _ => unreachable!(),
        }
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_, '_>, propagated: UniformSpline) {
        // Should we keep the previous allocation?
        match &mut *item.trajectory.write() {
            PredictionTrajectory::UniformSpline(trajectory) => *trajectory = propagated,
            _ => unreachable!(),
        }
    }
}

impl PropagationTarget for CelestialTrajectory<Backward> {
    type Propagator = NBodyPropagator<Backward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_, '_>, propagated: UniformSpline) {
        match &mut *item.trajectory.write() {
            PredictionTrajectory::UniformSpline(trajectory) => {
                Self::Propagator::join(trajectory, propagated)
            }
            _ => unreachable!(),
        }
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_, '_>, propagated: UniformSpline) {
        // Should we keep the previous allocation?
        match &mut *item.trajectory.write() {
            PredictionTrajectory::UniformSpline(trajectory) => *trajectory = propagated,
            _ => unreachable!(),
        }
    }
}
