pub mod explicit;
pub mod nystrom;

pub use explicit::*;
pub use nystrom::explicit::*;
pub use nystrom::explicit_generalized::*;
pub use nystrom::symplectic::*;

use crate::{
    AdaptiveMethodParams, DivCeil, FixedIntegrator, FixedMethodParams, Integrator, IntegratorState,
    Method, NewMethod, StepError,
    problem::{EvalFailed, Problem},
};
use num_traits::{Inv, One, Pow, bounds::UpperBounded, clamp, clamp_max};
use std::ops::{Add, Div, Mul, Neg, Sub};

pub trait RKCoefficients<P> {
    type Instance;
}

pub trait RKState {
    fn step_count(&self) -> u32;
}

pub trait RKInstance<P: Problem> {
    fn from_problem(problem: &P) -> Self;

    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed>;
}

#[derive(Clone, Copy, Debug)]
pub struct FixedRungeKutta<C, T> {
    pub h: T,
    pub _phantom: std::marker::PhantomData<C>,
}

impl<C, T> NewMethod<FixedMethodParams<T>> for FixedRungeKutta<C, T> {
    #[inline]
    fn new(params: FixedMethodParams<T>) -> Self {
        Self {
            h: params.h,
            _phantom: std::marker::PhantomData,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FixedRungeKuttaIntegrator<R, T> {
    pub h: T,
    pub rk: R,
}

impl<C, P> Method<P> for FixedRungeKutta<C, P::Time>
where
    P: Problem,
    C: RKCoefficients<P>,
    C::Instance: RKInstance<P>,
{
    type Integrator = FixedRungeKuttaIntegrator<C::Instance, P::Time>;

    #[inline]
    fn init(self, problem: &P) -> Self::Integrator {
        FixedRungeKuttaIntegrator {
            h: self.h,
            rk: C::Instance::from_problem(problem),
        }
    }
}

impl<R, T> IntegratorState for FixedRungeKuttaIntegrator<R, T>
where
    R: RKState,
    T: Sub<Output = T> + DivCeil + Copy,
{
    type Time = T;

    #[inline]
    fn step_size(&self) -> Self::Time {
        self.h
    }

    #[inline]
    fn step_count(&self) -> u32 {
        self.rk.step_count()
    }

    #[inline]
    fn step_count_until_bound_hint(&self, duration: Self::Time) -> (usize, Option<usize>) {
        let step_count = self.step_count_until_bound(duration);

        (step_count, Some(step_count))
    }
}

impl<R, T> FixedIntegrator for FixedRungeKuttaIntegrator<R, T>
where
    R: RKState,
    T: Sub<Output = T> + DivCeil + Copy,
{
}

impl<P, R> Integrator<P> for FixedRungeKuttaIntegrator<R, P::Time>
where
    P: Problem,
    P::Time: Add<Output = P::Time> + PartialOrd + Copy,
    R: RKInstance<P>,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        if problem.as_ref().time >= problem.as_ref().bound {
            return Err(StepError::BoundReached);
        }

        if problem.as_ref().time + self.h == problem.as_ref().time {
            return Err(StepError::StepSizeUnderflow);
        }

        self.rk.advance(self.h, problem)?;

        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveRungeKutta<C, T, Tol, U> {
    pub tolerance: Tol,
    pub h_init: T,
    pub h_max: T,
    pub fac_min: U,
    pub fac_max: U,
    pub fac: U,
    pub n_max: u32,
    pub _phantom: std::marker::PhantomData<C>,
}

impl<C, T, Tol, U> NewMethod<AdaptiveMethodParams<T, Tol, U>> for AdaptiveRungeKutta<C, T, Tol, U>
where
    T: Default,
{
    #[inline]
    fn new(params: AdaptiveMethodParams<T, Tol, U>) -> Self {
        Self {
            h_init: params.h_init,
            h_max: params.h_max,
            tolerance: params.tol,
            fac_min: params.fac_min,
            fac_max: params.fac_max,
            fac: params.fac,
            n_max: params.n_max,
            _phantom: std::marker::PhantomData,
        }
    }
}

pub trait RKEmbedded<T> {
    type State;

    const LOWER_ORDER: u16;

    fn error(&self, h: T, error: &mut Self::State);

    fn undo_step(&mut self, prev: &Self);
}

pub trait Tolerance<V> {
    type Output;

    /// Returns the value of the normalized error Eₙ = norm(eₙ / tol)
    fn err_over_tol(&mut self, state: &V, err: &V) -> Self::Output;
}

impl<V, U, F> Tolerance<V> for F
where
    F: FnMut(&V, &V) -> U,
{
    type Output = U;

    #[inline]
    fn err_over_tol(&mut self, state: &V, err: &V) -> Self::Output {
        self(state, err)
    }
}

/// Used for adaptive step size control
#[derive(Clone, Copy, Debug)]
pub struct IController<T, U> {
    fac_min: U,
    fac_max: U,
    fac: U,
    h_max: T,
}

impl<T, U> Default for IController<T, U>
where
    T: UpperBounded,
    U: Div<Output = U> + From<u16>,
{
    #[inline]
    fn default() -> Self {
        Self {
            fac_min: U::from(2) / U::from(10),
            fac_max: U::from(5),
            fac: U::from(9) / U::from(10),
            h_max: T::max_value(),
        }
    }
}

impl<T, U> IController<T, U> {
    #[inline]
    pub fn new(fac_min: U, fac_max: U, safety_factor: U, h_max: T) -> Self {
        Self {
            fac_min,
            fac_max,
            fac: safety_factor,
            h_max,
        }
    }

    #[inline]
    pub fn step(&mut self, err: U, h: &mut T, order: u16) -> bool
    where
        U: Pow<U, Output = U>
            + Mul<Output = U>
            + Neg<Output = U>
            + Inv<Output = U>
            + PartialOrd
            + From<u16>
            + One
            + Copy,
        T: Mul<U, Output = T> + PartialOrd + Copy,
    {
        let k = U::from(order);
        let m = self.fac * err.pow(-k.inv());
        let nh = *h * clamp(m, self.fac_min, self.fac_max);
        *h = clamp_max(nh, self.h_max);

        err <= U::one()
    }
}

#[derive(Debug, Clone, Copy)]
struct PreviousStep<R, T, V, Tol> {
    t: T,
    y: V,
    rk: R,
    tol: Tol,
}

impl<R, T, V, Tol> PreviousStep<R, T, V, Tol>
where
    R: RKEmbedded<T, State = V>,
{
    #[inline]
    fn store<P>(&mut self, problem: &P, rk: &R, tol: &Tol)
    where
        P: Problem<Time = T, State = V>,
        T: Clone,
        V: Clone,
        Tol: Clone,
    {
        self.t.clone_from(&problem.as_ref().time);
        self.y.clone_from(&problem.as_ref().state);
        self.rk.undo_step(rk);
        self.tol.clone_from(tol);
    }

    #[inline]
    fn restore<P>(&self, problem: &mut P, rk: &mut R, tol: &mut Tol)
    where
        P: Problem<Time = T, State = V>,
        T: Clone,
        V: Clone,
        Tol: Clone,
    {
        problem.as_mut().time.clone_from(&self.t);
        problem.as_mut().state.clone_from(&self.y);
        rk.undo_step(&self.rk);
        tol.clone_from(&self.tol);
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdaptiveRungeKuttaIntegrator<R, T, Tol, U, V = <R as RKEmbedded<T>>::State> {
    frk: FixedRungeKuttaIntegrator<R, T>,
    next_h: T,
    error: V,
    prev: PreviousStep<R, T, V, Tol>,
    tol: Tol,
    controller: IController<T, U>,
    n: u32,
    pub n_max: u32,
}

impl<C, P, Tol, U> Method<P> for AdaptiveRungeKutta<C, P::Time, Tol, U>
where
    P: Problem,
    P::Time: Copy,
    P::State: Clone,
    C: RKCoefficients<P>,
    C::Instance: RKInstance<P> + RKEmbedded<P::Time, State = P::State>,
    Tol: Clone,
{
    type Integrator = AdaptiveRungeKuttaIntegrator<C::Instance, P::Time, Tol, U>;

    #[inline]
    fn init(self, problem: &P) -> Self::Integrator {
        AdaptiveRungeKuttaIntegrator {
            frk: FixedRungeKutta::<C, _>::new(FixedMethodParams::new(self.h_init)).init(problem),
            next_h: self.h_init,
            error: problem.as_ref().state.clone(),
            prev: PreviousStep {
                t: problem.as_ref().time,
                y: problem.as_ref().state.clone(),
                rk: C::Instance::from_problem(problem),
                tol: self.tolerance.clone(),
            },
            tol: self.tolerance.clone(),
            controller: IController::new(self.fac_min, self.fac_max, self.fac, self.h_max),
            n: 0,
            n_max: self.n_max,
        }
    }
}

impl<R, T, Tol, U, V> IntegratorState for AdaptiveRungeKuttaIntegrator<R, T, Tol, U, V>
where
    R: RKState,
    T: Copy,
{
    type Time = T;

    #[inline]
    fn step_size(&self) -> Self::Time {
        self.frk.h
    }

    #[inline]
    fn step_count(&self) -> u32 {
        self.frk.rk.step_count()
    }
}

pub trait AdaptiveRKState {
    type Tolerance;

    fn tolerance(&self) -> &Self::Tolerance;

    fn n(&self) -> u32;

    fn n_max(&self) -> u32;

    fn set_n_max(&mut self, n_max: u32);
}

impl<T, R, Tol, U, V> AdaptiveRKState for AdaptiveRungeKuttaIntegrator<T, R, Tol, U, V> {
    type Tolerance = Tol;

    #[inline]
    fn tolerance(&self) -> &Self::Tolerance {
        &self.tol
    }

    #[inline]
    fn n(&self) -> u32 {
        self.n
    }

    #[inline]
    fn n_max(&self) -> u32 {
        self.n_max
    }

    #[inline]
    fn set_n_max(&mut self, n_max: u32) {
        self.n_max = n_max;
    }
}

impl<R, P, Tol, U> Integrator<P> for AdaptiveRungeKuttaIntegrator<R, P::Time, Tol, U>
where
    P: Problem,
    P::Time: Add<Output = P::Time> + Sub<Output = P::Time> + PartialOrd + Copy,
    P::Time: Mul<U, Output = P::Time>,
    P::State: Clone,
    R: RKInstance<P> + RKEmbedded<P::Time, State = P::State>,
    Tol: Tolerance<P::State, Output = U> + Clone,
    U: Pow<U, Output = U>
        + Mul<Output = U>
        + Neg<Output = U>
        + Inv<Output = U>
        + PartialOrd
        + From<u16>
        + One
        + Copy,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        self.prev.store(problem, &self.frk.rk, &self.tol);

        loop {
            if self.n > self.n_max {
                return Err(StepError::MaxIterationsReached);
            }

            if problem.as_ref().time + self.next_h > problem.as_ref().bound {
                self.next_h = problem.as_ref().bound - problem.as_ref().time;
            }

            self.frk.h = self.next_h;
            self.frk.advance(problem)?;
            self.n += 1;

            self.frk.rk.error(self.frk.h, &mut self.error);
            let err = self.tol.err_over_tol(&problem.as_ref().state, &self.error);
            match self.controller.step(err, &mut self.next_h, R::LOWER_ORDER) {
                true => break,
                false => self.prev.restore(problem, &mut self.frk.rk, &mut self.tol),
            }
        }

        Ok(())
    }
}
