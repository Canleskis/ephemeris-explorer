pub mod explicit;
pub mod nystrom;

pub use explicit::*;
pub use nystrom::explicit::*;
pub use nystrom::explicit_generalized::*;
pub use nystrom::symplectic::*;

use crate::{
    EvalFailed, FixedIntegrator, Integrator, IntegratorState, Method, NewMethod, Problem, StepError,
};

pub trait RKCoefficients<P> {
    type Instance;
}

pub trait RKState {
    fn step_count(&self) -> u32;
}

pub trait RKInstance<P> {
    fn from_problem(problem: &P) -> Self;

    fn advance(&mut self, h: f64, problem: &mut P) -> Result<(), EvalFailed>;
}

#[derive(Clone, Copy, Debug)]
pub struct FixedRungeKutta<C> {
    pub h: f64,
    pub _phantom: std::marker::PhantomData<C>,
}

impl<C> NewMethod<f64> for FixedRungeKutta<C> {
    #[inline]
    fn new(params: f64) -> Self {
        Self {
            h: params,
            _phantom: std::marker::PhantomData,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FixedRungeKuttaIntegrator<R> {
    pub h: f64,
    pub rk: R,
}

impl<P, C> Method<P> for FixedRungeKutta<C>
where
    P: Problem,
    C: RKCoefficients<P>,
    C::Instance: RKInstance<P>,
{
    type Integrator = FixedRungeKuttaIntegrator<C::Instance>;

    #[inline]
    fn init(&self, problem: &P) -> Self::Integrator {
        FixedRungeKuttaIntegrator {
            h: self.h,
            rk: C::Instance::from_problem(problem),
        }
    }
}

impl<R> IntegratorState for FixedRungeKuttaIntegrator<R>
where
    R: RKState,
{
    #[inline]
    fn step_size(&self) -> f64 {
        self.h
    }

    #[inline]
    fn step_count(&self) -> u32 {
        self.rk.step_count()
    }

    #[inline]
    fn step_count_until_bound_hint(&self, time: f64, bound: f64) -> (usize, Option<usize>) {
        let step_count = self.step_count_until_bound(time, bound);

        (step_count, Some(step_count))
    }
}

impl<R: RKState> FixedIntegrator for FixedRungeKuttaIntegrator<R> {}

impl<P, R> Integrator<P> for FixedRungeKuttaIntegrator<R>
where
    P: Problem,
    R: RKInstance<P>,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        if problem.as_ref().time >= problem.as_ref().bound {
            return Err(StepError::BoundReached);
        }

        if self.h.abs() <= f64::EPSILON * 10.0 {
            return Err(StepError::StepSizeUnderflow);
        }

        self.rk.advance(self.h, problem)?;

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AdaptiveRKParams<T> {
    pub tol: T,
    pub n_max: u32,
    pub init_h: f64,
}

impl<T> AdaptiveRKParams<T> {
    #[inline]
    pub const fn new(tol: T, n_max: u32) -> Self {
        Self {
            tol,
            n_max,
            init_h: f64::MAX,
        }
    }

    #[inline]
    pub fn tol(mut self, tol: T) -> Self {
        self.tol = tol;
        self
    }

    #[inline]
    pub fn n_max(mut self, n_max: u32) -> Self {
        self.n_max = n_max;
        self
    }

    #[inline]
    pub fn init_h(mut self, init_h: f64) -> Self {
        self.init_h = init_h;
        self
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveRungeKutta<C, T> {
    pub tolerance: T,
    pub n_max: u32,
    pub init_h: f64,
    pub _phantom: std::marker::PhantomData<C>,
}

impl<C, T> NewMethod<AdaptiveRKParams<T>> for AdaptiveRungeKutta<C, T> {
    #[inline]
    fn new(params: AdaptiveRKParams<T>) -> Self {
        Self {
            tolerance: params.tol,
            n_max: params.n_max,
            init_h: params.init_h,
            _phantom: std::marker::PhantomData,
        }
    }
}

pub trait RKEmbedded {
    type State;

    const LOWER_ORDER: u32;

    fn error(&self, h: f64, error: &mut Self::State);

    fn undo_step(&mut self, prev: &Self);
}

pub trait ErrorCriterion<V> {
    /// Returns the value of the normalized error Eₙ = norm(eₙ / tol)
    fn err_over_tol(&mut self, state: &V, err: &V) -> f64;
}

impl<V, F> ErrorCriterion<V> for F
where
    F: FnMut(&V, &V) -> f64,
{
    #[inline]
    fn err_over_tol(&mut self, state: &V, err: &V) -> f64 {
        self(state, err)
    }
}

/// Used for adaptive step size control
#[derive(Debug, Clone, Copy)]
pub struct Controller {
    alpha: f64,
    fac_min: f64,
    fac_max: f64,
    fac: f64,
    h_max: f64,
}

impl Controller {
    #[inline]
    pub fn new(alpha: f64, fac_min: f64, fac_max: f64, safety_factor: f64, h_max: f64) -> Self {
        Self {
            alpha,
            fac_min,
            fac_max,
            fac: safety_factor,
            h_max,
        }
    }

    #[inline]
    pub fn step(&mut self, err: f64, h: &mut f64) -> bool {
        let m = self.fac * err.powf(-self.alpha);
        // When the error is NaN, it is likely because the step is too large. As such, the order
        // between max and min here is important so that the step size is reduced as expected.
        *h = self.h_max.min(*h * m.max(self.fac_min).min(self.fac_max));

        err <= 1.0
    }
}

#[derive(Debug, Clone, Copy)]
struct PreviousStep<R, V> {
    t: f64,
    y: V,
    rk: R,
}

impl<R, V> PreviousStep<R, V>
where
    R: RKEmbedded<State = V>,
{
    #[inline]
    fn store<P>(&mut self, problem: &P, rk: &R)
    where
        P: Problem<State = V>,
        V: Clone,
    {
        self.t = problem.as_ref().time;
        self.y.clone_from(&problem.as_ref().state);
        self.rk.undo_step(rk);
    }

    #[inline]
    fn restore<P>(&self, problem: &mut P, rk: &mut R)
    where
        P: Problem<State = V>,
        V: Clone,
    {
        problem.as_mut().time = self.t;
        problem.as_mut().state.clone_from(&self.y);
        rk.undo_step(&self.rk);
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdaptiveRungeKuttaIntegrator<R, T, V = <R as RKEmbedded>::State> {
    frk: FixedRungeKuttaIntegrator<R>,
    error: V,
    previous_step: PreviousStep<R, V>,
    previous_h: f64,
    tol: T,
    controller: Controller,
    n: u32,
    pub n_max: u32,
}

impl<P, C, T> Method<P> for AdaptiveRungeKutta<C, T>
where
    P: Problem,
    P::State: Clone,
    C: RKCoefficients<P>,
    C::Instance: RKInstance<P> + RKEmbedded<State = P::State>,
    T: Clone,
{
    type Integrator = AdaptiveRungeKuttaIntegrator<C::Instance, T>;

    #[inline]
    fn init(&self, problem: &P) -> Self::Integrator {
        AdaptiveRungeKuttaIntegrator {
            frk: FixedRungeKutta::<C>::new(self.init_h).init(problem),
            error: problem.as_ref().state.clone(),
            previous_step: PreviousStep {
                t: problem.as_ref().time,
                y: problem.as_ref().state.clone(),
                rk: C::Instance::from_problem(problem),
            },
            previous_h: self.init_h,
            tol: self.tolerance.clone(),
            controller: Controller::new(
                1.0 / C::Instance::LOWER_ORDER as f64,
                0.2,
                5.0,
                0.9,
                f64::MAX,
            ),
            n: 0,
            n_max: self.n_max,
        }
    }
}

impl<R, T, V> IntegratorState for AdaptiveRungeKuttaIntegrator<R, T, V>
where
    R: RKState,
{
    #[inline]
    fn step_size(&self) -> f64 {
        self.previous_h
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

impl<R, T, V> AdaptiveRKState for AdaptiveRungeKuttaIntegrator<R, T, V> {
    type Tolerance = T;

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

impl<P, R, T> Integrator<P> for AdaptiveRungeKuttaIntegrator<R, T>
where
    P: Problem,
    P::State: Clone,
    R: RKInstance<P> + RKEmbedded<State = P::State>,
    T: ErrorCriterion<P::State>,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        self.previous_step.store(problem, &self.frk.rk);

        loop {
            if self.n > self.n_max {
                return Err(StepError::MaxIterationsReached);
            }

            if problem.as_ref().time + self.frk.h > problem.as_ref().bound {
                self.frk.h = problem.as_ref().bound - problem.as_ref().time;
            }

            self.previous_h = self.frk.h;

            self.frk.advance(problem)?;
            self.n += 1;

            self.frk.rk.error(self.frk.h, &mut self.error);
            let err = self.tol.err_over_tol(&problem.as_ref().state, &self.error);
            match self.controller.step(err, &mut self.frk.h) {
                true => break,
                false => self.previous_step.restore(problem, &mut self.frk.rk),
            }
        }

        Ok(())
    }
}
