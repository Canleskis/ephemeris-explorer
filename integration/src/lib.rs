pub mod problem;
pub mod ratio;

pub mod multistep;
pub mod runge_kutta;

pub mod methods;

use crate::{problem::Problem, ratio::Ratio};
use num_traits::{bounds::UpperBounded, real::Real};
use std::ops::{Div, Sub};

pub trait Method<P> {
    type Integrator;

    fn init(self, problem: &P) -> Self::Integrator;

    #[inline]
    fn integrate(self, problem: P) -> Integration<P, Self>
    where
        Self: Sized,
        Self::Integrator: Integrator<P>,
    {
        Integration {
            integrator: self.init(&problem),
            problem,
        }
    }
}

/// Common parameters for fixed step size methods.
pub trait NewMethod<Params> {
    fn new(params: Params) -> Self;
}

#[derive(Clone, Copy, Debug)]
pub struct FixedMethodParams<T> {
    pub h: T,
}

impl<T> FixedMethodParams<T> {
    #[inline]
    pub const fn new(h: T) -> Self {
        Self { h }
    }
}

/// Common parameters for adaptive step size methods. Implementations are free to use or ignore any
/// of these parameters.
pub struct AdaptiveMethodParams<T, Tol, U> {
    pub h_init: T,
    pub h_max: T,
    pub tol: Tol,
    pub fac_min: U,
    pub fac_max: U,
    pub fac: U,
    pub n_max: u32,
}

pub const FAC_MIN_DEFAULT: Ratio<u16> = Ratio::new_raw(1, 5);
pub const FAC_MAX_DEFAULT: Ratio<u16> = Ratio::new_raw(5, 1);
pub const FAC_DEFAULT: Ratio<u16> = Ratio::new_raw(9, 10);

impl<T, Tol, U> AdaptiveMethodParams<T, Tol, U> {
    #[inline]
    pub const fn with(
        h_init: T,
        h_max: T,
        tol: Tol,
        fac_min: U,
        fac_max: U,
        fac: U,
        n_max: u32,
    ) -> Self {
        Self {
            h_init,
            h_max,
            tol,
            fac_min,
            fac_max,
            fac,
            n_max,
        }
    }

    #[inline]
    pub fn with_ratios(
        h_init: T,
        h_max: T,
        tol: Tol,
        fac_min: Ratio<u16>,
        fac_max: Ratio<u16>,
        fac: Ratio<u16>,
        n_max: u32,
    ) -> Self
    where
        U: Div<Output = U> + From<u16>,
    {
        Self::with(
            h_init,
            h_max,
            tol,
            U::from(fac_min.numerator()) / U::from(fac_min.denominator()),
            U::from(fac_max.numerator()) / U::from(fac_max.denominator()),
            U::from(fac.numerator()) / U::from(fac.denominator()),
            n_max,
        )
    }

    #[inline]
    pub fn new(tol: Tol, n_max: u32) -> Self
    where
        T: UpperBounded,
        U: Div<Output = U> + From<u16>,
    {
        Self::with_ratios(
            T::max_value(),
            T::max_value(),
            tol,
            FAC_MIN_DEFAULT,
            FAC_MAX_DEFAULT,
            FAC_DEFAULT,
            n_max,
        )
    }

    #[inline]
    pub fn tol(mut self, tol: Tol) -> Self {
        self.tol = tol;
        self
    }

    #[inline]
    pub fn n_max(mut self, n_max: u32) -> Self {
        self.n_max = n_max;
        self
    }

    #[inline]
    pub fn init_h(mut self, init_h: T) -> Self {
        self.h_init = init_h;
        self
    }
}

pub trait IntegratorState {
    type Time;

    fn step_size(&self) -> Self::Time;

    fn step_count(&self) -> u32;

    #[inline]
    fn step_count_until_bound_hint(&self, _duration: Self::Time) -> (usize, Option<usize>) {
        (0, None)
    }
}

pub trait DivCeil {
    fn div_ceil(self, rhs: Self) -> usize;
}

impl<T: Real> DivCeil for T {
    #[inline]
    fn div_ceil(self, rhs: Self) -> usize {
        (self / rhs).ceil().to_usize().unwrap()
    }
}

pub trait FixedIntegrator: IntegratorState
where
    Self::Time: Sub<Output = Self::Time> + DivCeil,
{
    #[inline]
    fn step_count_until_bound(&self, duration: Self::Time) -> usize {
        duration.div_ceil(self.step_size())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepError {
    StepSizeUnderflow,
    MaxIterationsReached,
    BoundReached,
    EvalFailed,
}
impl std::fmt::Display for StepError {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::StepSizeUnderflow => write!(f, "step size underflow"),
            Self::MaxIterationsReached => write!(f, "max iterations reached"),
            Self::BoundReached => write!(f, "integration bound reached"),
            Self::EvalFailed => write!(f, "failed to evaluate ODE"),
        }
    }
}
impl std::error::Error for StepError {}

impl From<problem::EvalFailed> for StepError {
    #[inline]
    fn from(_: problem::EvalFailed) -> Self {
        StepError::EvalFailed
    }
}

pub trait Integrator<P> {
    fn step(&mut self, problem: &mut P) -> Result<(), StepError>;

    #[inline]
    fn step_until_bound(&mut self, problem: &mut P) -> Result<(), StepError>
    where
        P: Problem,
        P::Time: PartialOrd,
    {
        loop {
            self.step(problem)?;
            if problem.as_ref().time >= problem.as_ref().bound {
                return Ok(());
            }
        }
    }

    #[inline]
    fn advance<'a>(&mut self, problem: &'a mut P) -> Result<(&'a P::Time, &'a P::State), StepError>
    where
        P: Problem,
    {
        self.step(problem)?;

        Ok((&(*problem).as_ref().time, &(*problem).as_ref().state))
    }

    #[inline]
    fn solve<'a>(&mut self, problem: &'a mut P) -> Result<(&'a P::Time, &'a P::State), StepError>
    where
        P: Problem,
        P::Time: PartialOrd,
    {
        self.step_until_bound(problem)?;

        Ok((&(*problem).as_ref().time, &(*problem).as_ref().state))
    }
}

#[derive(Debug)]
pub struct Integration<P, M: Method<P>> {
    pub problem: P,
    pub integrator: M::Integrator,
}

impl<P, M> Clone for Integration<P, M>
where
    P: Clone,
    M: Method<P>,
    M::Integrator: Clone,
{
    fn clone(&self) -> Self {
        Self {
            problem: self.problem.clone(),
            integrator: self.integrator.clone(),
        }
    }
}

impl<P, M> Integration<P, M>
where
    M: Method<P>,
{
    #[inline]
    pub fn new(problem: P, method: M) -> Self {
        Self {
            integrator: method.init(&problem),
            problem,
        }
    }
}

impl<P, M> Integration<P, M>
where
    M: Method<P>,
    M::Integrator: IntegratorState,
{
    #[inline]
    pub fn step_size(&self) -> <M::Integrator as IntegratorState>::Time {
        self.integrator.step_size()
    }

    #[inline]
    pub fn step_count(&self) -> u32 {
        self.integrator.step_count()
    }
}

impl<P, M> Integration<P, M>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + Copy,
    M: Method<P>,
    M::Integrator: IntegratorState<Time = P::Time>,
{
    #[inline]
    pub fn step_count_until_bound_hint(&self) -> (usize, Option<usize>) {
        self.integrator
            .step_count_until_bound_hint(self.problem.as_ref().time - self.problem.as_ref().bound)
    }
}

impl<P, M> Integration<P, M>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + DivCeil + Copy,
    M: Method<P>,
    M::Integrator: FixedIntegrator<Time = P::Time>,
{
    #[inline]
    pub fn step_count_until_bound(&self) -> usize {
        self.integrator
            .step_count_until_bound(self.problem.as_ref().time - self.problem.as_ref().bound)
    }
}

impl<P, M> Integration<P, M>
where
    M: Method<P>,
    M::Integrator: Integrator<P>,
{
    #[inline]
    pub fn step(&mut self) -> Result<(), StepError> {
        self.integrator.step(&mut self.problem)
    }
}

impl<P, M> Integration<P, M>
where
    P: Problem,
    M: Method<P>,
    M::Integrator: Integrator<P>,
{
    #[inline]
    pub fn advance(&mut self) -> Result<(&P::Time, &P::State), StepError> {
        self.integrator.advance(&mut self.problem)
    }
}

impl<P, M> Integration<P, M>
where
    P: Problem,
    P::Time: PartialOrd,
    M: Method<P>,
    M::Integrator: Integrator<P>,
{
    #[inline]
    pub fn step_until_bound(&mut self) -> Result<(), StepError> {
        self.integrator.step_until_bound(&mut self.problem)
    }

    #[inline]
    pub fn solve(&mut self) -> Result<(&P::Time, &P::State), StepError> {
        self.integrator.solve(&mut self.problem)
    }
}

impl<P, M> Iterator for Integration<P, M>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + PartialOrd + Copy,
    P::State: Clone,
    M: Method<P>,
    M::Integrator: IntegratorState<Time = P::Time> + Integrator<P>,
{
    type Item = (P::Time, P::State);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.advance().ok().map(|(t, state)| (*t, state.clone()))
    }

    #[inline]
    fn last(mut self) -> Option<Self::Item> {
        self.solve().ok().map(|(t, state)| (*t, state.clone()))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.step_count_until_bound_hint()
    }
}

impl<P, M> ExactSizeIterator for Integration<P, M>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + DivCeil + Copy,
    M: Method<P>,
    M::Integrator: FixedIntegrator<Time = P::Time>,
    Integration<P, M>: Iterator,
{
    #[inline]
    fn len(&self) -> usize {
        self.step_count_until_bound()
    }
}

pub mod prelude {
    pub use crate::{
        AdaptiveMethodParams, DivCeil, FixedIntegrator, FixedMethodParams, Integration, Integrator,
        IntegratorState, Method, NewMethod, StepError,
        methods::*,
        multistep::{LinearMultistep, LinearMultistepIntegrator, Substepper},
        problem::{
            EvalFailed, FirstOrderODE, ODEProblem, Problem, SecondOrderODE, SecondOrderODEGeneral,
            SecondOrderState,
        },
        ratio::Ratio,
        runge_kutta::{
            AdaptiveRKState, AdaptiveRungeKutta, AdaptiveRungeKuttaIntegrator, FixedRungeKutta,
            FixedRungeKuttaIntegrator, Tolerance,
        },
    };
}
