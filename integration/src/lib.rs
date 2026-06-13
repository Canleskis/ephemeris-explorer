//! # integration
//!
//! A Rust library for building and using numerical ODE integrators through a trait-driven API. The
//! focus is on flexibility and composability, as such some defaults usually expected from numerical
//! integration libraries might be missing.
//!
//! Generic implementations for the following integration methods are provided:
//!
//! - Runge-Kutta (RK)
//! - Runge-Kutta-Nystrom (RKN)
//! - Generalized Runge-Kutta-Nystrom (RKGN)
//! - Symplectic Runge-Kutta-Nystrom (SRKN)
//! - First-order and second-order explicit linear multistep methods
//!
//! ## API Concepts
//!
//! - The [Problem] trait represents an integration problem by providing access to an
//!   [ODEProblem](problem::ODEProblem), which stores the current time, integration bound, state,
//!   and ODE.
//! - The [Method] trait defines how a numerical method initializes its corresponding integrator for
//!   a given problem.
//! - The [Integrator] trait defines the stepping interface.
//! - The [Integration] struct is a convenience wrapper that owns both a problem and an integrator,
//!   exposing more ergonomic stepping methods.
//! - The [FixedMethodParams] and [AdaptiveMethodParams] structs provide shared configuration for
//!   fixed-step and adaptive-step methods.
//!
//! ## Examples
//!
//! ### Fixed-Step RK4
//!
//! ```
//! use integration::prelude::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // y' = -y, y(0) = 1
//! let ode = |_: f64, y: &[f64; 1], dy: &mut [f64; 1]| {
//!     dy[0] = -y[0];
//!     Ok(())
//! };
//!
//! let problem = ODEProblem::new(0.0_f64, 5.0_f64, [1.0_f64], ode);
//! let expected_y = (-5.0_f64).exp();
//!
//! let method = RK4::new(FixedMethodParams::new(0.001));
//! let mut integration = method.integrate(problem);
//!
//! let (t_end, [y_end]) = integration.solve()?;
//! println!("t = {t_end}, y = {y_end}");
//! println!("Expected y = {expected_y}");
//!
//! assert!((y_end - expected_y).abs() < 1e-6);
//!
//! # Ok(())
//! # }
//! ```
//!
//! ### Adaptive Dormand-Prince 5(4)
//!
//! ```
//! use integration::prelude::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // y' = -y, y(0) = 1
//! let ode = |_: f64, y: &[f64; 1], dy: &mut [f64; 1]| {
//!     dy[0] = -y[0];
//!     Ok(())
//! };
//!
//! // Relative and absolute scalar tolerance model.
//! let tol = |atol, rtol| {
//!     move |state: &[f64; 1], err: &[f64; 1]| err[0].abs() / (atol + rtol * state[0].abs())
//! };
//!
//! let problem = ODEProblem::new(0.0_f64, 5.0_f64, [1.0_f64], ode);
//! let expected_y = (-5.0_f64).exp();
//!
//! let params = AdaptiveMethodParams::new(tol(1e-8, 1e-10), 10_000)
//!     .h_init(1e-3)
//!     .h_max(0.2);
//!
//! let method = DormandPrince54::new(params);
//! let mut integration = method.integrate(problem);
//!
//! let (t_end, [y_end]) = integration.solve()?;
//! println!("t = {t_end}, y = {y_end}");
//! println!("Expected y = {expected_y}");
//!
//! assert!((y_end - expected_y).abs() < 1e-6);
//!
//! # Ok(())
//! # }
//! ```
//!
//! ## Included Methods
//!
//! Aliases exposed under `integration::methods` include:
//!
//! - [`RK4`](methods::RK4)
//! - [`CashKarp45`](methods::CashKarp45)
//! - [`DormandPrince54`](methods::DormandPrince54)
//! - [`DormandPrince87`](methods::DormandPrince87)
//! - [`Fehlberg45`](methods::Fehlberg45)
//! - [`Tsitouras75`](methods::Tsitouras75)
//! - [`Verner87`](methods::Verner87)
//! - [`Verner98`](methods::Verner98)
//! - [`Fine45`](methods::Fine45)
//! - [`Tsitouras75Nystrom`](methods::Tsitouras75Nystrom)
//! - [`BlanesMoan6B`](methods::BlanesMoan6B)
//! - [`BlanesMoan11B`](methods::BlanesMoan11B)
//! - [`BlanesMoan14A`](methods::BlanesMoan14A)
//! - [`ForestRuth`](methods::ForestRuth)
//! - [`McLachlanO4`](methods::McLachlanO4)
//! - [`McLachlanSS17`](methods::McLachlanSS17)
//! - [`Pefrl`](methods::Pefrl)
//! - [`Ruth`](methods::Ruth)
//! - [`AdamsBashforth2`](methods::AdamsBashforth2)
//! - [`AdamsBashforth3`](methods::AdamsBashforth3)
//! - [`AdamsBashforth4`](methods::AdamsBashforth4)
//! - [`AdamsBashforth5`](methods::AdamsBashforth5)
//! - [`AdamsBashforth6`](methods::AdamsBashforth6)
//! - [`QuinlanTremaine12`](methods::QuinlanTremaine12)
//! - [`Stormer13`](methods::Stormer13)
//!
//! For low-level composition and custom coefficients, see the `methods::coeffs`, `runge_kutta`, and `multistep` modules.

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
        Integration::new(problem, self)
    }
}

pub trait NewMethod<Params> {
    fn new(params: Params) -> Self;
}

/// Common parameters for fixed step size methods.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdaptiveMethodParams<T, Tol, U> {
    pub h_init: T,
    pub h_max: T,
    pub tol: Tol,
    pub fac_min: U,
    pub fac_max: U,
    pub fac: U,
    pub n_max: u32,
}

pub const DEFAULT_FAC_MIN: Ratio<u16> = Ratio::new_raw(1, 5);
pub const DEFAULT_FAC_MAX: Ratio<u16> = Ratio::new_raw(5, 1);
pub const DEFAULT_FAC: Ratio<u16> = Ratio::new_raw(9, 10);

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
            DEFAULT_FAC_MIN,
            DEFAULT_FAC_MAX,
            DEFAULT_FAC,
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
    pub fn h_init(mut self, h_init: T) -> Self {
        self.h_init = h_init;
        self
    }

    #[inline]
    pub fn h_max(mut self, h_max: T) -> Self {
        self.h_max = h_max;
        self
    }
}

pub trait IntegratorState {
    const ORDER: u16;

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

pub trait Solout<P> {
    type Solution;

    fn new_solution(&self, problem: &P) -> Self::Solution;

    fn solout(&mut self, problem: &P, solution: &mut Self::Solution) -> bool;
}

impl<P> Solout<P> for () {
    type Solution = ();

    #[inline]
    fn new_solution(&self, _: &P) -> Self::Solution {}

    #[inline]
    fn solout(&mut self, _: &P, _: &mut Self::Solution) -> bool {
        true
    }
}

pub trait Integrator<P> {
    fn advance(&mut self, problem: &mut P) -> Result<(), StepError>;

    #[inline]
    fn advance_to_bound(&mut self, problem: &mut P) -> Result<(), StepError>
    where
        P: Problem,
        P::Time: PartialOrd,
    {
        // If the bound is already reached, `advance` would return an error and so we don't need to
        // check it beforehand.
        loop {
            self.advance(problem)?;
            if problem.as_ref().time >= problem.as_ref().bound {
                return Ok(());
            }
        }
    }

    #[inline]
    fn advance_solution<O>(
        &mut self,
        problem: &mut P,
        solout: &mut O,
        solution: &mut O::Solution,
    ) -> Result<bool, StepError>
    where
        O: Solout<P>,
    {
        self.advance(problem)?;

        Ok(solout.solout(problem, solution))
    }
}

#[derive(Debug)]
pub struct Integration<P, M: Method<P>, O: Solout<P> = ()> {
    pub problem: P,
    pub integrator: M::Integrator,
    pub solout: O,
    pub solution: O::Solution,
}

impl<P, M, O> Clone for Integration<P, M, O>
where
    P: Clone,
    M: Method<P>,
    M::Integrator: Clone,
    O: Solout<P> + Clone,
    O::Solution: Clone,
{
    fn clone(&self) -> Self {
        Self {
            problem: self.problem.clone(),
            integrator: self.integrator.clone(),
            solout: self.solout.clone(),
            solution: self.solution.clone(),
        }
    }
}

impl<P, M> Integration<P, M, ()>
where
    M: Method<P>,
{
    #[inline]
    pub fn new(problem: P, method: M) -> Self {
        Self {
            integrator: method.init(&problem),
            problem,
            solout: (),
            solution: (),
        }
    }

    #[inline]
    pub fn with_solout<O>(self, solout: O) -> Integration<P, M, O>
    where
        O: Solout<P>,
    {
        Integration {
            solution: solout.new_solution(&self.problem),
            problem: self.problem,
            integrator: self.integrator,
            solout,
        }
    }
}

impl<P, M, O> Integration<P, M, O>
where
    M: Method<P>,
    M::Integrator: IntegratorState,
    O: Solout<P>,
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

impl<P, M, O> Integration<P, M, O>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + Copy,
    M: Method<P>,
    M::Integrator: IntegratorState<Time = P::Time>,
    O: Solout<P>,
{
    #[inline]
    pub fn step_count_until_bound_hint(&self) -> (usize, Option<usize>) {
        self.integrator
            .step_count_until_bound_hint(self.problem.as_ref().time - self.problem.as_ref().bound)
    }

    #[inline]
    pub fn step_count_until_bound(&self) -> usize
    where
        P::Time: DivCeil,
        M::Integrator: FixedIntegrator<Time = P::Time>,
    {
        self.integrator
            .step_count_until_bound(self.problem.as_ref().time - self.problem.as_ref().bound)
    }
}

impl<P, M, O> Integration<P, M, O>
where
    M: Method<P>,
    M::Integrator: Integrator<P>,
    O: Solout<P>,
{
    #[inline]
    pub fn advance(&mut self) -> Result<bool, StepError>
    where
        O: Solout<P>,
    {
        self.integrator
            .advance_solution(&mut self.problem, &mut self.solout, &mut self.solution)
    }

    #[inline]
    pub fn advance_to_bound(&mut self) -> Result<(), StepError>
    where
        P: Problem,
        P::Time: PartialOrd,
        O: Solout<P>,
    {
        loop {
            if !self.advance()? || self.problem.as_ref().time >= self.problem.as_ref().bound {
                return Ok(());
            }
        }
    }

    #[inline]
    pub fn solve(mut self) -> Result<O::Solution, StepError>
    where
        P: Problem,
        P::Time: PartialOrd,
        O: Solout<P>,
    {
        self.advance_to_bound()?;

        Ok(self.solution)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EveryStepSolout;

impl<P> Solout<P> for EveryStepSolout
where
    P: Problem,
    P::Time: Clone,
    P::State: Clone,
{
    type Solution = (P::Time, P::State);

    #[inline]
    fn new_solution(&self, problem: &P) -> Self::Solution {
        (
            problem.as_ref().time.clone(),
            problem.as_ref().state.clone(),
        )
    }

    #[inline]
    fn solout(&mut self, problem: &P, (time, state): &mut Self::Solution) -> bool {
        time.clone_from(&problem.as_ref().time);
        state.clone_from(&problem.as_ref().state);

        true
    }
}

impl<P, M> Iterator for Integration<P, M, EveryStepSolout>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + PartialOrd + Copy,
    P::State: Clone,
    M: Method<P>,
    M::Integrator: IntegratorState<Time = P::Time> + Integrator<P>,
{
    type Item = <EveryStepSolout as Solout<P>>::Solution;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.advance().ok().map(|_| self.solution.clone())
    }

    #[inline]
    fn last(self) -> Option<Self::Item> {
        self.solve().ok()
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.step_count_until_bound_hint()
    }
}

impl<P, M, O> ExactSizeIterator for Integration<P, M, O>
where
    P: Problem,
    P::Time: Sub<Output = P::Time> + DivCeil + Copy,
    M: Method<P>,
    M::Integrator: FixedIntegrator<Time = P::Time>,
    O: Solout<P>,
    Integration<P, M, O>: Iterator,
{
    #[inline]
    fn len(&self) -> usize {
        self.step_count_until_bound()
    }
}

pub mod prelude {
    pub use crate::{
        AdaptiveMethodParams, DivCeil, EveryStepSolout, FixedIntegrator, FixedMethodParams,
        Integration, Integrator, IntegratorState, Method, NewMethod, Solout, StepError,
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
