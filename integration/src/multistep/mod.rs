use crate::{
    EvalFailed, FixedIntegrator, Integrator, IntegratorState, Method, NewMethod, Problem, StepError,
};

pub mod buffer;
pub mod first_order;
pub mod second_order;

pub use first_order::*;
pub use second_order::*;

pub trait LMCoefficients<P> {
    type Instance;
}

pub trait LMState {
    fn step_count(&self) -> u32;
}

pub trait LMInstance<P>: LMState {
    const ORDER: usize;

    fn from_problem(problem: &P) -> Self;

    fn advance(&mut self, h: f64, problem: &mut P) -> Result<(), EvalFailed>;

    fn advance_with<E, F>(&mut self, problem: &mut P, f: F) -> Result<(), E>
    where
        F: FnMut(&mut P) -> Result<(), E>,
        E: From<EvalFailed>;
}

/// A wrapper that subdivides the integration step for another integrator.
///
/// A `Substepper` takes an existing integrator `S` and executes a fixed number of sub-steps for
/// every single call to its `step` method. The number of sub-steps is determined by `SUBSTEPS`.
///
/// This is useful for the initialisation phase of multistep methods since they require several
/// initial points. This wrapper may be used with self-starting integrators in order to generate
/// these initial points with a smaller step size and more accurate results.
#[derive(Clone, Copy, Debug)]
pub struct Substepper<const SUBSTEPS: u32, S>(pub S);

impl<const SUBSTEPS: u32, S> NewMethod<f64> for Substepper<SUBSTEPS, S>
where
    S: NewMethod<f64>,
{
    #[inline]
    fn new(h: f64) -> Self {
        Substepper(S::new(h / SUBSTEPS as f64))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SubstepperIntegrator<const SUBSTEPS: u32, S>(S);

impl<P, const SUBSTEPS: u32, S> Method<P> for Substepper<SUBSTEPS, S>
where
    P: Problem,
    S: Method<P>,
{
    type Integrator = SubstepperIntegrator<SUBSTEPS, S::Integrator>;

    #[inline]
    fn init(&self, problem: &P) -> Self::Integrator {
        SubstepperIntegrator(self.0.init(problem))
    }
}

impl<const SUBSTEPS: u32, S> IntegratorState for SubstepperIntegrator<SUBSTEPS, S>
where
    S: IntegratorState,
{
    #[inline]
    fn step_size(&self) -> f64 {
        self.0.step_size() * SUBSTEPS as f64
    }

    #[inline]
    fn step_count(&self) -> u32 {
        self.0.step_count() / SUBSTEPS
    }
}

impl<P, const SUBSTEPS: u32, S> Integrator<P> for SubstepperIntegrator<SUBSTEPS, S>
where
    S: Integrator<P>,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        for _ in 0..SUBSTEPS {
            self.0.step(problem)?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LinearMultistep<C, S> {
    pub h: f64,
    pub starter: S,
    pub _phantom: std::marker::PhantomData<C>,
}

impl<C, S> NewMethod<f64> for LinearMultistep<C, S>
where
    S: NewMethod<f64>,
{
    #[inline]
    fn new(h: f64) -> Self {
        Self {
            h,
            starter: S::new(h),
            _phantom: std::marker::PhantomData,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LinearMultistepIntegrator<L, S> {
    h: f64,
    lm: L,
    starter: S,
}

impl<P, C, S> Method<P> for LinearMultistep<C, S>
where
    P: Problem,
    C: LMCoefficients<P>,
    C::Instance: LMInstance<P>,
    S: Method<P>,
{
    type Integrator = LinearMultistepIntegrator<C::Instance, S::Integrator>;

    #[inline]
    fn init(&self, problem: &P) -> Self::Integrator {
        LinearMultistepIntegrator {
            h: self.h,
            lm: C::Instance::from_problem(problem),
            starter: self.starter.init(problem),
        }
    }
}

impl<L, S> IntegratorState for LinearMultistepIntegrator<L, S>
where
    L: LMState,
    S: IntegratorState,
{
    #[inline]
    fn step_size(&self) -> f64 {
        self.h
    }

    #[inline]
    fn step_count(&self) -> u32 {
        self.starter.step_count() + self.lm.step_count()
    }

    #[inline]
    fn step_count_until_bound_hint(&self, time: f64, bound: f64) -> (usize, Option<usize>) {
        let step_count = self.step_count_until_bound(time, bound);

        (step_count, Some(step_count))
    }
}

impl<L: LMState, S: IntegratorState> FixedIntegrator for LinearMultistepIntegrator<L, S> {}

impl<P, L, S> Integrator<P> for LinearMultistepIntegrator<L, S>
where
    P: Problem,
    L: LMInstance<P>,
    S: IntegratorState + Integrator<P>,
{
    #[inline]
    fn step(&mut self, problem: &mut P) -> Result<(), StepError> {
        if problem.as_ref().time >= problem.as_ref().bound {
            return Err(StepError::BoundReached);
        }

        if self.h.abs() <= f64::EPSILON * 10.0 {
            return Err(StepError::StepSizeUnderflow);
        }

        if self.starter.step_count() < L::ORDER as u32 {
            if self.starter.step_count() == 0 {
                self.lm.advance_with(problem, |_| Ok::<_, EvalFailed>(()))?;
            }

            self.lm.advance_with(problem, |p| self.starter.step(p))?;

            return Ok(());
        }

        self.lm.advance(self.h, problem)?;

        Ok(())
    }
}
