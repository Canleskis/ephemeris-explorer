pub mod multistep;
pub mod runge_kutta;

pub mod methods;

use std::ops::{Add, Mul};

pub trait VariableOps: Copy + Default + Add<Self, Output = Self> + Mul<f64, Output = Self> {}
impl<T: Copy + Default + Add<T, Output = T> + Mul<f64, Output = T>> VariableOps for T {}

pub trait State: Sized {
    type Variable: VariableOps;

    type Iterator<'a>: Iterator<Item = &'a Self::Variable>
    where
        Self: 'a;

    type IteratorMut<'a>: Iterator<Item = &'a mut Self::Variable>
    where
        Self: 'a;

    fn iter(&self) -> Self::Iterator<'_>;

    fn iter_mut(&mut self) -> Self::IteratorMut<'_>;

    #[inline]
    fn fill_with(&mut self, value: Self::Variable) -> &mut Self {
        for var in self.iter_mut() {
            *var = value;
        }
        self
    }

    #[inline]
    fn filled_with(mut self, value: Self::Variable) -> Self {
        self.fill_with(value);
        self
    }

    #[inline]
    fn zero(&mut self) -> &mut Self {
        self.fill_with(Default::default())
    }

    #[inline]
    fn zeroed(self) -> Self {
        self.filled_with(Default::default())
    }
}

impl<T, Item> State for T
where
    Item: VariableOps,
    for<'a> &'a T: IntoIterator<Item = &'a Item>,
    for<'a> &'a mut T: IntoIterator<Item = &'a mut Item>,
{
    type Variable = Item;

    type Iterator<'a>
        = <&'a T as IntoIterator>::IntoIter
    where
        Self: 'a;

    type IteratorMut<'a>
        = <&'a mut T as IntoIterator>::IntoIter
    where
        Self: 'a;

    #[inline]
    fn iter(&self) -> Self::Iterator<'_> {
        self.into_iter()
    }

    #[inline]
    fn iter_mut(&mut self) -> Self::IteratorMut<'_> {
        self.into_iter()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SecondOrderState<V> {
    pub y: V,
    pub dy: V,
}

impl<V> SecondOrderState<V> {
    #[inline]
    pub fn new(y: V, dy: V) -> Self {
        Self { y, dy }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ODEProblem<V, ODE> {
    pub time: f64,
    pub bound: f64,
    pub state: V,
    pub ode: ODE,
}

impl<V, ODE> ODEProblem<V, ODE> {
    #[inline]
    pub fn new(time: f64, bound: f64, state: V, ode: ODE) -> Self {
        Self {
            time,
            bound,
            state,
            ode,
        }
    }
}

pub trait Problem:
    AsRef<ODEProblem<Self::State, Self::ODE>> + AsMut<ODEProblem<Self::State, Self::ODE>>
{
    type State;

    type ODE;
}
impl<P> Problem for &mut P
where
    P: Problem,
{
    type State = P::State;

    type ODE = P::ODE;
}

impl<V, ODE> AsRef<ODEProblem<V, ODE>> for ODEProblem<V, ODE> {
    #[inline]
    fn as_ref(&self) -> &ODEProblem<V, ODE> {
        self
    }
}

impl<V, ODE> AsMut<ODEProblem<V, ODE>> for ODEProblem<V, ODE> {
    #[inline]
    fn as_mut(&mut self) -> &mut ODEProblem<V, ODE> {
        self
    }
}

impl<V, ODE> Problem for ODEProblem<V, ODE> {
    type State = V;

    type ODE = ODE;
}

#[derive(Debug, Clone)]
pub struct EvalFailed;

/// Trait for first-order ordinary differential equations of the form: `y' = f(t, y)`
pub trait FirstOrderODE<V> {
    fn eval(&mut self, t: f64, y: &V, dy: &mut V) -> Result<(), EvalFailed>;
}

/// Trait for second-order ordinary differential equations of the form: `y'' = f(t, y)`
pub trait SecondOrderODE<V> {
    fn eval(&mut self, t: f64, y: &V, ddy: &mut V) -> Result<(), EvalFailed>;
}

/// Trait for second-order ordinary differential equations of the form: `y'' = f(t, y, y')`
pub trait SecondOrderODEGeneral<V> {
    fn eval(&mut self, t: f64, y: &V, dy: &V, ddy: &mut V) -> Result<(), EvalFailed>;
}

impl<F: FnMut(f64, &V, &mut V) -> Result<(), EvalFailed>, V> FirstOrderODE<V> for F {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, dy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, dy)
    }
}

impl<F: FnMut(f64, &V, &mut V) -> Result<(), EvalFailed>, V> SecondOrderODE<V> for F {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, ddy)
    }
}

impl<F: FnMut(f64, &V, &V, &mut V) -> Result<(), EvalFailed>, V> SecondOrderODEGeneral<V> for F {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, dy: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, dy, ddy)
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

impl From<EvalFailed> for StepError {
    #[inline]
    fn from(_: EvalFailed) -> Self {
        StepError::EvalFailed
    }
}

pub trait IntegratorState {
    fn step_size(&self) -> f64;

    fn step_count(&self) -> u32;

    #[inline]
    fn step_count_until_bound_hint(&self, _time: f64, _bound: f64) -> (usize, Option<usize>) {
        (0, None)
    }
}

pub trait FixedIntegrator: IntegratorState {
    #[inline]
    fn step_count_until_bound(&self, time: f64, bound: f64) -> usize {
        ((bound - time) / self.step_size()) as usize
    }
}

pub trait Integrator<P> {
    fn step(&mut self, problem: &mut P) -> Result<(), StepError>;

    #[inline]
    fn step_until_bound(&mut self, problem: &mut P) -> Result<(), StepError>
    where
        P: Problem,
    {
        loop {
            self.step(problem)?;
            if problem.as_ref().time >= problem.as_ref().bound {
                return Ok(());
            }
        }
    }

    #[inline]
    fn advance<'a>(&mut self, problem: &'a mut P) -> Result<(f64, &'a P::State), StepError>
    where
        P: Problem,
    {
        self.step(problem)?;

        Ok((problem.as_ref().time, &(*problem).as_ref().state))
    }

    #[inline]
    fn solve<'a>(&mut self, problem: &'a mut P) -> Result<(f64, &'a P::State), StepError>
    where
        P: Problem,
    {
        self.step_until_bound(problem)?;

        Ok((problem.as_ref().time, &(*problem).as_ref().state))
    }
}

pub trait Method<P> {
    type Integrator;

    fn init(&self, problem: &P) -> Self::Integrator;

    #[inline]
    fn integrate(&self, problem: P) -> Integration<P, Self>
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
impl<P, S: Method<P>> Method<P> for &S {
    type Integrator = S::Integrator;

    #[inline]
    fn init(&self, problem: &P) -> Self::Integrator {
        S::init(self, problem)
    }
}

pub trait NewMethod<Params> {
    fn new(params: Params) -> Self;
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
    pub fn step_size(&self) -> f64 {
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
    M: Method<P>,
    M::Integrator: FixedIntegrator,
{
    #[inline]
    pub fn step_count_until_bound(&self) -> usize {
        self.integrator
            .step_count_until_bound(self.problem.as_ref().time, self.problem.as_ref().bound)
    }
}

impl<P, M> Integration<P, M>
where
    P: Problem,
    M: Method<P>,
    M::Integrator: Integrator<P>,
{
    #[inline]
    pub fn step(&mut self) -> Result<(), StepError> {
        self.integrator.step(&mut self.problem)
    }

    #[inline]
    pub fn step_until_bound(&mut self) -> Result<(), StepError> {
        self.integrator.step_until_bound(&mut self.problem)
    }

    #[inline]
    pub fn advance(&mut self) -> Result<(f64, &P::State), StepError> {
        self.integrator.advance(&mut self.problem)
    }

    #[inline]
    pub fn solve(&mut self) -> Result<(f64, &P::State), StepError> {
        self.integrator.solve(&mut self.problem)
    }
}

impl<P, M> Iterator for Integration<P, M>
where
    P: Problem,
    P::State: Clone,
    M: Method<P>,
    M::Integrator: IntegratorState + Integrator<P>,
{
    type Item = (f64, P::State);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.advance().ok().map(|(t, state)| (t, state.clone()))
    }

    #[inline]
    fn last(mut self) -> Option<Self::Item> {
        self.solve().ok().map(|(t, state)| (t, state.clone()))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.integrator
            .step_count_until_bound_hint(self.problem.as_ref().time, self.problem.as_ref().bound)
    }
}

impl<P, M> ExactSizeIterator for Integration<P, M>
where
    P: Problem,
    P::State: Clone,
    M: Method<P>,
    M::Integrator: FixedIntegrator + Integrator<P>,
{
    #[inline]
    fn len(&self) -> usize {
        self.integrator
            .step_count_until_bound(self.problem.as_ref().time, self.problem.as_ref().bound)
    }
}

pub mod prelude {
    pub use crate::{
        EvalFailed, FirstOrderODE, FixedIntegrator, Integration, Integrator, IntegratorState,
        Method, NewMethod, ODEProblem, Problem, SecondOrderODE, SecondOrderODEGeneral,
        SecondOrderState, State, StepError,
        methods::*,
        multistep::{LinearMultistep, LinearMultistepIntegrator, Substepper},
        runge_kutta::{
            AdaptiveRKParams, AdaptiveRKState, AdaptiveRungeKutta, AdaptiveRungeKuttaIntegrator,
            ErrorCriterion, FixedRungeKutta, FixedRungeKuttaIntegrator,
        },
    };
}
