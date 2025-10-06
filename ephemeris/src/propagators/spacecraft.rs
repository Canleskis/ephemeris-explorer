use crate::{
    BranchingPropagator, DirectionalPropagator, IncrementalPropagator, Iterable, Propagator,
    trajectory::{BoundedTrajectory, CubicHermiteSplineSamples, EvaluateTrajectory, StateVector},
};

use ftime::{Duration, Epoch};
use integration::prelude::*;
use num_traits::bounds::UpperBounded;
use std::ops::Div;

pub trait AccelerationModel {
    type Vector;

    fn acceleration(&self, t: Epoch, state: &StateVector<Self::Vector>) -> Option<Self::Vector>;
}

pub trait PropagationContext {
    fn max_time(&self) -> Epoch;
}

pub trait Transform<V> {
    fn to_inertial(&self, v: &V) -> V;
}

pub trait ManoeuvreFrame<V> {
    type Transform;

    fn transform(&self, t: Epoch, sv: &StateVector<V>) -> Option<Self::Transform>;
}

#[derive(Clone, Default)]
pub enum ReferenceKind<T> {
    /// Relative to a reference trajectory.
    Relative(T),
    #[default]
    Inertial,
}

#[derive(Clone)]
pub struct ReferenceFrame<T, F> {
    reference: ReferenceKind<T>,
    _frame: std::marker::PhantomData<F>,
}

impl<T, F> ReferenceFrame<T, F> {
    #[inline]
    pub fn relative(trajectory: T) -> Self {
        Self {
            reference: ReferenceKind::Relative(trajectory),
            _frame: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn inertial() -> Self {
        Self {
            reference: ReferenceKind::Inertial,
            _frame: std::marker::PhantomData,
        }
    }
}

impl<T, F> ManoeuvreFrame<T::Vector> for ReferenceFrame<T, F>
where
    T: EvaluateTrajectory,
    T::Vector: std::ops::Sub<Output = T::Vector> + Copy,
    F: From<StateVector<T::Vector>> + Default,
{
    type Transform = F;

    #[inline]
    fn transform(&self, t: Epoch, sv: &StateVector<T::Vector>) -> Option<Self::Transform> {
        match &self.reference {
            ReferenceKind::Relative(reference) => Some(F::from(*sv - reference.state_vector(t)?)),
            ReferenceKind::Inertial => Some(F::default()),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ConstantThrust<V, F> {
    pub start: Epoch,
    pub duration: Duration,
    pub acceleration: V,
    pub frame: F,
}

impl<V, F> ConstantThrust<V, F> {
    #[inline]
    pub fn new(start: Epoch, duration: Duration, acceleration: V, frame: F) -> Self {
        Self {
            start,
            duration,
            acceleration,
            frame,
        }
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        self.start + self.duration
    }
}

impl<V, F> AccelerationModel for ConstantThrust<V, F>
where
    F: ManoeuvreFrame<V>,
    F::Transform: Transform<V>,
{
    type Vector = V;

    #[inline]
    fn acceleration(&self, t: Epoch, sv: &StateVector<Self::Vector>) -> Option<Self::Vector> {
        // We leave time bounds checking as a responsibility of the caller
        Some(self.frame.transform(t, sv)?.to_inertial(&self.acceleration))
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ManoeuvreEvent<V, F> {
    Start {
        time: Epoch,
        manoeuvre: ConstantThrust<V, F>,
    },
    Stop {
        time: Epoch,
    },
}

impl<V, F> ManoeuvreEvent<V, F> {
    #[inline]
    pub fn time(&self) -> Epoch {
        match self {
            Self::Start { time, .. } | Self::Stop { time } => *time,
        }
    }

    #[inline]
    pub fn manoeuvre(&self) -> Option<&ConstantThrust<V, F>> {
        match self {
            Self::Start { manoeuvre, .. } => Some(manoeuvre),
            Self::Stop { .. } => None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ManoeuvreSchedule<V, F> {
    events: Vec<ManoeuvreEvent<V, F>>,
    idx: usize,
}

impl<V, F> Default for ManoeuvreSchedule<V, F> {
    #[inline]
    fn default() -> Self {
        Self {
            events: vec![ManoeuvreEvent::Stop {
                time: Epoch::from_offset(Duration::MIN),
            }],
            idx: 0,
        }
    }
}

impl<V, F> ManoeuvreSchedule<V, F> {
    #[inline]
    pub fn binary_search(&self, time: Epoch) -> Result<usize, usize> {
        self.events
            .binary_search_by(|event| event.time().cmp(&time))
    }

    #[inline]
    fn last_event_at_idx(&self, time: Epoch) -> usize {
        self.binary_search(time)
            .unwrap_or_else(|i| i.saturating_sub(1))
    }

    #[inline]
    fn last_event_at_exclusive_idx(&self, time: Epoch) -> usize {
        self.binary_search(time)
            .unwrap_or_else(|i| i)
            .saturating_sub(1)
    }

    #[inline]
    fn next_event_at_idx(&self, time: Epoch) -> Option<usize> {
        self.binary_search(time)
            .map(|i| i + 1)
            .map_or_else(Some, Some)
            .filter(|&i| i < self.events.len())
    }

    #[inline]
    pub fn last_event_at(&self, time: Epoch) -> &ManoeuvreEvent<V, F> {
        &self.events[self.last_event_at_idx(time)]
    }

    #[inline]
    pub fn last_event_at_exclusive(&self, time: Epoch) -> &ManoeuvreEvent<V, F> {
        &self.events[self.last_event_at_exclusive_idx(time)]
    }

    #[inline]
    pub fn next_event_at(&self, time: Epoch) -> Option<&ManoeuvreEvent<V, F>> {
        self.next_event_at_idx(time)
            .and_then(|i| self.events.get(i))
    }

    /// Updates the schedule to the current time and returns true if the current manoeuvre has
    /// changed.
    #[inline]
    pub fn update(&mut self, time: Epoch) -> bool {
        let old_idx = self.idx;
        self.idx = self.last_event_at_idx(time);

        old_idx != self.idx
    }

    #[inline]
    pub fn current(&self) -> &ManoeuvreEvent<V, F> {
        &self.events[self.idx]
    }

    #[inline]
    pub fn next_event(&self) -> Option<&ManoeuvreEvent<V, F>> {
        self.events.get(self.idx + 1)
    }

    #[inline]
    pub fn next_event_time(&self) -> Option<Epoch> {
        self.events.get(self.idx + 1).map(ManoeuvreEvent::time)
    }

    #[inline]
    pub fn insert(&mut self, manoeuvre: ConstantThrust<V, F>) {
        let index = self.binary_search(manoeuvre.start).unwrap_or_else(|i| i);
        self.events.insert(
            index,
            ManoeuvreEvent::Stop {
                time: manoeuvre.end(),
            },
        );
        self.events.insert(
            index,
            ManoeuvreEvent::Start {
                time: manoeuvre.start,
                manoeuvre,
            },
        );
    }

    #[inline]
    pub fn clear(&mut self) {
        self.events.truncate(1);
        self.idx = 0;
    }
}

impl<V, F> AccelerationModel for ManoeuvreSchedule<V, F>
where
    V: Default,
    F: ManoeuvreFrame<V>,
    F::Transform: Transform<V>,
{
    type Vector = V;

    #[inline]
    fn acceleration(&self, t: Epoch, state: &StateVector<Self::Vector>) -> Option<Self::Vector> {
        if let ManoeuvreEvent::Start { manoeuvre, .. } = self.current() {
            return manoeuvre.acceleration(t, state);
        }
        Some(V::default())
    }
}

#[derive(Clone)]
pub struct SpacecraftModel<C, V, F> {
    pub context: C,
    pub manoeuvres: ManoeuvreSchedule<V, F>,
}

impl<C, V, F> FirstOrderODE<f64, [StateVector<V>; 1]> for SpacecraftModel<C, V, F>
where
    C: AccelerationModel<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    F: ManoeuvreFrame<V>,
    F::Transform: Transform<V>,
{
    #[inline]
    fn eval(
        &mut self,
        t: f64,
        [y]: &[StateVector<V>; 1],
        [dy]: &mut [StateVector<V>; 1],
    ) -> Result<(), EvalFailed> {
        let t = Epoch::from_offset(Duration::from_seconds(t));
        dy.velocity = self.context.acceleration(t, y).ok_or(EvalFailed)?
            + self.manoeuvres.acceleration(t, y).ok_or(EvalFailed)?;
        dy.position = y.velocity;

        Ok(())
    }
}

pub type SpacecraftProblem<C, V, F> =
    ODEProblem<f64, [StateVector<V>; 1], SpacecraftModel<C, V, F>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpacecraftPropagationError {
    IntegrationError(StepError),
}
impl std::fmt::Display for SpacecraftPropagationError {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpacecraftPropagationError::IntegrationError(e) => write!(f, "integration failed: {e}"),
        }
    }
}
impl std::error::Error for SpacecraftPropagationError {}

impl From<StepError> for SpacecraftPropagationError {
    #[inline]
    fn from(err: StepError) -> Self {
        SpacecraftPropagationError::IntegrationError(err)
    }
}

pub struct SpacecraftPropagator<C, V, F, M: Method<SpacecraftProblem<C, V, F>>> {
    integration: Integration<SpacecraftProblem<C, V, F>, M>,
}

impl<C, V, F, M> Clone for SpacecraftPropagator<C, V, F, M>
where
    C: Clone,
    V: Clone,
    F: Clone,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: IntegratorState + Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            integration: self.integration.clone(),
        }
    }
}

impl<C, V, F, M> SpacecraftPropagator<C, V, F, M>
where
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>>,
{
    #[inline]
    pub fn new<Params>(
        initial_time: Epoch,
        initial_state: StateVector<V>,
        params: Params,
        context: C,
    ) -> Self
    where
        C: PropagationContext,
        M: NewMethod<Params>,
    {
        Self {
            integration: M::new(params).integrate(SpacecraftProblem {
                time: initial_time.as_offset_seconds(),
                bound: context.max_time().as_offset_seconds(),
                state: [initial_state],
                ode: SpacecraftModel {
                    context,
                    manoeuvres: Default::default(),
                },
            }),
        }
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        Epoch::from_offset(Duration::from_seconds(self.integration.problem.time))
    }

    #[inline]
    pub fn delta(&self) -> Duration
    where
        M::Integrator: IntegratorState<Time = f64>,
    {
        Duration::from_seconds(self.integration.step_size())
    }

    #[inline]
    pub fn position(&self) -> V
    where
        V: Clone,
    {
        self.integration.problem.state[0].position.clone()
    }

    #[inline]
    pub fn velocity(&self) -> V
    where
        V: Clone,
    {
        self.integration.problem.state[0].velocity.clone()
    }

    #[inline]
    pub fn context(&self) -> &C {
        &self.integration.problem.ode.context
    }

    #[inline]
    pub fn schedule(&self) -> &ManoeuvreSchedule<V, F> {
        &self.integration.problem.ode.manoeuvres
    }

    #[inline]
    pub fn clear_manoeuvres(&mut self) {
        self.integration.problem.ode.manoeuvres.clear();
    }

    #[inline]
    pub fn insert_manoeuvre(&mut self, manoeuvre: ConstantThrust<V, F>) {
        self.integration.problem.ode.manoeuvres.insert(manoeuvre);
    }

    #[inline]
    pub fn set_initial_state(&mut self, time: Epoch, state_vector: StateVector<V>)
    where
        Integration<SpacecraftProblem<C, V, F>, M>: ResetIntegration,
    {
        self.integration.problem.time = time.as_offset_seconds();
        self.integration.problem.state[0] = state_vector;
        self.integration.reset();
    }

    #[inline]
    pub fn set_bound(&mut self, bound: Epoch)
    where
        C: PropagationContext,
    {
        let max = self.integration.problem.ode.context.max_time();
        self.integration.problem.bound = bound.min(max).as_offset_seconds();
    }

    #[inline]
    pub fn set_max_iterations(&mut self, max_iterations: usize)
    where
        M::Integrator: AdaptiveRKState,
    {
        self.integration.integrator.set_n_max(max_iterations as _);
    }
}

impl<C, V, F, M> Propagator for SpacecraftPropagator<C, V, F, M>
where
    M: Method<SpacecraftProblem<C, V, F>>,
{
    type Trajectory = <Self::Trajectories as Iterable>::Item;

    type Trajectories = [CubicHermiteSplineSamples<V>; 1];
}

impl<C, V, F, M> IncrementalPropagator for SpacecraftPropagator<C, V, F, M>
where
    V: Clone,
    C: PropagationContext,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>> + IntegratorState,
    Integration<SpacecraftProblem<C, V, F>, M>: ResetIntegration,
{
    type Error = SpacecraftPropagationError;

    #[inline]
    fn step(
        &mut self,
        [trajectory]: &mut Self::Trajectories,
    ) -> Result<(), SpacecraftPropagationError> {
        let manoeuvre_changed = self.integration.problem.ode.manoeuvres.update(self.time());
        let next_manoeuvre_change = self.integration.problem.ode.manoeuvres.next_event_time();
        self.set_bound(next_manoeuvre_change.unwrap_or(Epoch::from_offset(Duration::MAX)));
        // A manoeuvre change essentially means the problem definition has changed; we reset the
        // integrator so it can be re-initialised with the new problem definition so that
        // potentially cached information is forgotten.
        if manoeuvre_changed {
            self.integration.reset();
        }
        self.integration.step()?;

        trajectory.push(self.time(), self.position(), self.velocity());

        Ok(())
    }
}

impl<C, V, F, M> DirectionalPropagator for SpacecraftPropagator<C, V, F, M>
where
    V: Clone,
    C: PropagationContext,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>>,
    Integration<SpacecraftProblem<C, V, F>, M>: ResetIntegration,
{
    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        lhs.cmp(rhs)
    }

    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        to + duration
    }

    #[inline]
    fn boundary(trajectory: &Self::Trajectory) -> Epoch {
        trajectory.end()
    }
}

impl<C, V, F, M> BranchingPropagator for SpacecraftPropagator<C, V, F, M>
where
    V: Clone,
    C: PropagationContext,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>>,
    Integration<SpacecraftProblem<C, V, F>, M>: ResetIntegration,
{
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        [CubicHermiteSplineSamples::new(
            self.time(),
            self.position(),
            self.velocity(),
        )]
    }

    #[inline]
    fn merge(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_after(rhs.start());
        lhs.extend(rhs);
    }
}

pub trait ResetIntegration {
    fn reset(&mut self);
}

impl<P, C, T, I> ResetIntegration for Integration<P, FixedRungeKutta<C, T>>
where
    FixedRungeKutta<C, T>: Method<P, Integrator = I>,
    I: IntegratorState<Time = T>,
{
    #[inline]
    fn reset(&mut self) {
        self.integrator = FixedRungeKutta::new(FixedMethodParams::new(self.integrator.step_size()))
            .init(&self.problem);
    }
}

impl<P, C, T, Tol, U, I> ResetIntegration for Integration<P, AdaptiveRungeKutta<C, T, Tol, U>>
where
    AdaptiveRungeKutta<C, T, Tol, U>: Method<P, Integrator = I>,
    T: UpperBounded + Default,
    Tol: Clone,
    U: Div<Output = U> + From<u16>,
    I: AdaptiveRKState<Tolerance = Tol> + IntegratorState<Time = T>,
{
    #[inline]
    fn reset(&mut self) {
        self.integrator = AdaptiveRungeKutta::new(AdaptiveMethodParams::new(
            self.integrator.tolerance().clone(),
            self.integrator.n_max() - self.integrator.n(),
        ))
        .init(&self.problem);
    }
}

impl<P, C, T, S, I> ResetIntegration for Integration<P, LinearMultistep<C, T, S>>
where
    LinearMultistep<C, T, S>: Method<P, Integrator = I>,
    T: Copy,
    S: NewMethod<FixedMethodParams<T>>,
    I: IntegratorState<Time = T>,
{
    #[inline]
    fn reset(&mut self) {
        self.integrator = LinearMultistep::new(FixedMethodParams::new(self.integrator.step_size()))
            .init(&self.problem);
    }
}
