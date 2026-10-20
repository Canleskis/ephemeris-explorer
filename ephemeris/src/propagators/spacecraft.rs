use crate::{
    DirectionalPropagator, DirectionalSolout, Forward, IncrementalPropagator, PropagationDirection,
    Propagator,
    trajectory::{BoundedTrajectory, KinematicState, QuinticHermiteSpline, StateVector},
};

use ftime::{Duration, Epoch};
use integration::prelude::*;

pub trait AccelerationModel {
    type Vector;

    fn acceleration(&mut self, t: Epoch, state: &StateVector<Self::Vector>)
    -> Option<Self::Vector>;
}

pub trait Transform<V> {
    fn to_inertial(&self, v: &V) -> V;
}

pub trait Frame<V, C> {
    type Transform;

    fn transform(&self, t: Epoch, sv: &StateVector<V>, context: &mut C) -> Option<Self::Transform>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConstantThrust<V, F> {
    pub acceleration: V,
    pub frame: F,
}

impl<V, F> ConstantThrust<V, F> {
    #[inline]
    pub fn new(acceleration: V, frame: F) -> Self {
        Self {
            acceleration,
            frame,
        }
    }

    #[inline]
    fn acceleration<C>(&self, t: Epoch, sv: &StateVector<V>, context: &mut C) -> Option<V>
    where
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        Some(
            self.frame
                .transform(t, sv, context)?
                .to_inertial(&self.acceleration),
        )
    }
}

#[derive(Clone, Debug)]
pub enum Segment<V, F> {
    Burn {
        start: Epoch,
        end: Epoch,
        thrust: ConstantThrust<V, F>,
    },
    Coast {
        start: Epoch,
        end: Epoch,
    },
}

impl<V, F> Default for Segment<V, F> {
    #[inline]
    fn default() -> Self {
        Segment::Coast {
            start: Epoch::MIN,
            end: Epoch::MAX,
        }
    }
}

impl<V, F> Segment<V, F> {
    #[inline]
    pub fn start(&self) -> Epoch {
        match self {
            Segment::Burn { start, .. } | Segment::Coast { start, .. } => *start,
        }
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        match self {
            Segment::Burn { end, .. } | Segment::Coast { end, .. } => *end,
        }
    }

    #[inline]
    pub fn thrust(&self) -> Option<&ConstantThrust<V, F>> {
        match self {
            Segment::Burn { thrust, .. } => Some(thrust),
            Segment::Coast { .. } => None,
        }
    }

    #[inline]
    pub fn acceleration<C>(&self, t: Epoch, sv: &StateVector<V>, context: &mut C) -> Option<V>
    where
        V: Default,
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        match self {
            Segment::Burn { thrust, .. } => thrust.acceleration(t, sv, context),
            Segment::Coast { .. } => Some(V::default()),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Timeline<V, F>(Vec<Segment<V, F>>);

impl<V, F> Default for Timeline<V, F> {
    #[inline]
    fn default() -> Self {
        Self::new(vec![])
    }
}

impl<V, F> Timeline<V, F> {
    #[inline]
    pub fn new(mut burns: Vec<(Epoch, Epoch, ConstantThrust<V, F>)>) -> Self {
        burns.sort_by_key(|(start, ..)| *start);

        let mut timeline = Vec::with_capacity(burns.len() * 2 + 1);
        let mut cursor = Epoch::MIN;

        for (start, end, thrust) in burns {
            if start > cursor {
                timeline.push(Segment::Coast {
                    start: cursor,
                    end: start,
                });
            }
            cursor = end;
            timeline.push(Segment::Burn { start, end, thrust });
        }

        // If final burn is finite.
        if cursor < Epoch::MAX {
            timeline.push(Segment::Coast {
                start: cursor,
                end: Epoch::MAX,
            });
        }

        Self(timeline)
    }

    #[inline]
    pub fn segments(&self) -> &[Segment<V, F>] {
        &self.0
    }

    #[inline]
    pub fn segment_idx_at(&self, time: Epoch) -> usize {
        self.0.partition_point(|seg| seg.end() <= time)
    }

    #[inline]
    pub fn segment_at(&self, time: Epoch) -> &Segment<V, F> {
        &self.0[self.segment_idx_at(time)]
    }

    #[inline]
    pub fn segments_between(&self, start: Epoch, end: Epoch) -> &[Segment<V, F>] {
        &self.0[self.segment_idx_at(start)..self.0.partition_point(|seg| seg.start() < end)]
    }

    #[inline]
    pub fn common_times(&self, other: &Self) -> impl Iterator<Item = Epoch>
    where
        V: PartialEq,
        F: PartialEq,
    {
        self.0
            .iter()
            .zip(other.0.iter())
            .scan(false, |done, (s1, s2)| {
                if *done || s1.start() != s2.start() {
                    return None;
                }
                let result = Some(s1.start());

                if s1.thrust() != s2.thrust() {
                    *done = true;
                }

                result
            })
    }

    #[inline]
    pub fn divergence_time_before(&self, other: &Self, before: Epoch) -> Epoch
    where
        V: PartialEq,
        F: PartialEq,
    {
        self.common_times(other)
            .take_while(|&t| t < before)
            .last()
            .unwrap()
    }

    #[inline]
    pub fn divergence_time(&self, other: &Self) -> Epoch
    where
        V: PartialEq,
        F: PartialEq,
    {
        self.common_times(other).last().unwrap()
    }
}

#[derive(Clone)]
pub struct SpacecraftModel<V, F, C> {
    reference: Epoch,
    end: Epoch,
    current_segment: usize,
    timeline: Timeline<V, F>,
    context: C,
}

impl<V, F, C> SpacecraftModel<V, F, C> {
    #[inline]
    pub fn new(start: Epoch, end: Epoch, timeline: Timeline<V, F>, context: C) -> Self {
        Self {
            reference: start,
            end,
            current_segment: timeline.segment_idx_at(start),
            timeline,
            context,
        }
    }

    /// The problem of propagating `state` with this model.
    #[inline]
    pub fn into_problem<T>(self, state: T) -> SpacecraftProblem<T, F, C, V> {
        ODEProblem::new(0.0, self.offset(self.bound()), state, self)
    }

    #[inline]
    pub fn epoch(&self, t: f64) -> Epoch {
        self.reference + Duration::from_seconds(t)
    }

    #[inline]
    pub fn offset(&self, epoch: Epoch) -> f64 {
        (epoch - self.reference).as_seconds()
    }

    #[inline]
    pub fn reference(&self) -> Epoch {
        self.reference
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        self.end
    }

    #[inline]
    pub fn bound(&self) -> Epoch {
        self.current_segment().end().min(self.end)
    }

    #[inline]
    pub fn advance_timeline(&mut self) -> bool {
        if self.reference < self.current_segment().end() {
            return false;
        }
        self.current_segment = self.timeline.segment_idx_at(self.reference);
        true
    }

    #[inline]
    pub fn current_segment(&self) -> &Segment<V, F> {
        &self.timeline.0[self.current_segment]
    }

    #[inline]
    pub fn acceleration(&mut self, t: Epoch, sv: &StateVector<V>) -> Result<V, EvalFailed>
    where
        C: AccelerationModel<Vector = V>,
        V: std::ops::Add<Output = V> + Default,
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        Ok(self.context_acceleration(t, sv)? + self.manoeuvre_acceleration(t, sv)?)
    }

    #[inline]
    pub fn context_acceleration(&mut self, t: Epoch, sv: &StateVector<V>) -> Result<V, EvalFailed>
    where
        C: AccelerationModel<Vector = V>,
    {
        self.context.acceleration(t, sv).ok_or(EvalFailed)
    }

    #[inline]
    pub fn manoeuvre_acceleration(&mut self, t: Epoch, sv: &StateVector<V>) -> Result<V, EvalFailed>
    where
        V: Default,
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        self.timeline.0[self.current_segment]
            .acceleration(t, sv, &mut self.context)
            .ok_or(EvalFailed)
    }

    #[inline]
    pub fn context(&self) -> &C {
        &self.context
    }

    #[inline]
    pub fn context_mut(&mut self) -> &mut C {
        &mut self.context
    }
}

impl<V, F, C> FirstOrderODE<f64, [StateVector<V>; 1]> for SpacecraftModel<V, F, C>
where
    C: AccelerationModel<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    F: Frame<V, C>,
    F::Transform: Transform<V>,
{
    #[inline]
    fn eval(
        &mut self,
        t: f64,
        [y]: &[StateVector<V>; 1],
        [dy]: &mut [StateVector<V>; 1],
    ) -> Result<(), EvalFailed> {
        dy.velocity = self.acceleration(self.epoch(t), y)?;
        dy.position = y.velocity;

        Ok(())
    }
}

impl<V, F, C> SecondOrderODEGeneral<f64, [V; 1]> for SpacecraftModel<V, F, C>
where
    C: AccelerationModel<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    F: Frame<V, C>,
    F::Transform: Transform<V>,
{
    #[inline]
    fn eval(
        &mut self,
        t: f64,
        [y]: &[V; 1],
        [dy]: &[V; 1],
        [ddy]: &mut [V; 1],
    ) -> Result<(), EvalFailed> {
        *ddy = self.acceleration(self.epoch(t), &StateVector::new(*y, *dy))?;

        Ok(())
    }
}

pub trait SpacecraftState {
    type Vector;

    fn new(state: StateVector<Self::Vector>) -> Self;

    fn position(&self) -> &Self::Vector;

    fn velocity(&self) -> &Self::Vector;
}

impl<V> SpacecraftState for SecondOrderState<[V; 1]> {
    type Vector = V;

    #[inline]
    fn new(state: StateVector<Self::Vector>) -> Self {
        Self::new([state.position], [state.velocity])
    }

    #[inline]
    fn position(&self) -> &Self::Vector {
        &self.y[0]
    }

    #[inline]
    fn velocity(&self) -> &Self::Vector {
        &self.dy[0]
    }
}

impl<V> SpacecraftState for [StateVector<V>; 1] {
    type Vector = V;

    #[inline]
    fn new(state: StateVector<Self::Vector>) -> Self {
        [state]
    }

    #[inline]
    fn position(&self) -> &Self::Vector {
        &self[0].position
    }

    #[inline]
    fn velocity(&self) -> &Self::Vector {
        &self[0].velocity
    }
}

pub type SpacecraftProblem<T, F, C, V = <T as SpacecraftState>::Vector> =
    ODEProblem<f64, T, SpacecraftModel<V, F, C>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpacecraftPropagatorError {
    IntegrationError(StepError),
    Solout,
}
impl std::fmt::Display for SpacecraftPropagatorError {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpacecraftPropagatorError::IntegrationError(e) => write!(f, "integration failed: {e}"),
            SpacecraftPropagatorError::Solout => write!(f, "solout exit"),
        }
    }
}
impl std::error::Error for SpacecraftPropagatorError {}

impl From<StepError> for SpacecraftPropagatorError {
    #[inline]
    fn from(err: StepError) -> Self {
        SpacecraftPropagatorError::IntegrationError(err)
    }
}

impl From<EvalFailed> for SpacecraftPropagatorError {
    #[inline]
    fn from(_: EvalFailed) -> Self {
        SpacecraftPropagatorError::from(StepError::EvalFailed)
    }
}

pub struct SpacecraftPropagator<
    T: SpacecraftState,
    F,
    C,
    M: Method<SpacecraftProblem<T, F, C>>,
    O: Solout<SpacecraftProblem<T, F, C>>,
> {
    method: M,
    integrator: M::Integrator,
    problem: SpacecraftProblem<T, F, C>,
    solout: O,
    solution: O::Solution,
}

impl<T, F, C, M, O> Clone for SpacecraftPropagator<T, F, C, M, O>
where
    T: SpacecraftState + Clone,
    T::Vector: Clone,
    F: Clone,
    C: Clone,
    M: Method<SpacecraftProblem<T, F, C>> + Clone,
    M::Integrator: Clone,
    O: Solout<SpacecraftProblem<T, F, C>> + Clone,
    O::Solution: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            method: self.method.clone(),
            integrator: self.integrator.clone(),
            problem: self.problem.clone(),
            solout: self.solout.clone(),
            solution: self.solution.clone(),
        }
    }
}

impl<T, F, C, M> SpacecraftPropagator<T, F, C, M, ()>
where
    T: SpacecraftState,
    M: Method<SpacecraftProblem<T, F, C>>,
{
    #[inline]
    pub fn new<Params>(params: Params, problem: SpacecraftProblem<T, F, C>) -> Self
    where
        Params: Clone,
        M: NewMethod<Params>,
        M::Integrator: Integrator<SpacecraftProblem<T, F, C>>,
    {
        let mut propagator = Self {
            integrator: M::new(params.clone()).init(&problem),
            method: M::new(params),
            problem,
            solout: (),
            solution: (),
        };
        propagator.update_bound();
        propagator
    }

    #[inline]
    pub fn with_solout<O>(self, solout: O) -> SpacecraftPropagator<T, F, C, M, O>
    where
        O: Solout<SpacecraftProblem<T, F, C>>,
    {
        SpacecraftPropagator {
            solution: solout.new_solution(&self.problem),
            method: self.method,
            integrator: self.integrator,
            problem: self.problem,
            solout,
        }
    }
}

impl<T, F, C, M, O> SpacecraftPropagator<T, F, C, M, O>
where
    T: SpacecraftState,
    M: Method<SpacecraftProblem<T, F, C>>,
    O: Solout<SpacecraftProblem<T, F, C>>,
{
    #[inline]
    pub fn reset_integrator(&mut self)
    where
        M: Clone,
    {
        self.integrator = self.method.clone().init(&self.problem);
    }

    #[inline]
    fn rebase(&mut self) -> Result<(), StepError> {
        let reference = self.time();
        let previous = std::mem::replace(&mut self.problem.ode.reference, reference);
        self.problem.time = 0.0;
        self.update_bound();
        match reference != previous {
            true => Ok(()),
            false => Err(StepError::StepSizeUnderflow),
        }
    }

    #[inline]
    fn update_bound(&mut self) {
        self.problem.bound = self.problem.ode.offset(self.problem.ode.bound());
    }

    #[inline]
    fn update_segment(&mut self)
    where
        M: Clone,
    {
        // A manoeuvre change essentially means the problem definition has changed; we reset the
        // integrator to re-initialise it with the new problem definition so that potentially cached
        // information is forgotten. This also means that depending on the specific details of the
        // integrator used, propagation may only be restarted at these manoeuvre changes (in this
        // context, restarting means creating a new integrator from the current problem and expecting
        // the same subsequent steps).
        if self.problem.ode.advance_timeline() {
            self.update_bound();
            self.reset_integrator();
        }
    }

    #[inline]
    pub fn max_iterations(&self) -> usize
    where
        M::Integrator: AdaptiveRKState,
    {
        self.integrator.n_max() as usize
    }

    #[inline]
    pub fn tolerance(&self) -> &<M::Integrator as AdaptiveRKState>::Tolerance
    where
        M::Integrator: AdaptiveRKState,
    {
        self.integrator.tolerance()
    }

    #[inline]
    pub fn problem(&self) -> &SpacecraftProblem<T, F, C> {
        &self.problem
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        // `reference + time` could miss by a rounding.
        if self.problem.time == self.problem.bound {
            self.problem.ode.bound()
        } else {
            self.problem.ode.epoch(self.problem.time)
        }
    }

    #[inline]
    pub fn bound(&self) -> Epoch {
        self.problem.ode.end()
    }

    #[inline]
    pub fn delta(&self) -> Duration
    where
        M::Integrator: IntegratorState<Time = f64>,
    {
        Duration::from_seconds(self.integrator.step_size())
    }

    #[inline]
    pub fn position(&self) -> T::Vector
    where
        T::Vector: Clone,
    {
        self.problem.state.position().clone()
    }

    #[inline]
    pub fn velocity(&self) -> T::Vector
    where
        T::Vector: Clone,
    {
        self.problem.state.velocity().clone()
    }

    #[inline]
    pub fn context(&self) -> &C {
        &self.problem.ode.context
    }

    #[inline]
    pub fn timeline(&self) -> &Timeline<T::Vector, F> {
        &self.problem.ode.timeline
    }

    #[inline]
    pub fn solout(&self) -> &O {
        &self.solout
    }

    #[inline]
    pub fn solution(&self) -> &O::Solution {
        &self.solution
    }
}

impl<T, F, C, M, O> Propagator for SpacecraftPropagator<T, F, C, M, O>
where
    T: SpacecraftState,
    M: Method<SpacecraftProblem<T, F, C>>,
    O: Solout<SpacecraftProblem<T, F, C>>,
{
    type Solution = O::Solution;

    #[inline]
    fn take_solution(&mut self) -> Self::Solution {
        std::mem::replace(&mut self.solution, self.solout.new_solution(&self.problem))
    }
}

impl<T, F, C, M, O> IncrementalPropagator for SpacecraftPropagator<T, F, C, M, O>
where
    T: SpacecraftState,
    T::Vector: Clone,
    M: Method<SpacecraftProblem<T, F, C>> + Clone,
    M::Integrator: Integrator<SpacecraftProblem<T, F, C>>,
    O: Solout<SpacecraftProblem<T, F, C>>,
{
    type Error = SpacecraftPropagatorError;

    #[inline]
    fn step(&mut self) -> Result<(), Self::Error> {
        self.update_segment();
        self.integrator.advance(&mut self.problem)?;
        self.rebase()?;

        if !self.solout.solout(&mut self.problem, &mut self.solution) {
            return Err(SpacecraftPropagatorError::Solout);
        }

        Ok(())
    }
}

impl<T, F, C, M, O> DirectionalPropagator for SpacecraftPropagator<T, F, C, M, O>
where
    T: SpacecraftState,
    M: Method<SpacecraftProblem<T, F, C>>,
    O: DirectionalSolout<SpacecraftProblem<T, F, C>>,
{
    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        Forward::offset(to, duration)
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        Forward::distance(from, to)
    }

    #[inline]
    fn time(&self) -> Epoch {
        O::solution_time(&self.solution)
    }

    #[inline]
    fn has_reached(&self, time: Epoch) -> bool {
        O::has_reached(&self.solution, time)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct QuinticHermiteSplineSolout<V> {
    segment: usize,
    time: Epoch,
    state: KinematicState<V>,
}

impl<V> QuinticHermiteSplineSolout<V> {
    #[inline]
    pub fn new<T, F, C>(problem: &mut SpacecraftProblem<T, F, C>) -> Result<Self, EvalFailed>
    where
        T: SpacecraftState<Vector = V>,
        V: std::ops::Add<Output = V> + Default + Copy,
        C: AccelerationModel<Vector = V>,
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        let time = problem.ode.epoch(problem.time);
        let sv = StateVector::new(*problem.state.position(), *problem.state.velocity());
        let acceleration = problem.ode.acceleration(time, &sv)?;

        Ok(Self {
            time,
            state: KinematicState::new(sv.position, sv.velocity, acceleration),
            segment: problem.ode.current_segment,
        })
    }

    #[inline]
    fn push_step<T, F, C>(
        &mut self,
        problem: &mut SpacecraftProblem<T, F, C>,
        solution: &mut QuinticHermiteSpline<V>,
    ) -> Result<(), EvalFailed>
    where
        T: SpacecraftState<Vector = V>,
        V: std::ops::Add<Output = V> + Default + Copy,
        C: AccelerationModel<Vector = V>,
        F: Frame<V, C>,
        F::Transform: Transform<V>,
    {
        let change_acceleration = (problem.ode.current_segment != self.segment)
            .then(|| problem.ode.acceleration(self.time, &self.state.into()))
            .transpose()?;

        *self = Self::new(problem)?;
        let (at, k) = (self.time, self.state);

        match change_acceleration {
            Some(change) => {
                solution.push_back_after_change(change, at, k.position, k.velocity, k.acceleration)
            }
            None => solution.push_back(at, k.position, k.velocity, k.acceleration),
        }

        Ok(())
    }
}

impl<V, T, F, C> Solout<SpacecraftProblem<T, F, C>> for QuinticHermiteSplineSolout<V>
where
    T: SpacecraftState<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    C: AccelerationModel<Vector = V>,
    F: Frame<V, C>,
    F::Transform: Transform<V>,
{
    type Solution = QuinticHermiteSpline<V>;

    #[inline]
    fn new_solution(&self, _: &SpacecraftProblem<T, F, C>) -> Self::Solution {
        QuinticHermiteSpline::new(
            self.time,
            self.state.position,
            self.state.velocity,
            self.state.acceleration,
        )
    }

    #[inline]
    fn solout(
        &mut self,
        problem: &mut SpacecraftProblem<T, F, C>,
        solution: &mut Self::Solution,
    ) -> bool {
        self.push_step(problem, solution).is_ok()
    }
}

impl<V, T, F, C> DirectionalSolout<SpacecraftProblem<T, F, C>> for QuinticHermiteSplineSolout<V>
where
    T: SpacecraftState<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    C: AccelerationModel<Vector = V>,
    F: Frame<V, C>,
    F::Transform: Transform<V>,
{
    type Direction = Forward;

    #[inline]
    fn solution_time(solution: &Self::Solution) -> Epoch {
        solution.end()
    }

    #[inline]
    fn has_reached(solution: &Self::Solution, time: Epoch) -> bool {
        solution.end() >= time
    }
}
