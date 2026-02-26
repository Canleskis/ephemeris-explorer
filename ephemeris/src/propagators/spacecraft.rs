use crate::{
    BranchingPropagator, DirectionalPropagator, IncrementalPropagator, Iterable, Propagator,
    trajectory::{BoundedTrajectory, CubicHermiteSplineSamples, EvaluateTrajectory, StateVector},
};

use ftime::{Duration, Epoch};
use integration::prelude::*;

pub trait AccelerationModel {
    type Vector;

    fn acceleration(&self, t: Epoch, state: &StateVector<Self::Vector>) -> Option<Self::Vector>;
}

pub trait PropagationEnvironment {
    fn max_time(&self) -> Epoch;
}

pub trait Transform<V> {
    fn to_inertial(&self, v: &V) -> V;
}

pub trait Frame<V> {
    type Transform;

    fn transform(&self, t: Epoch, sv: &StateVector<V>) -> Option<Self::Transform>;
}

#[derive(Clone, Default, Debug, PartialEq)]
pub enum ReferenceKind<T> {
    /// Relative to a reference trajectory.
    Relative(T),
    #[default]
    Inertial,
}

#[derive(Clone, Debug, PartialEq)]
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

impl<T, F> Frame<T::Vector> for ReferenceFrame<T, F>
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
}

impl<V, F> AccelerationModel for ConstantThrust<V, F>
where
    F: Frame<V>,
    F::Transform: Transform<V>,
{
    type Vector = V;

    #[inline]
    fn acceleration(&self, t: Epoch, sv: &StateVector<Self::Vector>) -> Option<Self::Vector> {
        Some(self.frame.transform(t, sv)?.to_inertial(&self.acceleration))
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
}

impl<V, F> AccelerationModel for Segment<V, F>
where
    V: Default,
    F: Frame<V>,
    F::Transform: Transform<V>,
{
    type Vector = V;

    #[inline]
    fn acceleration(&self, t: Epoch, state: &StateVector<Self::Vector>) -> Option<Self::Vector> {
        match self {
            Segment::Burn { thrust, .. } => thrust.acceleration(t, state),
            _ => Some(V::default()),
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
pub struct SpacecraftModel<C, V, F> {
    pub environment: C,
    pub current_segment: usize,
    pub timeline: Timeline<V, F>,
}

impl<C, V, F> SpacecraftModel<C, V, F> {
    #[inline]
    pub fn new(time: Epoch, environment: C, timeline: Timeline<V, F>) -> Self {
        Self {
            environment,
            current_segment: timeline.segment_idx_at(time),
            timeline,
        }
    }

    #[inline]
    pub fn update_index(&mut self, time: Epoch) -> bool {
        let old_index = self.current_segment;
        self.current_segment = self.timeline.segment_idx_at(time);

        old_index != self.current_segment
    }

    #[inline]
    pub fn advance_timeline(&mut self, time: Epoch) -> Option<Epoch> {
        if time >= self.current_segment().end() {
            self.current_segment += 1;
            return Some(self.current_segment().end());
        }
        None
    }

    #[inline]
    pub fn current_segment(&self) -> &Segment<V, F> {
        &self.timeline.0[self.current_segment]
    }

    #[inline]
    pub fn environment_acceleration(&self, t: Epoch, sv: &StateVector<V>) -> Result<V, EvalFailed>
    where
        C: AccelerationModel<Vector = V>,
    {
        self.environment.acceleration(t, sv).ok_or(EvalFailed)
    }

    #[inline]
    pub fn manoeuvre_acceleration(&self, t: Epoch, sv: &StateVector<V>) -> Result<V, EvalFailed>
    where
        V: Default,
        F: Frame<V>,
        F::Transform: Transform<V>,
    {
        self.current_segment().acceleration(t, sv).ok_or(EvalFailed)
    }
}

impl<C, V, F> FirstOrderODE<f64, [StateVector<V>; 1]> for SpacecraftModel<C, V, F>
where
    C: AccelerationModel<Vector = V>,
    V: std::ops::Add<Output = V> + Default + Copy,
    F: Frame<V>,
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
        dy.velocity = self.environment_acceleration(t, y)? + self.manoeuvre_acceleration(t, y)?;
        dy.position = y.velocity;

        Ok(())
    }
}

pub type SpacecraftProblem<C, V, F> =
    ODEProblem<f64, [StateVector<V>; 1], SpacecraftModel<C, V, F>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpacecraftPropagatorError {
    IntegrationError(StepError),
}
impl std::fmt::Display for SpacecraftPropagatorError {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpacecraftPropagatorError::IntegrationError(e) => write!(f, "integration failed: {e}"),
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

pub struct SpacecraftPropagator<C, V, F, M: Method<SpacecraftProblem<C, V, F>>> {
    method: M,
    integration: Integration<SpacecraftProblem<C, V, F>, M>,
}

impl<C, V, F, M> Clone for SpacecraftPropagator<C, V, F, M>
where
    C: Clone,
    V: Clone,
    F: Clone,
    M: Method<SpacecraftProblem<C, V, F>> + Clone,
    M::Integrator: IntegratorState + Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            method: self.method.clone(),
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
        timeline: Timeline<V, F>,
    ) -> Self
    where
        C: PropagationEnvironment,
        M: NewMethod<Params>,
        Params: Clone,
    {
        Self {
            method: M::new(params.clone()),
            integration: M::new(params).integrate(SpacecraftProblem {
                time: initial_time.as_offset_seconds(),
                bound: timeline.segment_at(initial_time).end().as_offset_seconds(),
                state: [initial_state],
                ode: SpacecraftModel::new(initial_time, context, timeline),
            }),
        }
    }

    #[inline]
    pub fn reset_integrator(&mut self)
    where
        M: Clone,
    {
        self.integration.integrator = self.method.clone().init(&self.integration.problem);
    }

    #[inline]
    pub fn integration(&self) -> &Integration<SpacecraftProblem<C, V, F>, M> {
        &self.integration
    }

    #[inline]
    pub fn max_iterations(&self) -> usize
    where
        M::Integrator: AdaptiveRKState,
    {
        self.integration.integrator.n_max() as usize
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
        &self.integration.problem.ode.environment
    }

    #[inline]
    pub fn timeline(&self) -> &Timeline<V, F> {
        &self.integration.problem.ode.timeline
    }

    #[inline]
    pub fn join(lhs: &mut CubicHermiteSplineSamples<V>, rhs: CubicHermiteSplineSamples<V>) {
        lhs.clear_after(rhs.start());
        lhs.extend(rhs);
    }

    #[inline]
    fn set_bound(&mut self, bound: Epoch)
    where
        C: PropagationEnvironment,
    {
        self.integration.problem.bound = bound.as_offset_seconds();
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
    C: PropagationEnvironment,
    M: Method<SpacecraftProblem<C, V, F>> + Clone,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>> + IntegratorState,
{
    type Error = SpacecraftPropagatorError;

    #[inline]
    fn step(&mut self, [trajectory]: &mut Self::Trajectories) -> Result<(), Self::Error> {
        // A manoeuvre change essentially means the problem definition has changed; we reset the
        // integrator to re-initialise it with the new problem definition so that potentially cached
        // information is forgotten. This also means that depending on the specific details of the
        // integrator used, propagation may only be restarted at these manoeuvre changes (in this
        // context, restarting means creating a new integrator from the current problem and
        // expecting the same subsequent steps).
        if let Some(new_end) = self.integration.problem.ode.advance_timeline(self.time()) {
            self.set_bound(new_end);
            self.reset_integrator();
        }
        self.integration.step()?;

        trajectory.push(self.time(), self.position(), self.velocity());

        Ok(())
    }
}

impl<C, V, F, M> DirectionalPropagator for SpacecraftPropagator<C, V, F, M>
where
    V: Clone,
    C: PropagationEnvironment,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>>,
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
    C: PropagationEnvironment,
    M: Method<SpacecraftProblem<C, V, F>>,
    M::Integrator: Integrator<SpacecraftProblem<C, V, F>>,
{
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        [CubicHermiteSplineSamples::new(
            self.time(),
            self.position(),
            self.velocity(),
        )]
    }
}
