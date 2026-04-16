use crate::{
    dynamics::{PredictionTrajectory, StateVector, Trajectory},
    prediction::PredictionTarget,
};

use bevy::ecs::query::QueryData;
use bevy::math::{DMat3, DVec3};
use bevy::prelude::*;
use ephemeris::{
    AccelerationModel, BoundedTrajectory, BranchingPropagator, DirectionalPropagator,
    EvaluateTrajectory, Frame, IncrementalPropagator, PropagationContext, Propagator,
    SpacecraftPropagatorError, SpacecraftPropagatorState, Transform,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

pub type CubicHermiteSplineSamples = ephemeris::CubicHermiteSpline<DVec3>;

#[derive(Clone, Copy, Debug, Component, Deref, DerefMut)]
pub struct Mu(pub f64);

#[derive(Clone, Copy, Debug, Component)]
pub struct SphereOfInfluence {
    pub radius: f64,
}

impl SphereOfInfluence {
    pub const INFINITY: Self = Self {
        radius: f64::INFINITY,
    };

    #[inline]
    pub fn approximate(a: f64, m: f64, m_parent: f64) -> Self {
        Self {
            radius: a * (m / m_parent).powf(2.0 / 5.0),
        }
    }
}

#[derive(Clone)]
pub struct GravitationalBody {
    pub trajectory: Trajectory,
    pub mu: Mu,
    pub soi: SphereOfInfluence,
}

impl GravitationalBody {
    #[inline]
    pub fn new(trajectory: Trajectory, mu: Mu, soi: SphereOfInfluence) -> Self {
        Self {
            trajectory,
            mu,
            soi,
        }
    }

    #[inline]
    fn acceleration_at(&self, t: Epoch, position: &DVec3) -> Option<DVec3> {
        let body_position = self.trajectory.position(t)?;
        Some((body_position, *self.mu).acceleration_at::<false>(position, &0.0))
    }

    #[inline]
    fn soi_distance_squared_at(&self, t: Epoch, position: DVec3) -> Option<f64> {
        Some(
            position.distance_squared(self.trajectory.position(t)?)
                - self.soi.radius * self.soi.radius,
        )
    }
}

impl GravitationalBody {
    #[inline]
    pub fn soi_transition_with<T>(
        &self,
        trajectory: T,
        t0: Epoch,
        t1: Epoch,
        p0: DVec3,
        p1: DVec3,
    ) -> Option<(Epoch, DVec3, f64)>
    where
        T: EvaluateTrajectory<Vector = DVec3>,
    {
        let d0 = self.soi_distance_squared_at(t0, p0)?;
        let d1 = self.soi_distance_squared_at(t1, p1)?;

        if d0.signum() == d1.signum() {
            return None;
        }

        let f = |t| self.soi_distance_squared_at(t, trajectory.position(t)?);

        // We unwrap since bisection guarantees that the evaluations remain within the interval.
        find_root_bisection(|t| f(t).unwrap(), t0, t1, d0, d1, 100, 1e-3).and_then(|time| {
            let trajectory_sv = trajectory.state_vector(time)?;
            let relative_sv = trajectory_sv - self.trajectory.state_vector(time)?;
            let rate = 2.0 * relative_sv.position.dot(relative_sv.velocity);
            Some((time, trajectory_sv.position, rate))
        })
    }
}

#[inline]
fn find_root_bisection<F>(
    mut f: F,
    mut x0: Epoch,
    mut x1: Epoch,
    mut f0: f64,
    _f1: f64,
    max_iterations: usize,
    precision: f64,
) -> Option<Epoch>
where
    F: FnMut(Epoch) -> f64,
{
    for _ in 0..max_iterations {
        let mid = x0 + (x1 - x0) / 2.0;
        let f_mid = f(mid);

        if f0.signum() != f_mid.signum() {
            x1 = mid;
        } else {
            x0 = mid;
            f0 = f_mid;
        }

        if (x1 - x0).abs().as_seconds() < precision {
            return Some(x0);
        }
    }

    None
}

#[derive(Clone)]
pub struct Bodies(pub bevy::ecs::entity::EntityHashMap<GravitationalBody>);

impl Bodies {
    #[inline]
    pub fn iter(
        &self,
    ) -> bevy::platform::collections::hash_map::Iter<'_, Entity, GravitationalBody> {
        self.0.iter()
    }

    #[inline]
    pub fn soi_at(&self, t: Epoch, position: DVec3, except: &[Entity]) -> Option<Entity> {
        find_soi(
            self.0
                .iter()
                .filter(|(entity, _)| !except.contains(entity))
                .filter_map(|(entity, body)| {
                    Some((*entity, body.trajectory.position(t)?, body.soi.radius))
                }),
            position,
        )
    }

    #[inline]
    pub fn is_valid_at(&self, t: Epoch) -> bool {
        self.0.values().all(|body| body.trajectory.contains(t))
    }
}

#[inline]
pub fn find_soi<I>(iter: I, position: DVec3) -> Option<Entity>
where
    I: IntoIterator<Item = (Entity, DVec3, f64)>,
{
    iter.into_iter()
        .map(|(entity, soi_position, soi_radius)| {
            (entity, position.distance_squared(soi_position), soi_radius)
        })
        .filter(|(_, soi_distance_sq, soi_radius)| *soi_distance_sq < soi_radius * soi_radius)
        .min_by(|(_, a, _), (_, b, _)| a.total_cmp(b))
        .map(|(entity, _, _)| entity)
}

impl AccelerationModel for Bodies {
    type Vector = DVec3;

    #[inline]
    fn acceleration(&self, t: Epoch, state: &StateVector) -> Option<Self::Vector> {
        let mut acc = DVec3::ZERO;
        for body in self.0.values() {
            acc += body.acceleration_at(t, &state.position)?;
        }
        Some(acc)
    }
}

impl PropagationContext for Bodies {
    #[inline]
    fn max_time(&self) -> Epoch {
        self.0
            .values()
            .fold(Epoch::MAX, |end, body| end.min(body.trajectory.end()))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TNB(pub DMat3);

impl TNB {
    pub const IDENTITY: Self = Self(DMat3::IDENTITY);

    #[inline]
    pub fn new(sv: StateVector) -> Self {
        let x = sv.velocity.normalize_or_zero();
        let y = sv.position.cross(sv.velocity).normalize_or_zero();
        let z = x.cross(y).normalize_or_zero();
        Self(DMat3::from_cols(x, z, y))
    }
}

impl Transform<DVec3> for TNB {
    #[inline]
    fn to_inertial(&self, v: &DVec3) -> DVec3 {
        self.0.mul_vec3(*v)
    }
}

#[derive(Clone, Default, Debug, PartialEq)]
pub enum ReferenceFrame {
    /// Relative to a reference trajectory.
    Relative(Entity),
    #[default]
    Inertial,
}

impl ReferenceFrame {
    #[inline]
    pub fn relative(entity: Entity) -> Self {
        Self::Relative(entity)
    }

    #[inline]
    pub fn inertial() -> Self {
        Self::Inertial
    }
}

impl Frame<DVec3, Bodies> for ReferenceFrame {
    type Transform = TNB;

    #[inline]
    fn transform(&self, t: Epoch, sv: &StateVector, bodies: &Bodies) -> Option<Self::Transform> {
        match &self {
            ReferenceFrame::Relative(reference) => Some(TNB::new(
                *sv - bodies.0.get(reference)?.trajectory.state_vector(t)?,
            )),
            ReferenceFrame::Inertial => Some(TNB::IDENTITY),
        }
    }
}

pub type ConstantThrust = ephemeris::ConstantThrust<DVec3, ReferenceFrame>;
pub type Segment = ephemeris::Segment<DVec3, ReferenceFrame>;
pub type Timeline = ephemeris::Timeline<DVec3, ReferenceFrame>;

pub type SpacecraftProblem<S, V = DVec3> =
    ephemeris::SpacecraftProblem<S, ReferenceFrame, Bodies, V>;
pub type BaseSpacecraftPropagator<S, M> =
    ephemeris::SpacecraftPropagator<S, ReferenceFrame, Bodies, M>;

/// A list of sphere of influence transitions, sorted by time.
#[derive(Clone, Debug, Default, Component)]
pub struct SoiTransitions(Vec<(Epoch, Entity)>);

impl SoiTransitions {
    #[inline]
    pub fn binary_search(&self, time: Epoch) -> Result<usize, usize> {
        self.0.binary_search_by(|(t, ..)| t.cmp(&time))
    }

    #[inline]
    pub fn soi_at_idx(&self, time: Epoch) -> Option<usize> {
        match self.binary_search(time) {
            Ok(i) => Some(i),
            Err(0) => None,
            Err(i) => Some(i - 1),
        }
    }

    #[inline]
    pub fn soi_at(&self, time: Epoch) -> Option<Entity> {
        Some(self.0[self.soi_at_idx(time)?].1)
    }

    #[inline]
    pub fn insert(&mut self, time: Epoch, entity: Entity) {
        match self.binary_search(time) {
            Ok(i) => self.0[i] = (time, entity),
            Err(i) if i > 0 && self.0.get(i - 1).is_some_and(|(_, c)| *c == entity) => (),
            Err(i) => self.0.insert(i, (time, entity)),
        }
    }

    #[inline]
    pub fn clear_after(&mut self, time: Epoch) {
        match self.binary_search(time) {
            Ok(i) => self.0.truncate(i + 1),
            Err(i) => self.0.truncate(i),
        }
    }

    #[inline]
    pub fn clear_before(&mut self, time: Epoch) {
        match self.binary_search(time) {
            Ok(i) | Err(i) => self.0.drain(..i),
        };
    }

    #[inline]
    pub fn extend(&mut self, other: SoiTransitions) {
        self.0.extend(other.0);
    }

    #[inline]
    pub fn get(&self, index: usize) -> Option<&(Epoch, Entity)> {
        self.0.get(index)
    }

    #[inline]
    pub fn iter(&self) -> std::slice::Iter<'_, (Epoch, Entity)> {
        self.0.iter()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

pub struct SpacecraftPropagatorSoiDetection<S, M>(pub BaseSpacecraftPropagator<S, M>)
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>>;

impl<S, M> Clone for SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3> + Clone,
    S::Vector: Clone,
    M: Method<SpacecraftProblem<S>> + Clone,
    M::Integrator: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<S, M> SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>>,
{
    #[inline]
    pub fn new<Params>(
        initial_time: Epoch,
        initial_state: StateVector,
        params: Params,
        context: Bodies,
        timeline: Timeline,
    ) -> Self
    where
        Params: Clone,
        M: NewMethod<Params>,
        M::Integrator: Integrator<SpacecraftProblem<S>>,
    {
        Self(BaseSpacecraftPropagator::new(
            initial_time,
            initial_state,
            params,
            timeline,
            context,
        ))
    }

    #[inline]
    pub fn max_iterations(&self) -> usize
    where
        M::Integrator: AdaptiveRKState,
    {
        self.0.max_iterations()
    }

    #[inline]
    pub fn tolerance(&self) -> &<M::Integrator as AdaptiveRKState>::Tolerance
    where
        M::Integrator: AdaptiveRKState,
    {
        self.0.tolerance()
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        self.0.time()
    }

    #[inline]
    pub fn delta(&self) -> Duration
    where
        M::Integrator: IntegratorState<Time = f64>,
    {
        self.0.delta()
    }

    #[inline]
    pub fn position(&self) -> DVec3 {
        self.0.position()
    }

    #[inline]
    pub fn velocity(&self) -> DVec3 {
        self.0.velocity()
    }

    #[inline]
    pub fn context(&self) -> &Bodies {
        self.0.context()
    }

    #[inline]
    pub fn timeline(&self) -> &Timeline {
        self.0.timeline()
    }

    #[inline]
    pub fn join(lhs: &mut CubicHermiteSplineSamples, rhs: CubicHermiteSplineSamples) {
        BaseSpacecraftPropagator::<S, M>::join(lhs, rhs)
    }
}

impl<S, M> Propagator for SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>>,
{
    type Trajectories = [(CubicHermiteSplineSamples, SoiTransitions); 1];
}

impl<S, M> IncrementalPropagator for SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>> + Clone,
    M::Integrator: Integrator<SpacecraftProblem<S>> + IntegratorState,
{
    type Error = <BaseSpacecraftPropagator<S, M> as IncrementalPropagator>::Error;

    #[inline]
    fn step(&mut self, [(traj, transitions)]: &mut Self::Trajectories) -> Result<(), Self::Error> {
        let t0 = self.time();
        let p0 = self.position();

        self.0.step(std::array::from_mut(traj))?;

        let t1 = self.time();
        let p1 = self.position();
        for (entity, body) in self.context().iter() {
            // The trajectory passed here always has at least two points because branching this
            // propagator creates a trajectory with one point, and we just added a second one.
            if let Some((time, pos, rate)) = body.soi_transition_with(&*traj, t0, t1, p0, p1) {
                if rate.is_sign_negative() {
                    transitions.insert(time, *entity);
                } else if let Some(entered) = self.context().soi_at(time, pos, &[*entity]) {
                    transitions.insert(time, entered);
                }
            }
        }

        Ok(())
    }
}

impl<S, M> DirectionalPropagator for SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>>,
{
    #[inline]
    fn offset(time: Epoch, duration: Duration) -> Epoch {
        BaseSpacecraftPropagator::<S, M>::offset(time, duration)
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        BaseSpacecraftPropagator::<S, M>::distance(from, to)
    }

    #[inline]
    fn boundaries([(trajectory, _)]: &Self::Trajectories) -> impl Iterator<Item = Epoch> + '_ {
        BaseSpacecraftPropagator::<S, M>::boundaries(std::array::from_ref(trajectory))
    }
}

impl<S, M> BranchingPropagator for SpacecraftPropagatorSoiDetection<S, M>
where
    S: SpacecraftPropagatorState<Vector = DVec3>,
    M: Method<SpacecraftProblem<S>>,
{
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        [(
            CubicHermiteSplineSamples::new(self.time(), self.position(), self.velocity()),
            SoiTransitions::default(),
        )]
    }
}

#[derive(Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct AbsTol {
    pub position: f64,
    pub velocity: f64,
}

impl Tolerance<[StateVector; 1]> for AbsTol {
    type Output = f64;

    #[inline]
    fn err_over_tol(&mut self, _: &[StateVector; 1], [e]: &[StateVector; 1]) -> Self::Output {
        f64::max(
            (e.position / self.position).abs().max_element(),
            (e.velocity / self.velocity).abs().max_element(),
        )
    }
}

impl Tolerance<SecondOrderState<[DVec3; 1]>> for AbsTol {
    type Output = f64;

    #[inline]
    fn err_over_tol(
        &mut self,
        _: &SecondOrderState<[DVec3; 1]>,
        e: &SecondOrderState<[DVec3; 1]>,
    ) -> Self::Output {
        f64::max(
            (e.y[0] / self.position).abs().max_element(),
            (e.dy[0] / self.velocity).abs().max_element(),
        )
    }
}

#[derive(Clone)]
#[expect(clippy::large_enum_variant)]
pub enum SpacecraftPropagator {
    CashKarp45(SpacecraftPropagatorSoiDetection<[StateVector; 1], CashKarp45<f64, AbsTol, f64>>),
    DormandPrince54(
        SpacecraftPropagatorSoiDetection<[StateVector; 1], DormandPrince54<f64, AbsTol, f64>>,
    ),
    DormandPrince87(
        SpacecraftPropagatorSoiDetection<[StateVector; 1], DormandPrince87<f64, AbsTol, f64>>,
    ),
    Fehlberg45(SpacecraftPropagatorSoiDetection<[StateVector; 1], Fehlberg45<f64, AbsTol, f64>>),
    Tsitouras75(SpacecraftPropagatorSoiDetection<[StateVector; 1], Tsitouras75<f64, AbsTol, f64>>),
    Verner87(SpacecraftPropagatorSoiDetection<[StateVector; 1], Verner87<f64, AbsTol, f64>>),
    Verner98(SpacecraftPropagatorSoiDetection<[StateVector; 1], Verner98<f64, AbsTol, f64>>),
    Fine45(
        SpacecraftPropagatorSoiDetection<SecondOrderState<[DVec3; 1]>, Fine45<f64, AbsTol, f64>>,
    ),
}

macro_rules! delegate {
    ($self:expr, $method:ident $(, $args:expr)*) => {
        match $self {
            SpacecraftPropagator::CashKarp45(p) => p.$method($($args),*),
            SpacecraftPropagator::DormandPrince54(p) => p.$method($($args),*),
            SpacecraftPropagator::DormandPrince87(p) => p.$method($($args),*),
            SpacecraftPropagator::Fehlberg45(p) => p.$method($($args),*),
            SpacecraftPropagator::Tsitouras75(p) => p.$method($($args),*),
            SpacecraftPropagator::Verner87(p) => p.$method($($args),*),
            SpacecraftPropagator::Verner98(p) => p.$method($($args),*),
            SpacecraftPropagator::Fine45(p) => p.$method($($args),*),
        }
    };
}

impl SpacecraftPropagator {
    #[inline]
    pub fn new(
        initial_time: Epoch,
        initial_state: StateVector,
        params: AdaptiveMethodParams<f64, AbsTol, f64>,
        context: Bodies,
        timeline: Timeline,
    ) -> Self {
        Self::Verner87(SpacecraftPropagatorSoiDetection::new(
            initial_time,
            initial_state,
            params,
            context,
            timeline,
        ))
    }

    #[inline]
    pub fn max_iterations(&self) -> usize {
        delegate!(self, max_iterations)
    }

    #[inline]
    pub fn tolerance(&self) -> &AbsTol {
        delegate!(self, tolerance)
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        delegate!(self, time)
    }

    #[inline]
    pub fn delta(&self) -> Duration {
        delegate!(self, delta)
    }

    #[inline]
    pub fn position(&self) -> DVec3 {
        delegate!(self, position)
    }

    #[inline]
    pub fn velocity(&self) -> DVec3 {
        delegate!(self, velocity)
    }

    #[inline]
    pub fn context(&self) -> &Bodies {
        delegate!(self, context)
    }

    #[inline]
    pub fn timeline(&self) -> &Timeline {
        delegate!(self, timeline)
    }

    #[inline]
    pub fn join(lhs: &mut CubicHermiteSplineSamples, rhs: CubicHermiteSplineSamples) {
        lhs.clear_after(rhs.start());
        lhs.extend(rhs);
    }
}

impl Propagator for SpacecraftPropagator {
    type Trajectories = [(CubicHermiteSplineSamples, SoiTransitions); 1];
}

impl IncrementalPropagator for SpacecraftPropagator {
    type Error = SpacecraftPropagatorError;

    #[inline]
    fn step(&mut self, trajectories: &mut Self::Trajectories) -> Result<(), Self::Error> {
        delegate!(self, step, trajectories)
    }
}

impl DirectionalPropagator for SpacecraftPropagator {
    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        to + duration
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        to - from
    }

    #[inline]
    fn boundaries([(trajectory, _)]: &Self::Trajectories) -> impl Iterator<Item = Epoch> + '_ {
        [trajectory.end()].into_iter()
    }
}

impl BranchingPropagator for SpacecraftPropagator {
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        delegate!(self, branch)
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct SpacecraftTrajectory {
    pub trajectory: &'static mut Trajectory,
    pub transitions: &'static mut SoiTransitions,
}

impl PredictionTarget for SpacecraftTrajectory {
    type Propagator = SpacecraftPropagator;

    #[inline]
    fn merge(
        item: &mut Self::Item<'_, '_>,
        (trajectory, transitions): (CubicHermiteSplineSamples, SoiTransitions),
    ) {
        let PredictionTrajectory::CubicHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        item.transitions.clear_after(trajectory.start());
        item.transitions.extend(transitions);
        Self::Propagator::join(world_traj, trajectory)
    }

    #[inline]
    fn overwrite(
        item: &mut Self::Item<'_, '_>,
        (trajectory, transitions): (CubicHermiteSplineSamples, SoiTransitions),
    ) {
        let PredictionTrajectory::CubicHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        *item.transitions = transitions;
        *world_traj = trajectory;
    }
}
