use crate::{
    dynamics::{PredictionTrajectory, StateVector, Trajectory},
    prediction::PropagationTarget,
};

use bevy::ecs::query::QueryData;
use bevy::math::{DMat3, DVec3};
use bevy::prelude::*;
use ephemeris::{
    AccelerationModel, BoundedTrajectory, BranchingPropagator, DirectionalPropagator,
    EvaluateTrajectory, Frame, IncrementalPropagator, PropagationEnvironment, Propagator,
    Transform,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

pub type CubicHermiteSplineSamples = ephemeris::CubicHermiteSplineSamples<DVec3>;

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
    Relative(Trajectory),
    #[default]
    Inertial,
}

impl ReferenceFrame {
    #[inline]
    pub fn relative(trajectory: Trajectory) -> Self {
        Self::Relative(trajectory)
    }

    #[inline]
    pub fn inertial() -> Self {
        Self::Inertial
    }
}

impl<C> Frame<DVec3, C> for ReferenceFrame {
    type Transform = TNB;

    #[inline]
    fn transform(&self, t: Epoch, sv: &StateVector, _: &C) -> Option<Self::Transform> {
        match &self {
            ReferenceFrame::Relative(reference) => Some(TNB::new(*sv - reference.state_vector(t)?)),
            ReferenceFrame::Inertial => Some(TNB::IDENTITY),
        }
    }
}

pub type ConstantThrust = ephemeris::ConstantThrust<DVec3, ReferenceFrame>;

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
                .flat_map(|(entity, body)| {
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

impl PropagationEnvironment for Bodies {
    #[inline]
    fn max_time(&self) -> Epoch {
        self.0
            .values()
            .fold(Epoch::MAX, |end, body| end.min(body.trajectory.end()))
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
        (e.position / self.position)
            .abs()
            .max_element()
            .max((e.velocity / self.velocity).abs().max_element())
    }
}

pub type Segment = ephemeris::Segment<DVec3, ReferenceFrame>;
pub type Timeline = ephemeris::Timeline<DVec3, ReferenceFrame>;

pub type SpacecraftPropagator =
    ephemeris::SpacecraftPropagator<Bodies, DVec3, ReferenceFrame, Verner87<f64, AbsTol, f64>>;

/// Stores the times at which an entity entered spheres of influence.
#[derive(Clone, Debug, Default, Component)]
pub struct SoiTransitions(Vec<(Epoch, Entity)>);

impl SoiTransitions {
    #[inline]
    pub fn binary_search(&self, time: Epoch) -> Result<usize, usize> {
        self.0.binary_search_by(|(t, ..)| t.cmp(&time))
    }

    #[inline]
    fn soi_at_idx(&self, time: Epoch) -> Option<usize> {
        match self.binary_search(time) {
            Ok(i) => Some(i),
            Err(0) => None,
            Err(i) => Some(i - 1),
        }
    }

    #[inline]
    pub fn soi_at(&self, time: Epoch) -> Option<Entity> {
        if self.0.is_empty() {
            return None;
        }

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

#[derive(Clone)]
pub struct SpacecraftPropagatorSoiDetection(pub SpacecraftPropagator);

impl SpacecraftPropagatorSoiDetection {
    #[inline]
    pub fn new(
        initial_time: Epoch,
        initial_state: StateVector,
        params: AdaptiveMethodParams<f64, AbsTol, f64>,
        context: Bodies,
        timeline: Timeline,
    ) -> Self {
        Self(SpacecraftPropagator::new(
            initial_time,
            initial_state,
            params,
            context,
            timeline,
        ))
    }

    #[inline]
    pub fn max_iterations(&self) -> usize {
        self.0.max_iterations()
    }

    #[inline]
    pub fn tolerance(&self) -> &AbsTol {
        self.0.tolerance()
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        self.0.time()
    }

    #[inline]
    pub fn delta(&self) -> Duration {
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
    pub fn state_vector(&self) -> StateVector {
        StateVector {
            position: self.position(),
            velocity: self.velocity(),
        }
    }

    #[inline]
    pub fn environment(&self) -> &Bodies {
        self.0.environment()
    }

    #[inline]
    pub fn timeline(&self) -> &Timeline {
        self.0.timeline()
    }

    #[inline]
    pub fn join(lhs: &mut CubicHermiteSplineSamples, rhs: CubicHermiteSplineSamples) {
        SpacecraftPropagator::join(lhs, rhs)
    }
}

impl Propagator for SpacecraftPropagatorSoiDetection {
    type Trajectories = [(CubicHermiteSplineSamples, SoiTransitions); 1];
}

impl IncrementalPropagator for SpacecraftPropagatorSoiDetection {
    type Error = <SpacecraftPropagator as IncrementalPropagator>::Error;

    #[inline]
    fn step(&mut self, [(traj, transitions)]: &mut Self::Trajectories) -> Result<(), Self::Error> {
        let t0 = self.time();
        let p0 = self.position();
        self.0.step(std::array::from_mut(traj))?;

        let t1 = self.time();
        let p1 = self.position();
        for (entity, body) in self.environment().iter() {
            if let Some((time, pos, rate)) = body.soi_transition_with(&*traj, t0, t1, p0, p1) {
                if rate.is_sign_negative() {
                    transitions.insert(time, *entity);
                } else if let Some(entered) = self.environment().soi_at(time, pos, &[*entity]) {
                    transitions.insert(time, entered);
                }
            }
        }

        Ok(())
    }
}

impl DirectionalPropagator for SpacecraftPropagatorSoiDetection {
    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        SpacecraftPropagator::cmp(lhs, rhs)
    }

    #[inline]
    fn offset(time: Epoch, duration: Duration) -> Epoch {
        SpacecraftPropagator::offset(time, duration)
    }

    #[inline]
    fn boundaries([(trajectory, _)]: &Self::Trajectories) -> impl Iterator<Item = Epoch> + '_ {
        SpacecraftPropagator::boundaries(std::array::from_ref(trajectory))
    }
}

impl BranchingPropagator for SpacecraftPropagatorSoiDetection {
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        [(
            CubicHermiteSplineSamples::new(self.time(), self.position(), self.velocity()),
            SoiTransitions::default(),
        )]
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct SpacecraftTrajectory {
    pub trajectory: &'static mut Trajectory,
    pub transitions: &'static mut SoiTransitions,
}

impl PropagationTarget for SpacecraftTrajectory {
    type Propagator = SpacecraftPropagatorSoiDetection;

    #[inline]
    fn merge(
        item: &mut Self::Item<'_, '_>,
        (trajectory, transitions): (CubicHermiteSplineSamples, SoiTransitions),
    ) {
        item.transitions.clear_after(trajectory.start());
        item.transitions.extend(transitions);
        match &mut *item.trajectory.write() {
            PredictionTrajectory::CubicHermiteSplineSamples(item_traj) => {
                Self::Propagator::join(item_traj, trajectory)
            }
            _ => unreachable!(),
        };
    }

    #[inline]
    fn overwrite(
        item: &mut Self::Item<'_, '_>,
        (trajectory, transitions): (CubicHermiteSplineSamples, SoiTransitions),
    ) {
        *item.transitions = transitions;
        match &mut *item.trajectory.write() {
            PredictionTrajectory::CubicHermiteSplineSamples(item_traj) => *item_traj = trajectory,
            _ => unreachable!(),
        }
    }
}
