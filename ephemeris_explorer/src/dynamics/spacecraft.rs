use crate::{
    dynamics::{PredictionTrajectory, StateVector, Trajectory},
    prediction::{PredictionPropagator, PredictionTarget},
};

use bevy::ecs::query::QueryData;
use bevy::math::{DMat3, DVec3};
use bevy::prelude::*;
use ephemeris::{
    AccelerationModel, BoundedTrajectory, DirectionalPropagator, DirectionalSolout,
    EvaluateTrajectory, Forward, Frame, IncrementalPropagator, PropagationContext,
    PropagationDirection, Propagator, SpacecraftPropagatorError, SpacecraftState, Transform,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

pub type CubicHermiteSpline = ephemeris::CubicHermiteSpline<DVec3>;

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

#[derive(Clone, Copy, Debug)]
pub enum CrossingDirection {
    Ascending,
    Descending,
}

#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub time: Epoch,
    pub direction: CrossingDirection,
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

    #[inline]
    fn radial_velocity_at(&self, t: Epoch, sv: StateVector) -> Option<f64> {
        let relative_sv = sv - self.trajectory.state_vector(t)?;
        Some(relative_sv.position.dot(relative_sv.velocity))
    }

    #[inline]
    pub fn find_soi_crossing<T>(&self, traj: &T, t0: Epoch, t1: Epoch) -> Option<Event>
    where
        T: EvaluateTrajectory<Vector = DVec3>,
    {
        find_zero_crossing(t0, t1, |t| {
            self.soi_distance_squared_at(t, traj.position(t)?)
        })
    }

    #[inline]
    pub fn find_apsis<T>(&self, traj: &T, t0: Epoch, t1: Epoch) -> Option<Event>
    where
        T: EvaluateTrajectory<Vector = DVec3>,
    {
        find_zero_crossing(t0, t1, |t| {
            self.radial_velocity_at(t, traj.state_vector(t)?)
        })
    }
}

#[inline]
pub fn find_zero_crossing<F>(t0: Epoch, t1: Epoch, mut f: F) -> Option<Event>
where
    F: FnMut(Epoch) -> Option<f64>,
{
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

    let f0 = f(t0)?;
    let f1 = f(t1)?;

    if f0.signum() == f1.signum() {
        return None;
    }

    find_root_bisection(|t| f(t).unwrap(), t0, t1, f0, f1, 100, 1e-3).map(|time| Event {
        time,
        direction: match f0.is_sign_negative() {
            true => CrossingDirection::Ascending,
            false => CrossingDirection::Descending,
        },
    })
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
    pub fn soi_at_except(&self, t: Epoch, position: DVec3, except: &[Entity]) -> Option<Entity> {
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
    pub fn soi_at(&self, t: Epoch, position: DVec3) -> Option<Entity> {
        self.soi_at_except(t, position, &[])
    }

    #[inline]
    pub fn get(&self, entity: Entity) -> Option<&GravitationalBody> {
        self.0.get(&entity)
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
    pub fn try_new(sv: StateVector) -> Option<Self> {
        let x = sv.velocity.try_normalize()?;
        let y = sv.position.cross(sv.velocity).try_normalize()?;
        let z = x.cross(y).normalize();
        Some(Self(DMat3::from_cols(x, z, y)))
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
            ReferenceFrame::Relative(reference) => Some(TNB::try_new(
                *sv - bodies.get(*reference)?.trajectory.state_vector(t)?,
            )?),
            ReferenceFrame::Inertial => Some(TNB::IDENTITY),
        }
    }
}

pub type ConstantThrust = ephemeris::ConstantThrust<DVec3, ReferenceFrame>;
pub type Segment = ephemeris::Segment<DVec3, ReferenceFrame>;
pub type Timeline = ephemeris::Timeline<DVec3, ReferenceFrame>;

pub type SpacecraftProblem<T, V = DVec3> =
    ephemeris::SpacecraftProblem<T, ReferenceFrame, Bodies, V>;

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
    pub fn starting_at(&self, time: Epoch) -> &[(Epoch, Entity)] {
        &self.0[self.soi_at_idx(time).unwrap_or(0)..]
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
        self.0.reserve(other.0.len());
        for (time, entity) in other.0 {
            self.insert(time, entity);
        }
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

#[derive(Clone, Copy, Debug)]
pub enum Apsis {
    Periapsis { time: Epoch, distance: f64 },
    Apoapsis { time: Epoch, distance: f64 },
}

impl Apsis {
    #[inline]
    pub fn periapsis(time: Epoch, distance: f64) -> Self {
        Self::Periapsis { time, distance }
    }

    #[inline]
    pub fn apoapsis(time: Epoch, distance: f64) -> Self {
        Self::Apoapsis { time, distance }
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        match self {
            Apsis::Apoapsis { time, .. } | Apsis::Periapsis { time, .. } => *time,
        }
    }

    #[inline]
    pub fn distance(&self) -> f64 {
        match self {
            Apsis::Apoapsis { distance, .. } | Apsis::Periapsis { distance, .. } => *distance,
        }
    }
}

#[derive(Clone, Debug, Default, Component)]
pub struct Apsides(Vec<(Apsis, Entity)>);

impl Apsides {
    #[inline]
    pub fn binary_search(&self, time: Epoch) -> Result<usize, usize> {
        self.0
            .binary_search_by(|(apsis, _)| apsis.time().cmp(&time))
    }

    #[inline]
    pub fn insert(&mut self, apsis: Apsis, entity: Entity) {
        match self.binary_search(apsis.time()) {
            Ok(i) => self.0[i] = (apsis, entity),
            Err(i) => self.0.insert(i, (apsis, entity)),
        }
    }

    #[inline]
    pub fn extend(&mut self, apsides: Apsides) {
        self.0.extend(apsides.0);
    }

    #[inline]
    pub fn clear_after(&mut self, time: Epoch) {
        match self.binary_search(time) {
            Ok(i) => self.0.truncate(i + 1),
            Err(i) => self.0.truncate(i),
        }
    }

    #[inline]
    pub fn iter(&self) -> std::slice::Iter<'_, (Apsis, Entity)> {
        self.0.iter()
    }
}

#[derive(Clone)]
pub struct SpacecraftSolution {
    pub trajectory: CubicHermiteSpline,
    pub transitions: SoiTransitions,
    pub apsides: Apsides,
}

impl SpacecraftSolution {
    #[inline]
    pub fn last_trajectory_segment(&self) -> ephemeris::CubicHermite<DVec3> {
        self.trajectory
            .hermite3(self.trajectory.segment_count() - 1)
            .unwrap()
    }

    #[inline]
    pub fn add_state<T>(&mut self, problem: &SpacecraftProblem<T>) -> bool
    where
        T: SpacecraftState<Vector = DVec3>,
    {
        let t = Epoch::from_offset_seconds(problem.time);
        let position = problem.state.position();
        let velocity = problem.state.velocity();
        self.trajectory.push(t, *position, *velocity);

        let step_traj = self.last_trajectory_segment();
        let t0 = step_traj.start();
        let t1 = step_traj.end();

        let context = problem.ode.context();

        for (entity, body) in context.iter() {
            if let Some(event) = body.find_soi_crossing(&step_traj, t0, t1) {
                if matches!(event.direction, CrossingDirection::Descending) {
                    self.transitions.insert(event.time, *entity);
                } else if let Some(pos) = step_traj.position(event.time)
                    && let Some(entered) = context.soi_at_except(event.time, pos, &[*entity])
                {
                    self.transitions.insert(event.time, entered);
                }
            }
        }

        let crossed_in_step = self.transitions.starting_at(t0);
        for (i, &(t, soi)) in crossed_in_step.iter().enumerate() {
            let t0 = t.max(t0);
            let t1 = crossed_in_step.get(i + 1).map(|(t, _)| *t).unwrap_or(t1);
            if let Some(body) = context.get(soi)
                && let Some(event) = body.find_apsis(&step_traj, t0, t1)
                && let Some(distance) = body.trajectory.distance_at(&step_traj, event.time)
            {
                match event.direction {
                    CrossingDirection::Ascending => self
                        .apsides
                        .insert(Apsis::periapsis(event.time, distance), soi),
                    CrossingDirection::Descending => self
                        .apsides
                        .insert(Apsis::apoapsis(event.time, distance), soi),
                }
            }
        }

        true
    }
}

#[derive(Clone)]
pub struct SpacecraftSolout;

impl<T> Solout<SpacecraftProblem<T>> for SpacecraftSolout
where
    T: SpacecraftState<Vector = DVec3>,
{
    type Solution = SpacecraftSolution;

    #[inline]
    fn new_solution(&self, problem: &SpacecraftProblem<T>) -> Self::Solution {
        let time = Epoch::from_offset_seconds(problem.time);
        let position = problem.state.position();
        let velocity = problem.state.velocity();
        let current_soi = problem.ode.context().soi_at(time, *position);

        SpacecraftSolution {
            trajectory: CubicHermiteSpline::new(time, *position, *velocity),
            transitions: current_soi
                .map(|entity| SoiTransitions(vec![(time, entity)]))
                .unwrap_or_default(),
            apsides: Apsides::default(),
        }
    }

    #[inline]
    fn solout(&mut self, problem: &SpacecraftProblem<T>, solution: &mut Self::Solution) -> bool {
        let t1 = Epoch::from_offset_seconds(problem.time);
        let position = problem.state.position();
        let velocity = problem.state.velocity();
        solution.trajectory.push(t1, *position, *velocity);

        let context = problem.ode.context();

        let step_traj = &solution
            .trajectory
            .hermite3(solution.trajectory.segment_count() - 1)
            .unwrap();
        let t0 = step_traj.start();

        for (entity, body) in context.iter() {
            if let Some(event) = body.find_soi_crossing(step_traj, t0, t1) {
                if matches!(event.direction, CrossingDirection::Descending) {
                    solution.transitions.insert(event.time, *entity);
                } else if let Some(pos) = step_traj.position(event.time)
                    && let Some(entered) = context.soi_at_except(event.time, pos, &[*entity])
                {
                    solution.transitions.insert(event.time, entered);
                }
            }
        }

        let crossed_in_step = solution.transitions.starting_at(t0);
        for (i, &(t, soi)) in crossed_in_step.iter().enumerate() {
            let t0 = t.max(t0);
            let t1 = crossed_in_step.get(i + 1).map(|(t, _)| *t).unwrap_or(t1);
            if let Some(body) = context.get(soi)
                && let Some(event) = body.find_apsis(step_traj, t0, t1)
                && let Some(distance) = body.trajectory.distance_at(step_traj, event.time)
            {
                match event.direction {
                    CrossingDirection::Ascending => solution
                        .apsides
                        .insert(Apsis::periapsis(event.time, distance), soi),
                    CrossingDirection::Descending => solution
                        .apsides
                        .insert(Apsis::apoapsis(event.time, distance), soi),
                }
            }
        }

        true
    }
}

impl<T> DirectionalSolout<SpacecraftProblem<T>> for SpacecraftSolout
where
    T: SpacecraftState<Vector = DVec3>,
{
    type Direction = Forward;

    #[inline]
    fn solution_time(solution: &Self::Solution) -> Epoch {
        solution.trajectory.end()
    }

    #[inline]
    fn has_reached(solution: &Self::Solution, time: Epoch) -> bool {
        Forward::cmp(&solution.trajectory.end(), &time).is_ge()
    }
}

pub type BaseSpacecraftPropagator<T, M> =
    ephemeris::SpacecraftPropagator<T, ReferenceFrame, Bodies, M, SpacecraftSolout>;

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

macro_rules! spacecraft_propagator {
    (
        $( $variant:ident($state:ty) ),* $(,)?
    ) => {
        #[derive(Clone)]
        #[expect(clippy::large_enum_variant)]
        pub enum SpacecraftPropagator {
            $(
                $variant(BaseSpacecraftPropagator<$state, $variant<f64, AbsTol, f64>>),
            )*
        }

        $(
            impl From<BaseSpacecraftPropagator<$state, $variant<f64, AbsTol, f64>>> for SpacecraftPropagator {
                #[inline]
                fn from(propagator: BaseSpacecraftPropagator<$state, $variant<f64, AbsTol, f64>>) -> Self {
                    Self::$variant(propagator)
                }
            }
        )*

        impl SpacecraftPropagator {
            #[inline]
            pub fn new(
                initial_time: Epoch,
                initial_state: StateVector,
                params: AdaptiveMethodParams<f64, AbsTol, f64>,
                context: Bodies,
                timeline: Timeline,
            ) -> Self {
                Self::Verner87(BaseSpacecraftPropagator::new(
                    initial_time,
                    initial_state,
                    params,
                    timeline,
                    context,
                    SpacecraftSolout,
                ))
            }

            #[inline]
            pub fn max_iterations(&self) -> usize {
                match self {
                    $( Self::$variant(p) => p.max_iterations(), )*
                }
            }

            #[inline]
            pub fn tolerance(&self) -> &AbsTol {
                match self {
                    $( Self::$variant(p) => p.tolerance(), )*
                }
            }

            #[inline]
            pub fn time(&self) -> Epoch {
                match self {
                    $( Self::$variant(p) => p.time(), )*
                }
            }

            #[inline]
            pub fn delta(&self) -> Duration {
                match self {
                    $( Self::$variant(p) => p.delta(), )*
                }
            }

            #[inline]
            pub fn position(&self) -> DVec3 {
                match self {
                    $( Self::$variant(p) => p.position(), )*
                }
            }

            #[inline]
            pub fn velocity(&self) -> DVec3 {
                match self {
                    $( Self::$variant(p) => p.velocity(), )*
                }
            }

            #[inline]
            pub fn context(&self) -> &Bodies {
                match self {
                    $( Self::$variant(p) => p.context(), )*
                }
            }

            #[inline]
            pub fn timeline(&self) -> &Timeline {
                match self {
                    $( Self::$variant(p) => p.timeline(), )*
                }
            }

            #[inline]
            pub fn join(lhs: &mut CubicHermiteSpline, rhs: CubicHermiteSpline) {
                lhs.clear_after(rhs.start());
                lhs.extend(rhs);
            }
        }

        impl Propagator for SpacecraftPropagator {
            type Solution = [SpacecraftSolution; 1];

            #[inline]
            fn take_solution(&mut self) -> Self::Solution {
                match self {
                    $( Self::$variant(p) => [p.take_solution()], )*
                }
            }
        }

        impl IncrementalPropagator for SpacecraftPropagator {
            type Error = SpacecraftPropagatorError;

            #[inline]
            fn step(&mut self) -> Result<(), Self::Error> {
                match self {
                    $( Self::$variant(p) => p.step(), )*
                }
            }
        }

        impl DirectionalPropagator for SpacecraftPropagator {
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
                match self {
                    $( Self::$variant(p) => p.solution().trajectory.end(), )*
                }
            }
        }
    };
}

spacecraft_propagator! {
    CashKarp45([StateVector; 1]),
    DormandPrince54([StateVector; 1]),
    DormandPrince87([StateVector; 1]),
    Fehlberg45([StateVector; 1]),
    Tsitouras75([StateVector; 1]),
    Verner87([StateVector; 1]),
    Verner98([StateVector; 1]),
    Fine45(SecondOrderState<[DVec3; 1]>),
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct SpacecraftTrajectory {
    pub trajectory: &'static mut Trajectory,
    pub transitions: &'static mut SoiTransitions,
    pub apsides: &'static mut Apsides,
}

impl SpacecraftTrajectory {
    #[inline]
    pub fn new_propagator(
        initial_time: Epoch,
        initial_state: StateVector,
        params: AdaptiveMethodParams<f64, AbsTol, f64>,
        context: Bodies,
        timeline: Timeline,
    ) -> PredictionPropagator<Self> {
        PredictionPropagator(SpacecraftPropagator::new(
            initial_time,
            initial_state,
            params,
            context,
            timeline,
        ))
    }
}

impl PredictionTarget for SpacecraftTrajectory {
    type Propagator = SpacecraftPropagator;

    #[inline]
    fn merge(item: &mut Self::Item<'_, '_>, solution: SpacecraftSolution) {
        let PredictionTrajectory::CubicHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        item.apsides.clear_after(solution.trajectory.start());
        item.apsides.extend(solution.apsides);
        item.transitions.clear_after(solution.trajectory.start());
        item.transitions.extend(solution.transitions);
        Self::Propagator::join(world_traj, solution.trajectory)
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_, '_>, solution: SpacecraftSolution) {
        let PredictionTrajectory::CubicHermiteSpline(world_traj) = &mut *item.trajectory.write()
        else {
            unreachable!()
        };
        *item.apsides = solution.apsides;
        *item.transitions = solution.transitions;
        *world_traj = solution.trajectory;
    }
}
