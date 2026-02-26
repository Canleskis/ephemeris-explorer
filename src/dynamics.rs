use crate::prediction::PropagationTarget;

use bevy::ecs::query::QueryData;
use bevy::math::{DMat3, DVec3};
use bevy::prelude::*;
use deepsize::DeepSizeOf;
use ephemeris::{
    AccelerationModel, BoundedTrajectory, BranchingPropagator, DirectionalPropagator,
    EvaluateTrajectory, IncrementalPropagator, InterpolationAlgorithm, Polynomial,
    PropagationEnvironment, Propagator, Transform, eval_slice_horner,
};
use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;

pub use ephemeris::{Backward, Forward};

pub type StateVector = ephemeris::StateVector<DVec3>;

pub type UniformSpline = ephemeris::UniformSpline<DVec3>;
pub type CubicHermiteSplineSamples = ephemeris::CubicHermiteSplineSamples<DVec3>;

pub trait PredictionTrajectory:
    BoundedTrajectory + EvaluateTrajectory<Vector = DVec3> + DeepSizeOf + Send + Sync
{
    fn as_any(&self) -> &dyn std::any::Any;

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl<T> PredictionTrajectory for T
where
    T: BoundedTrajectory + EvaluateTrajectory<Vector = DVec3> + DeepSizeOf + Send + Sync + 'static,
{
    #[inline]
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    #[inline]
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[derive(Component, Clone)]
pub struct Trajectory(std::sync::Arc<std::sync::RwLock<dyn PredictionTrajectory>>);

impl std::fmt::Debug for Trajectory {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Trajectory")
            .field("start", &self.start())
            .field("end", &self.end())
            .finish()
    }
}

impl DeepSizeOf for Trajectory {
    #[inline]
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.read().deep_size_of_children(context)
    }
}

impl Trajectory {
    #[inline]
    pub fn new<T: PredictionTrajectory + 'static>(trajectory: T) -> Self {
        Self(std::sync::Arc::new(std::sync::RwLock::new(trajectory)))
    }

    #[inline]
    pub fn read(&self) -> std::sync::RwLockReadGuard<'_, dyn PredictionTrajectory> {
        self.0.read().unwrap()
    }

    #[inline]
    pub fn write(&mut self, f: impl FnOnce(&mut dyn PredictionTrajectory)) {
        f(&mut *self.0.write().unwrap())
    }

    #[inline]
    pub fn heap_size(&self) -> usize {
        self.deep_size_of()
    }
}

impl BoundedTrajectory for Trajectory {
    #[inline]
    fn start(&self) -> Epoch {
        self.read().start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.read().end()
    }

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        self.read().contains(time)
    }

    #[inline]
    fn len(&self) -> usize {
        self.read().len()
    }
}

impl EvaluateTrajectory for Trajectory {
    type Vector = DVec3;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        self.read().position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector> {
        self.read().state_vector(at)
    }
}

impl PartialEq for Trajectory {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        std::sync::Arc::ptr_eq(&self.0, &other.0)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LeastSquaresFit {
    pub degree: usize,
}

impl InterpolationAlgorithm<DVec3> for LeastSquaresFit {
    type Error = ();

    // Credit to the poly_it crate (https://github.com/SkyeC0re/polyit-rs)
    #[inline]
    fn interpolate(&self, ts: &[f64], xs: &[DVec3]) -> Result<Polynomial<DVec3>, Self::Error> {
        debug_assert!(ts.len() == xs.len());
        let data_len = ts.len();
        let (mut d_0, gamma_0, mut b_0) = ts.iter().zip(xs.iter()).fold(
            (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO),
            |mut acc, (xi, yi)| {
                acc.0 += yi;
                acc.1 += DVec3::ONE;
                acc.2 += xi;
                acc
            },
        );

        if gamma_0 == DVec3::ZERO {
            return Err(());
        }
        // Impossible for `data_len` to be zero at this stage.
        let degree = self.degree.min(data_len - 1);

        b_0 /= gamma_0;
        d_0 /= gamma_0;

        let mut p_data = smallvec::SmallVec::new_const();

        if degree == 0 {
            p_data.push(d_0);
            return Ok(Polynomial::new(p_data));
        }

        for _ in 0..(degree + 1) {
            p_data.push(DVec3::ZERO);
        }
        let mut p_km1 = p_data.clone();
        let mut p_km1 = p_km1.as_mut_slice();
        let mut p_k = p_data.clone();
        let mut p_k = p_k.as_mut_slice();
        let p_data_slice = p_data.as_mut_slice();
        p_data_slice[0] = d_0;
        // Safety: `p_data` has `deg + 1` elements.
        *unsafe { p_k.get_unchecked_mut(0) } = DVec3::ONE;

        let mut gamma_k = gamma_0;
        let mut b_k = b_0;
        let mut minus_c_k = DVec3::ZERO;
        let mut kp1 = 1;

        loop {
            // Overwrite $p_{k-1}$ with $p_{k+1}$
            for i in 0..kp1 {
                // Safety: `kp1 <= deg + 1`.
                let p_km1i = unsafe { p_km1.get_unchecked_mut(i) };
                *p_km1i = minus_c_k * *p_km1i - b_k * unsafe { p_k.get_unchecked(i) };
            }

            for im1 in 0..kp1 {
                // Safety: `kp1 <= deg`.
                let p_km1i = unsafe { p_km1.get_unchecked_mut(im1 + 1) };
                *p_km1i += unsafe { p_k.get_unchecked(im1) };
            }

            let (mut d_kp1, gamma_kp1, mut b_kp1) = ts.iter().zip(xs.iter()).fold(
                (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO),
                |mut acc, (xi, yi)| {
                    // Safety: `kp1 <= deg`.
                    let px = eval_slice_horner(unsafe { p_km1.get_unchecked(0..(kp1 + 1)) }, xi);
                    let wipx = px;
                    acc.0 += yi * wipx;

                    let wipxpx = wipx * px;
                    acc.1 += wipxpx;
                    acc.2 += xi * wipxpx;

                    acc
                },
            );

            if gamma_kp1 == DVec3::ZERO {
                break;
            }

            d_kp1 /= gamma_kp1;

            for i in 0..(kp1 + 1) {
                // Safety: `kp1 <= deg`.
                let pi = unsafe { p_data_slice.get_unchecked_mut(i) };
                *pi += d_kp1 * unsafe { p_km1.get_unchecked(i) };
            }

            if kp1 == degree {
                break;
            }

            // Delay the remaining work left for $b_{k+1}$ until it is certain to be needed.
            b_kp1 /= gamma_kp1;

            kp1 += 1;
            b_k = b_kp1;
            minus_c_k = -(gamma_kp1 / gamma_k);
            gamma_k = gamma_kp1;

            // Reorient the offsets
            std::mem::swap(&mut p_k, &mut p_km1);
        }
        let mut p = Polynomial::new(p_data);
        p.trim();
        Ok(p)
    }
}

pub type NBodyPropagator<D> =
    ephemeris::NBodyPropagator<D, DVec3, QuinlanTremaine12<f64>, LeastSquaresFit>;

#[derive(QueryData)]
#[query_data(mutable)]
pub struct CelestialTrajectory<D> {
    pub trajectory: &'static mut Trajectory,
    marker: std::marker::PhantomData<D>,
}

impl PropagationTarget for CelestialTrajectory<Forward> {
    type Propagator = NBodyPropagator<Forward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_>, propagated: UniformSpline) {
        item.trajectory.write(|trajectory| {
            let trajectory = trajectory
                .as_any_mut()
                .downcast_mut::<UniformSpline>()
                .unwrap();
            trajectory.clear_after(propagated.start());
            trajectory.append(propagated);
        });
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_>, propagated: UniformSpline) {
        // Should we keep the previous allocation?
        item.trajectory.write(|trajectory| {
            *trajectory.as_any_mut().downcast_mut().unwrap() = propagated;
        });
    }
}

impl PropagationTarget for CelestialTrajectory<Backward> {
    type Propagator = NBodyPropagator<Backward>;

    #[inline]
    fn merge(item: &mut Self::Item<'_>, propagated: UniformSpline) {
        item.trajectory.write(|trajectory| {
            let trajectory = trajectory
                .as_any_mut()
                .downcast_mut::<UniformSpline>()
                .unwrap();
            trajectory.clear_before(propagated.end());
            trajectory.prepend(propagated);
        });
    }

    #[inline]
    fn overwrite(item: &mut Self::Item<'_>, propagated: UniformSpline) {
        // Should we keep the previous allocation?
        item.trajectory.write(|trajectory| {
            *trajectory.as_any_mut().downcast_mut().unwrap() = propagated;
        });
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TNB(pub DMat3);

impl Default for TNB {
    #[inline]
    fn default() -> Self {
        Self(DMat3::IDENTITY)
    }
}

impl From<StateVector> for TNB {
    #[inline]
    fn from(sv: StateVector) -> Self {
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

pub type ReferenceFrame = ephemeris::ReferenceFrame<Trajectory, TNB>;

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
    pub fn approximate(a: f64, m: f64, parent: f64) -> Self {
        Self {
            radius: a * (m / parent).powf(2.0 / 5.0),
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
    fn soi_transition_with<T>(
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

impl Bodies {
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

#[derive(Clone, Copy, Default)]
pub struct AbsTol(pub StateVector);

impl Tolerance<[StateVector; 1]> for AbsTol {
    type Output = f64;

    #[inline]
    fn err_over_tol(&mut self, _: &[StateVector; 1], [e]: &[StateVector; 1]) -> Self::Output {
        let atol = self.0;
        (e.position / atol.position)
            .abs()
            .max_element()
            .max((e.velocity / atol.velocity).abs().max_element())
    }
}

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
    pub fn context(&self) -> &Bodies {
        self.0.context()
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
    type Trajectory = (CubicHermiteSplineSamples, SoiTransitions);

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
        for (entity, body) in self.0.context().0.iter() {
            if let Some((time, position, rate)) = body.soi_transition_with(&*traj, t0, t1, p0, p1) {
                if rate.is_sign_negative() {
                    transitions.insert(time, *entity);
                } else if let Some(entered) = self.context().soi_at(time, position, &[*entity]) {
                    transitions.insert(time, entered)
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
    fn boundary((trajectory, _): &Self::Trajectory) -> Epoch {
        SpacecraftPropagator::boundary(trajectory)
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
        item: &mut Self::Item<'_>,
        (trajectory, transitions): <Self::Propagator as Propagator>::Trajectory,
    ) {
        item.transitions.clear_after(trajectory.start());
        item.transitions.extend(transitions);
        item.trajectory.write(|t| {
            SpacecraftPropagatorSoiDetection::join(
                t.as_any_mut().downcast_mut().unwrap(),
                trajectory,
            );
        });
    }

    #[inline]
    fn overwrite(
        item: &mut Self::Item<'_>,
        (trajectory, transitions): <Self::Propagator as Propagator>::Trajectory,
    ) {
        *item.transitions = transitions;
        item.trajectory.write(|t| {
            *t.as_any_mut().downcast_mut().unwrap() = trajectory;
        });
    }
}

const fn ratio_f64(ratio: Ratio<u16>) -> f64 {
    ratio.numerator() as f64 / ratio.denominator() as f64
}

pub const DEFAULT_ADAPTIVE_PARAMS: AdaptiveMethodParams<f64, AbsTol, f64> =
    AdaptiveMethodParams::with(
        60.0,
        f64::MAX,
        AbsTol(StateVector {
            position: DVec3::splat(1e-7),
            velocity: DVec3::splat(1e-5),
        }),
        // Const workaround instead of using `AdaptiveMethodParams::new`.
        ratio_f64(integration::DEFAULT_FAC_MIN),
        ratio_f64(integration::DEFAULT_FAC_MAX),
        ratio_f64(integration::DEFAULT_FAC),
        1_000_000,
    );

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transitions() {
        let mut transitions = SoiTransitions::default();
        transitions.insert(Epoch::from_offset_seconds(1.0), Entity::from_raw(1));
        transitions.insert(Epoch::from_offset_seconds(2.0), Entity::from_raw(2));
        transitions.insert(Epoch::from_offset_seconds(3.0), Entity::from_raw(3));

        dbg!(&transitions);

        transitions.clear_before(Epoch::from_offset_seconds(2.1));
        dbg!(&transitions);
    }
}
