use bevy::math::DVec3;
use bevy::prelude::*;
use deepsize::DeepSizeOf;
use hifitime::{Duration, Epoch};
use std::collections::VecDeque;

#[derive(Clone, Copy, Default, Debug, PartialEq, PartialOrd)]
pub struct StateVector<V> {
    pub position: V,
    pub velocity: V,
}

impl<V> StateVector<V> {
    pub fn new(position: V, velocity: V) -> Self {
        Self { position, velocity }
    }

    pub fn from_position(position: V) -> Self
    where
        V: Default,
    {
        Self {
            position,
            velocity: Default::default(),
        }
    }
}

impl std::ops::Index<usize> for StateVector<DVec3> {
    type Output = f64;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.position[0],
            1 => &self.position[1],
            2 => &self.position[2],
            3 => &self.velocity[0],
            4 => &self.velocity[1],
            5 => &self.velocity[2],
            _ => panic!("Index out of bounds: {}", index),
        }
    }
}

impl std::ops::IndexMut<usize> for StateVector<DVec3> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.position[0],
            1 => &mut self.position[1],
            2 => &mut self.position[2],
            3 => &mut self.velocity[0],
            4 => &mut self.velocity[1],
            5 => &mut self.velocity[2],
            _ => panic!("Index out of bounds: {}", index),
        }
    }
}

impl<S, V> std::ops::Mul<S> for StateVector<V>
where
    S: Clone,
    V: std::ops::Mul<S, Output = V>,
{
    type Output = StateVector<V>;

    #[inline]
    fn mul(self, rhs: S) -> Self::Output {
        StateVector {
            position: self.position * rhs.clone(),
            velocity: self.velocity * rhs,
        }
    }
}

impl<S, V> std::ops::Div<S> for StateVector<V>
where
    S: Clone,
    V: std::ops::Div<S, Output = V>,
{
    type Output = StateVector<V>;

    #[inline]
    fn div(self, rhs: S) -> Self::Output {
        StateVector {
            position: self.position / rhs.clone(),
            velocity: self.velocity / rhs,
        }
    }
}

impl<V> std::ops::Add for StateVector<V>
where
    V: std::ops::Add<Output = V>,
{
    type Output = StateVector<V>;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        StateVector {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
        }
    }
}

impl<V> std::ops::Sub for StateVector<V>
where
    V: std::ops::Sub<Output = V>,
{
    type Output = StateVector<V>;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        StateVector {
            position: self.position - rhs.position,
            velocity: self.velocity - rhs.velocity,
        }
    }
}

pub trait TrajectoryData {
    fn start(&self) -> Epoch;

    fn end(&self) -> Epoch;

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        time >= self.start() && time <= self.end()
    }

    /// Returns the number of interpolants in the trajectory.
    fn len(&self) -> usize;

    #[inline]
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn position(&self, at: Epoch) -> Option<DVec3>;

    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>>;
}

impl<T: TrajectoryData> TrajectoryData for &T {
    #[inline]
    fn start(&self) -> Epoch {
        (**self).start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        (**self).end()
    }

    #[inline]
    fn len(&self) -> usize {
        (**self).len()
    }

    #[inline]
    fn position(&self, at: Epoch) -> Option<DVec3> {
        (**self).position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        (**self).state_vector(at)
    }
}

pub trait TrajectoryInner: TrajectoryData + DeepSizeOf + Send + Sync {
    fn as_any(&self) -> &dyn std::any::Any;

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl<T: TrajectoryData + DeepSizeOf + Send + Sync + 'static> TrajectoryInner for T {
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
pub struct Trajectory(std::sync::Arc<std::sync::RwLock<dyn TrajectoryInner>>);

impl Trajectory {
    pub fn new<T: TrajectoryInner + 'static>(trajectory: T) -> Self {
        Self(std::sync::Arc::new(std::sync::RwLock::new(trajectory)))
    }

    pub fn read(&self) -> std::sync::RwLockReadGuard<dyn TrajectoryInner> {
        self.0.read().unwrap()
    }

    pub fn write(&mut self, f: impl FnOnce(&mut dyn TrajectoryInner)) {
        f(&mut *self.0.write().unwrap())
    }

    pub fn size(&self) -> usize {
        self.read().deep_size_of()
    }
}

impl TrajectoryData for Trajectory {
    #[inline]
    fn start(&self) -> Epoch {
        self.read().start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.read().end()
    }

    fn contains(&self, time: Epoch) -> bool {
        self.read().contains(time)
    }

    fn len(&self) -> usize {
        self.read().len()
    }

    fn is_empty(&self) -> bool {
        self.read().is_empty()
    }

    fn position(&self, at: Epoch) -> Option<DVec3> {
        self.read().position(at)
    }

    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        self.read().state_vector(at)
    }
}

#[derive(Clone, Copy)]
pub struct RelativeTrajectory<T1, T2 = T1> {
    pub trajectory: T1,
    pub reference: Option<T2>,
}

impl<T1, T2> RelativeTrajectory<T1, T2> {
    pub fn new(trajectory: T1, reference: Option<T2>) -> Self {
        Self {
            trajectory,
            reference,
        }
    }
}

impl<T1: TrajectoryData, T2: TrajectoryData> RelativeTrajectory<T1, T2> {
    // Ternary search is appropriate for unimodal functions, but the distance is not guaranteed to
    // be, so this isn't ideal but the best we can do cheaply without fancy orbit determination or
    // more complex algorithms.
    #[inline]
    pub fn closest_separation_between(
        &self,
        left: Epoch,
        right: Epoch,
        precision: f64,
        max_iterations: usize,
    ) -> Option<Epoch> {
        let trajectory = &self.trajectory;
        let reference = self.reference.as_ref()?;

        let mut left = self.start().max(left);
        let mut right = self.end().min(right);

        if right <= left {
            return None;
        }

        let distance_squared = |at| {
            trajectory
                .position(at)
                .unwrap()
                .distance_squared(reference.position(at).unwrap())
        };

        let mut i = 0;
        // Ternary search
        loop {
            i += 1;
            let total_secs = right - left;
            let mid1 = left + total_secs / 3;
            let mid2 = right - total_secs / 3;

            match distance_squared(mid1) - distance_squared(mid2) {
                d if d.abs() < precision || i > max_iterations => {
                    return Some(mid1 + (mid2 - mid1) / 2);
                }
                d if d.is_sign_positive() => left = mid1,
                _ => right = mid2,
            }
        }
    }

    #[expect(unused)]
    #[inline]
    pub fn closest_separation(&self, precision: f64, max_iterations: usize) -> Option<Epoch> {
        self.closest_separation_between(self.start(), self.end(), precision, max_iterations)
    }
}

impl<T1: DeepSizeOf, T2: DeepSizeOf> DeepSizeOf for RelativeTrajectory<T1, T2> {
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.trajectory.deep_size_of_children(context)
            + self.reference.deep_size_of_children(context)
    }
}

impl<T1: TrajectoryData, T2: TrajectoryData> TrajectoryData for RelativeTrajectory<T1, T2> {
    #[inline]
    fn start(&self) -> Epoch {
        self.trajectory.start().max(
            self.reference
                .as_ref()
                .map_or(Epoch::from_tai_duration(Duration::MIN), T2::start),
        )
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.trajectory.end().min(
            self.reference
                .as_ref()
                .map_or(Epoch::from_tai_duration(Duration::MAX), T2::end),
        )
    }

    #[inline]
    fn len(&self) -> usize {
        self.trajectory
            .len()
            .min(self.reference.as_ref().map_or(usize::MAX, T2::len))
    }

    #[inline]
    fn position(&self, at: Epoch) -> Option<DVec3> {
        let reference_pos = self
            .reference
            .as_ref()
            .map_or(Some(Default::default()), |t| t.position(at))?;
        Some(self.trajectory.position(at)? - reference_pos)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        let reference_sv = self
            .reference
            .as_ref()
            .map_or(Some(Default::default()), |t| t.state_vector(at))?;
        Some(self.trajectory.state_vector(at)? - reference_sv)
    }
}

pub const DIV: usize = 8;
pub type SegmentStorage = poly_it::storage::smallvec::SmallVec<[f64; DIV]>;

#[derive(Clone, Debug)]
pub struct Segment {
    pub x: poly_it::Polynomial<f64, SegmentStorage>,
    pub y: poly_it::Polynomial<f64, SegmentStorage>,
    pub z: poly_it::Polynomial<f64, SegmentStorage>,
}

impl DeepSizeOf for Segment {
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.x.coeffs().deep_size_of_children(context)
            + self.y.coeffs().deep_size_of_children(context)
            + self.z.coeffs().deep_size_of_children(context)
    }
}

impl Segment {
    #[inline]
    pub fn eval(&self, t: f64) -> DVec3 {
        DVec3::new(self.x.eval(t), self.y.eval(t), self.z.eval(t))
    }

    #[inline]
    pub fn eval_and_deriv(&self, t: f64) -> (DVec3, DVec3) {
        let (x, vx) = fast_eval_and_deriv(self.x.coeffs(), t);
        let (y, vy) = fast_eval_and_deriv(self.y.coeffs(), t);
        let (z, vz) = fast_eval_and_deriv(self.z.coeffs(), t);

        (DVec3::new(x, y, z), DVec3::new(vx, vy, vz))
    }
}

#[inline]
fn fast_eval_and_deriv(coeffs: &[f64], x: f64) -> (f64, f64) {
    let first = coeffs.first().copied().unwrap_or_default();
    let last = coeffs.last().copied().unwrap_or_default();

    let mut eval = last;
    let mut deriv = last;
    for ci in coeffs.iter().skip(1).rev().skip(1) {
        eval = eval * x + ci;
        deriv = deriv * x + eval;
    }
    eval = eval * x + first;

    (eval, deriv)
}

#[derive(Clone, Debug)]
pub struct FixedSegments {
    start: Epoch,
    granule: Duration,
    segments: VecDeque<Segment>,
}

impl DeepSizeOf for FixedSegments {
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.segments.deep_size_of_children(context)
    }
}

impl TrajectoryData for FixedSegments {
    #[inline]
    fn start(&self) -> Epoch {
        self.start
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.start + self.span()
    }

    #[inline]
    fn len(&self) -> usize {
        self.segments.len()
    }

    #[inline]
    fn position(&self, at: Epoch) -> Option<DVec3> {
        self.evaluate(at, Segment::eval)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        self.evaluate(at, Segment::eval_and_deriv)
            .map(|(position, velocity)| {
                // Segment are normalised to τ = t / granule so the derivative is dx/dτ.
                // As such, dx/dt = dx/dτ * dτ/dt = dx/dτ / granule.
                StateVector::new(position, velocity / self.granule.to_seconds())
            })
    }
}

impl FixedSegments {
    pub fn new(start: Epoch, granule: Duration) -> Self {
        Self {
            start,
            granule,
            segments: VecDeque::new(),
        }
    }

    #[expect(unused)]
    #[inline]
    pub fn between(&self, start: Epoch, end: Epoch) -> Option<Self> {
        if self.is_empty() {
            return None;
        }

        let start = self.segment_index_exclusive(start - self.start)?;
        let end = self.segment_index_exclusive(end - self.start)?;

        Some(Self {
            start: self.start + self.granule * start as i64,
            granule: self.granule,
            segments: (start..end + 1)
                .filter_map(|i| self.segments.get(i).cloned())
                .collect(),
        })
    }

    #[inline]
    pub fn push_front(&mut self, segment: Segment) {
        self.segments.push_front(segment);
        self.start -= self.granule;
    }

    #[inline]
    pub fn push_back(&mut self, segment: Segment) {
        self.segments.push_back(segment);
    }

    #[inline]
    pub fn prepend(&mut self, trajectory: Self) {
        assert_eq!(self.start, trajectory.end());
        assert_eq!(self.granule, trajectory.granule);

        self.start = trajectory.start;

        self.segments.reserve(trajectory.segments.len());
        for segment in trajectory.segments.into_iter().rev() {
            self.segments.push_front(segment);
        }
    }

    #[inline]
    pub fn append(&mut self, trajectory: Self) {
        assert_eq!(self.end(), trajectory.start);
        assert_eq!(self.granule, trajectory.granule);

        self.segments.extend(trajectory.segments);
    }

    #[inline]
    pub fn clear_before(&mut self, at: Epoch) {
        if let Some(idx) = self.index_exclusive(at + self.granule) {
            self.start += self.granule * idx as i64;
            self.segments.drain(0..idx);
        }
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        if let Some(idx) = self.index(at) {
            self.segments.truncate(idx);
        }
    }

    #[inline]
    pub fn evaluate<F, T>(&self, at: Epoch, eval: F) -> Option<T>
    where
        F: Fn(&Segment, f64) -> T,
    {
        let local = at - self.start;
        let segment_index = self.segment_index_exclusive(local)?;
        let local_segment = local - self.granule * segment_index as i64;
        let normalised = local_segment.to_seconds() / self.granule.to_seconds();

        Some(eval(self.segments.get(segment_index)?, normalised))
    }

    #[inline]
    pub fn index(&self, at: Epoch) -> Option<usize> {
        self.segment_index(at - self.start)
    }

    #[inline]
    pub fn index_exclusive(&self, at: Epoch) -> Option<usize> {
        self.segment_index_exclusive(at - self.start)
    }

    #[inline]
    fn segment_index(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time >= self.span() {
            return None;
        }

        Some((time.total_nanoseconds() / self.granule.total_nanoseconds()) as usize)
    }

    #[inline]
    fn segment_index_exclusive(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time > self.span() {
            return None;
        }

        // We subtract 1 nanosecond so that "previous" segments are returned for times equal to
        // multiples of the granule.
        Some(
            (time.total_nanoseconds().saturating_sub(1) / self.granule.total_nanoseconds())
                as usize,
        )
    }

    #[inline]
    pub fn granule(&self) -> Duration {
        self.granule
    }

    #[inline]
    pub fn span(&self) -> Duration {
        self.granule * self.segments.len() as i64
    }

    #[expect(unused)]
    #[inline]
    pub fn segments(&self) -> &VecDeque<Segment> {
        &self.segments
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Hermite3<V> {
    bounds: (f64, f64),
    a0: V,
    a1: V,
    a2: V,
    a3: V,
}

impl<V> Hermite3<V>
where
    V: std::ops::Add<Output = V>
        + std::ops::Sub<Output = V>
        + std::ops::Mul<f64, Output = V>
        + PartialEq
        + Default
        + Copy,
{
    #[inline]
    pub fn new(bounds: (f64, f64), values: (V, V), derivatives: (V, V)) -> Self {
        let a0 = values.0;
        let a1 = derivatives.0;
        let dt = bounds.1 - bounds.0;

        let (a2, a3) = if dt == 0.0 && values.0 == values.1 && derivatives.0 == derivatives.1 {
            (V::default(), V::default())
        } else {
            let dt_recip = 1.0 / dt;
            let dt_recip_2 = dt_recip * dt_recip;
            let dt_recip_3 = dt_recip * dt_recip_2;
            let dt_val = values.1 - values.0;
            let a2 = dt_val * dt_recip_2 * 3.0 - (derivatives.0 * 2.0 + derivatives.1) * dt_recip;
            let a3 = dt_val * dt_recip_3 * -2.0 + (derivatives.0 + derivatives.1) * dt_recip_2;
            (a2, a3)
        };

        Hermite3 {
            bounds,
            a0,
            a1,
            a2,
            a3,
        }
    }

    #[inline]
    pub fn eval(&self, t: f64) -> V {
        let dt = t - self.bounds.0;
        (((self.a3 * dt + self.a2) * dt) + self.a1) * dt + self.a0
    }

    #[inline]
    pub fn eval_derivative(&self, t: f64) -> V {
        let dt = t - self.bounds.0;
        ((self.a3 * dt * 3.0 + self.a2 * 2.0) * dt) + self.a1
    }

    #[inline]
    pub fn eval_derivative2(&self, t: f64) -> V {
        let dt = t - self.bounds.0;
        self.a3 * dt * 6.0 + self.a2 * 2.0
    }
}

#[derive(Clone, Debug)]
pub struct DiscreteStates(Vec<(Epoch, StateVector<DVec3>)>);

impl DeepSizeOf for DiscreteStates {
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.0.capacity() * size_of::<(Epoch, StateVector<DVec3>)>()
    }
}

impl TrajectoryData for DiscreteStates {
    fn start(&self) -> Epoch {
        self.0.first().unwrap().0
    }

    fn end(&self) -> Epoch {
        self.0.last().unwrap().0
    }

    fn len(&self) -> usize {
        self.0.len().saturating_sub(1)
    }

    fn position(&self, at: Epoch) -> Option<DVec3> {
        match self.binary_search(at) {
            Ok(i) => Some(self.0[i].1.position),
            Err(i) => {
                let &(t1, sv1) = self.0.get(i.checked_sub(1)?)?;
                let &(t2, sv2) = self.0.get(i)?;
                let hermite = Hermite3::new(
                    (t1.to_tai_seconds(), t2.to_tai_seconds()),
                    (sv1.position, sv2.position),
                    (sv1.velocity, sv2.velocity),
                );

                Some(hermite.eval(at.to_tai_seconds()))
            }
        }
    }

    fn state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        match self.binary_search(at) {
            Ok(i) => Some(self.0[i].1),
            Err(i) => {
                let hermite = self.hermite3(i.checked_sub(1)?)?;

                Some(StateVector {
                    position: hermite.eval(at.to_tai_seconds()),
                    velocity: hermite.eval_derivative(at.to_tai_seconds()),
                })
            }
        }
    }
}

impl DiscreteStates {
    pub fn new(start: Epoch, velocity: DVec3, position: DVec3) -> Self {
        Self(vec![(start, StateVector::new(position, velocity))])
    }

    #[inline]
    pub fn push(&mut self, at: Epoch, velocity: DVec3, position: DVec3) {
        self.0.push((at, StateVector::new(position, velocity)));
    }

    #[inline]
    pub fn binary_search(&self, at: Epoch) -> Result<usize, usize> {
        self.0.binary_search_by(|(t, _)| t.cmp(&at))
    }

    #[inline]
    fn hermite3(&self, i: usize) -> Option<Hermite3<DVec3>> {
        let (t1, sv1) = self.0.get(i)?;
        let (t2, sv2) = self.0.get(i + 1)?;
        Some(Hermite3::new(
            (t1.to_tai_seconds(), t2.to_tai_seconds()),
            (sv1.position, sv2.position),
            (sv1.velocity, sv2.velocity),
        ))
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        self.0.retain(|(k, _)| &at >= k);
    }

    #[inline]
    pub fn extend(&mut self, rhs: DiscreteStates) {
        self.0.extend(rhs.0);
    }

    #[inline]
    pub fn get(&self, at: Epoch) -> Option<&StateVector<DVec3>> {
        self.binary_search(at).ok().map(|i| &self.0[i].1)
    }

    #[inline]
    pub fn points(&self) -> &[(Epoch, StateVector<DVec3>)] {
        &self.0
    }

    #[expect(unused)]
    #[inline]
    fn kinematic_state(&self, at: Epoch) -> Option<(StateVector<DVec3>, DVec3)> {
        match self.binary_search(at) {
            Ok(i) => Some((
                self.0[i].1,
                self.hermite3(i.saturating_sub(1))?
                    .eval_derivative2(at.to_tai_seconds()),
            )),
            Err(i) => {
                let hermite = self.hermite3(i.checked_sub(1)?)?;

                Some((
                    StateVector {
                        position: hermite.eval(at.to_tai_seconds()),
                        velocity: hermite.eval_derivative(at.to_tai_seconds()),
                    },
                    hermite.eval_derivative2(at.to_tai_seconds()),
                ))
            }
        }
    }
}
