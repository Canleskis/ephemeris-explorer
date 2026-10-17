use std::collections::VecDeque;
use std::ops::{Add, Mul, Sub};

use deepsize::DeepSizeOf;
use ftime::Epoch;

#[derive(Clone, Copy, Default, Debug, PartialEq, PartialOrd)]
pub struct StateVector<V> {
    pub position: V,
    pub velocity: V,
}

impl<V> StateVector<V> {
    #[inline]
    pub const fn new(position: V, velocity: V) -> Self {
        Self { position, velocity }
    }

    #[inline]
    pub fn splat(value: V) -> Self
    where
        V: Clone,
    {
        Self {
            position: value.clone(),
            velocity: value,
        }
    }

    #[inline]
    pub fn from_position(position: V) -> Self
    where
        V: Default,
    {
        Self {
            position,
            velocity: Default::default(),
        }
    }

    #[inline]
    pub fn component_mul(&self, other: &Self) -> Self
    where
        V: std::ops::Mul<Output = V> + Clone,
    {
        StateVector {
            position: self.position.clone() * other.position.clone(),
            velocity: self.velocity.clone() * other.velocity.clone(),
        }
    }

    #[inline]
    pub fn component_div(&self, other: &Self) -> Self
    where
        V: std::ops::Div<Output = V> + Clone,
    {
        StateVector {
            position: self.position.clone() / other.position.clone(),
            velocity: self.velocity.clone() / other.velocity.clone(),
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

#[derive(Clone, Copy, Default, Debug, PartialEq, PartialOrd)]
pub struct KinematicState<V> {
    pub position: V,
    pub velocity: V,
    pub acceleration: V,
}

impl<V> From<KinematicState<V>> for StateVector<V> {
    #[inline]
    fn from(value: KinematicState<V>) -> Self {
        StateVector {
            position: value.position,
            velocity: value.velocity,
        }
    }
}

impl<V> KinematicState<V> {
    #[inline]
    pub const fn new(position: V, velocity: V, acceleration: V) -> Self {
        Self {
            position,
            velocity,
            acceleration,
        }
    }

    #[inline]
    pub fn splat(value: V) -> Self
    where
        V: Clone,
    {
        Self {
            position: value.clone(),
            velocity: value.clone(),
            acceleration: value,
        }
    }

    #[inline]
    pub fn from_position(position: V) -> Self
    where
        V: Default,
    {
        Self {
            position,
            velocity: Default::default(),
            acceleration: Default::default(),
        }
    }

    #[inline]
    pub fn component_mul(&self, other: &Self) -> Self
    where
        V: std::ops::Mul<Output = V> + Clone,
    {
        Self {
            position: self.position.clone() * other.position.clone(),
            velocity: self.velocity.clone() * other.velocity.clone(),
            acceleration: self.acceleration.clone() * other.acceleration.clone(),
        }
    }

    #[inline]
    pub fn component_div(&self, other: &Self) -> Self
    where
        V: std::ops::Div<Output = V> + Clone,
    {
        Self {
            position: self.position.clone() / other.position.clone(),
            velocity: self.velocity.clone() / other.velocity.clone(),
            acceleration: self.acceleration.clone() / other.acceleration.clone(),
        }
    }
}

impl<S, V> std::ops::Mul<S> for KinematicState<V>
where
    S: Clone,
    V: std::ops::Mul<S, Output = V>,
{
    type Output = KinematicState<V>;

    #[inline]
    fn mul(self, rhs: S) -> Self::Output {
        Self::Output {
            position: self.position * rhs.clone(),
            velocity: self.velocity * rhs.clone(),
            acceleration: self.acceleration * rhs,
        }
    }
}

impl<S, V> std::ops::Div<S> for KinematicState<V>
where
    S: Clone,
    V: std::ops::Div<S, Output = V>,
{
    type Output = KinematicState<V>;

    #[inline]
    fn div(self, rhs: S) -> Self::Output {
        Self::Output {
            position: self.position / rhs.clone(),
            velocity: self.velocity / rhs.clone(),
            acceleration: self.acceleration / rhs,
        }
    }
}

impl<V> std::ops::Add for KinematicState<V>
where
    V: std::ops::Add<Output = V>,
{
    type Output = KinematicState<V>;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::Output {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
            acceleration: self.acceleration + rhs.acceleration,
        }
    }
}

impl<V> std::ops::Sub for KinematicState<V>
where
    V: std::ops::Sub<Output = V>,
{
    type Output = KinematicState<V>;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output {
            position: self.position - rhs.position,
            velocity: self.velocity - rhs.velocity,
            acceleration: self.acceleration - rhs.acceleration,
        }
    }
}

pub trait BoundedTrajectory {
    fn start(&self) -> Epoch;

    fn end(&self) -> Epoch;

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        time >= self.start() && time <= self.end()
    }

    fn segment_count(&self) -> usize;

    #[inline]
    fn is_empty(&self) -> bool {
        self.segment_count() == 0
    }
}

pub trait EvaluateTrajectory {
    type Vector;

    fn position(&self, at: Epoch) -> Option<Self::Vector>;

    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>>;
}

impl<T> BoundedTrajectory for &T
where
    T: BoundedTrajectory,
{
    #[inline]
    fn start(&self) -> Epoch {
        (**self).start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        (**self).end()
    }

    #[inline]
    fn segment_count(&self) -> usize {
        (**self).segment_count()
    }
}

impl<T> EvaluateTrajectory for &T
where
    T: EvaluateTrajectory,
{
    type Vector = T::Vector;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        (**self).position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        (**self).state_vector(at)
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

impl<T1, T2> RelativeTrajectory<T1, T2>
where
    Self: BoundedTrajectory,
{
    // Ternary search is appropriate for unimodal functions, but the distance is not guaranteed to
    // be, so this isn't ideal.
    #[inline]
    pub fn closest_separation_between<F>(
        &self,
        left: Epoch,
        right: Epoch,
        precision: f64,
        max_iterations: usize,
        distance: F,
    ) -> Option<Epoch>
    where
        F: Fn(&T1, &T2, Epoch) -> f64,
    {
        let trajectory = &self.trajectory;
        let reference = self.reference.as_ref()?;

        let mut left = self.start().max(left);
        let mut right = self.end().min(right);

        if right <= left {
            return None;
        }

        let distance = |at| distance(trajectory, reference, at);

        let mut i = 0;
        // Ternary search
        loop {
            i += 1;
            let total_secs = right - left;
            let mid1 = left + total_secs / 3.0;
            let mid2 = right - total_secs / 3.0;

            match distance(mid1) - distance(mid2) {
                d if d.abs() < precision || i > max_iterations => {
                    return Some(mid1 + (mid2 - mid1) / 2.0);
                }
                d if d.is_sign_positive() => left = mid1,
                _ => right = mid2,
            }
        }
    }

    #[inline]
    pub fn closest_separation<F>(
        &self,
        precision: f64,
        max_iterations: usize,
        distance: F,
    ) -> Option<Epoch>
    where
        F: Fn(&T1, &T2, Epoch) -> f64,
    {
        self.closest_separation_between(
            self.start(),
            self.end(),
            precision,
            max_iterations,
            distance,
        )
    }
}

impl<T1: DeepSizeOf, T2: DeepSizeOf> DeepSizeOf for RelativeTrajectory<T1, T2> {
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.trajectory.deep_size_of_children(context)
            + self.reference.deep_size_of_children(context)
    }
}

impl<T1, T2> BoundedTrajectory for RelativeTrajectory<T1, T2>
where
    T1: BoundedTrajectory,
    T2: BoundedTrajectory,
{
    #[inline]
    fn start(&self) -> Epoch {
        match self.reference.as_ref() {
            Some(reference) => self.trajectory.start().max(reference.start()),
            None => self.trajectory.start(),
        }
    }

    #[inline]
    fn end(&self) -> Epoch {
        match self.reference.as_ref() {
            Some(reference) => self.trajectory.end().min(reference.end()),
            None => self.trajectory.end(),
        }
    }

    #[inline]
    fn segment_count(&self) -> usize {
        match self.reference.as_ref() {
            Some(reference) => self
                .trajectory
                .segment_count()
                .min(reference.segment_count()),
            None => self.trajectory.segment_count(),
        }
    }
}

impl<T1, T2, V> EvaluateTrajectory for RelativeTrajectory<T1, T2>
where
    T1: EvaluateTrajectory<Vector = V>,
    T2: EvaluateTrajectory<Vector = V>,
    V: std::ops::Sub<Output = T1::Vector> + Default,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        let reference_pos = self
            .reference
            .as_ref()
            .map_or(Some(Default::default()), |t| t.position(at))?;
        Some(self.trajectory.position(at)? - reference_pos)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        let reference_sv = self
            .reference
            .as_ref()
            .map_or(Some(Default::default()), |t| t.state_vector(at))?;
        Some(self.trajectory.state_vector(at)? - reference_sv)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct QuinticHermiteSegment<V> {
    bounds: (f64, f64),
    a0: V,
    a1: V,
    a2: V,
    a3: V,
    a4: V,
    a5: V,
}

impl<V> QuinticHermiteSegment<V> {
    #[inline]
    pub fn new(t1: Epoch, s1: KinematicState<V>, t2: Epoch, s2: KinematicState<V>) -> Self
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        debug_assert!(t1 <= t2, "segment start {t1} is after its end {t2}");

        let bounds = (t1.as_offset_seconds(), t2.as_offset_seconds());
        let a0 = s1.position;
        let a1 = s1.velocity;
        let a2 = s1.acceleration * 0.5;
        let dt = bounds.1 - bounds.0;

        let (a3, a4, a5) = if dt == 0.0
            && s1.position == s2.position
            && s1.velocity == s2.velocity
            && s1.acceleration == s2.acceleration
        {
            (V::default(), V::default(), V::default())
        } else {
            let dt_recip = 1.0 / dt;
            let dt_recip_2 = dt_recip * dt_recip;
            let dt_recip_3 = dt_recip * dt_recip_2;
            let dt_recip_4 = dt_recip * dt_recip_3;
            let dt_recip_5 = dt_recip * dt_recip_4;

            let dp = s2.position - s1.position;
            let dv0 = s1.velocity;
            let dv1 = s2.velocity;
            let da0 = s1.acceleration;
            let da1 = s2.acceleration;

            let c0 = dp * 10.0 - (dv0 * 6.0 + dv1 * 4.0) * dt + (da1 * 0.5 - da0 * 1.5) * (dt * dt);
            let c1 = dp * -15.0 + (dv0 * 8.0 + dv1 * 7.0) * dt + (da0 * 1.5 - da1) * (dt * dt);
            let c2 = dp * 6.0 - (dv0 + dv1) * 3.0 * dt + (da1 * 0.5 - da0 * 0.5) * (dt * dt);

            (c0 * dt_recip_3, c1 * dt_recip_4, c2 * dt_recip_5)
        };

        Self {
            bounds,
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
        }
    }

    #[inline]
    pub fn new_unordered(t1: Epoch, s1: KinematicState<V>, t2: Epoch, s2: KinematicState<V>) -> Self
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        if t1 <= t2 {
            Self::new(t1, s1, t2, s2)
        } else {
            Self::new(t2, s2, t1, s1)
        }
    }

    #[inline]
    fn eval(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        (((((self.a5 * dt + self.a4) * dt) + self.a3) * dt + self.a2) * dt + self.a1) * dt + self.a0
    }

    #[inline]
    fn eval_derivative(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        ((((self.a5 * dt * 5.0 + self.a4 * 4.0) * dt) + self.a3 * 3.0) * dt + self.a2 * 2.0) * dt
            + self.a1
    }

    #[inline]
    fn eval_second_derivative(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        (((self.a5 * dt * 20.0 + self.a4 * 12.0) * dt) + self.a3 * 6.0) * dt + self.a2 * 2.0
    }

    /// The acceleration at `at`, if the segment contains it.
    #[inline]
    pub fn acceleration(&self, at: Epoch) -> Option<V>
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        self.contains(at)
            .then(|| self.eval_second_derivative(at.as_offset_seconds()))
    }

    /// The position, velocity and acceleration at `at`, if the segment contains it.
    #[inline]
    pub fn kinematic_state(&self, at: Epoch) -> Option<KinematicState<V>>
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        self.contains(at).then(|| {
            let t = at.as_offset_seconds();
            KinematicState::new(
                self.eval(t),
                self.eval_derivative(t),
                self.eval_second_derivative(t),
            )
        })
    }
}

impl<V> BoundedTrajectory for QuinticHermiteSegment<V> {
    #[inline]
    fn start(&self) -> Epoch {
        Epoch::from_offset_seconds(self.bounds.0)
    }

    #[inline]
    fn end(&self) -> Epoch {
        Epoch::from_offset_seconds(self.bounds.1)
    }

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        let t = time.as_offset_seconds();
        let (start, end) = self.bounds;
        t == start || (start < t && t < end)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        1
    }
}

impl<V> EvaluateTrajectory for QuinticHermiteSegment<V>
where
    V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<V> {
        self.contains(at).then(|| self.eval(at.as_offset_seconds()))
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
        let t = at.as_offset_seconds();
        self.contains(at)
            .then(|| StateVector::new(self.eval(t), self.eval_derivative(t)))
    }
}

// A segment alone excludes its end (it belongs to the next segment of a spline).
#[derive(Clone, Copy, Debug)]
pub struct ClosedQuinticHermiteSegment<V> {
    segment: QuinticHermiteSegment<V>,
    end: (Epoch, KinematicState<V>),
}

impl<V> ClosedQuinticHermiteSegment<V> {
    #[inline]
    pub fn new(t1: Epoch, s1: KinematicState<V>, t2: Epoch, s2: KinematicState<V>) -> Self
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        Self {
            segment: QuinticHermiteSegment::new(t1, s1, t2, s2),
            end: (t2, s2),
        }
    }

    #[inline]
    pub fn acceleration(&self, at: Epoch) -> Option<V>
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        if at == self.end.0 {
            Some(self.end.1.acceleration)
        } else {
            self.segment.acceleration(at)
        }
    }

    #[inline]
    pub fn kinematic_state(&self, at: Epoch) -> Option<KinematicState<V>>
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        if at == self.end.0 {
            Some(self.end.1)
        } else {
            self.segment.kinematic_state(at)
        }
    }
}

impl<V> BoundedTrajectory for ClosedQuinticHermiteSegment<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.segment.start()
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.end.0
    }

    #[inline]
    fn segment_count(&self) -> usize {
        1
    }
}

impl<V> EvaluateTrajectory for ClosedQuinticHermiteSegment<V>
where
    V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<V> {
        if at == self.end.0 {
            Some(self.end.1.position)
        } else {
            self.segment.position(at)
        }
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
        if at == self.end.0 {
            Some(self.end.1.into())
        } else {
            self.segment.state_vector(at)
        }
    }
}

pub type Knot<V> = (Epoch, KinematicState<V>);

#[derive(Clone, Debug)]
pub struct QuinticHermiteSpline<V> {
    knots: VecDeque<Knot<V>>,
}

impl<V> Default for QuinticHermiteSpline<V> {
    #[inline]
    fn default() -> Self {
        Self {
            knots: VecDeque::new(),
        }
    }
}

impl<V> DeepSizeOf for QuinticHermiteSpline<V> {
    #[inline]
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.knots.capacity() * size_of::<Knot<V>>()
    }
}

impl<V> BoundedTrajectory for QuinticHermiteSpline<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.knots.front().map_or(Epoch::MIN, |(t, _)| *t)
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.knots.back().map_or(Epoch::MAX, |(t, _)| *t)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        self.knots.len().saturating_sub(1)
    }
}

impl<V> EvaluateTrajectory for QuinticHermiteSpline<V>
where
    V: Add<Output = V> + Sub<Output = V> + Mul<f64, Output = V> + PartialEq + Default + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<V> {
        self.segment_at(at)?.position(at)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
        self.segment_at(at)?.state_vector(at)
    }
}

impl<V> QuinticHermiteSpline<V> {
    #[inline]
    pub fn new(start: Epoch, position: V, velocity: V, acceleration: V) -> Self {
        Self {
            knots: [(start, KinematicState::new(position, velocity, acceleration))].into(),
        }
    }

    #[inline]
    pub fn knots(&self) -> &VecDeque<Knot<V>> {
        &self.knots
    }

    #[inline]
    pub fn push_front(&mut self, at: Epoch, position: V, velocity: V, acceleration: V) {
        self.knots
            .push_front((at, KinematicState::new(position, velocity, acceleration)));
    }

    #[inline]
    pub fn push_back(&mut self, at: Epoch, position: V, velocity: V, acceleration: V) {
        self.knots
            .push_back((at, KinematicState::new(position, velocity, acceleration)));
    }

    #[inline]
    pub fn push_back_after_change(
        &mut self,
        change_acceleration: V,
        at: Epoch,
        position: V,
        velocity: V,
        acceleration: V,
    ) where
        V: Copy,
    {
        if let Some(&(t, k)) = self.knots.back() {
            self.push_back(t, k.position, k.velocity, change_acceleration);
            self.push_back(at, position, velocity, acceleration);
        }
    }

    #[inline]
    pub fn knot_ending_at(&self, at: Epoch) -> Option<&KinematicState<V>> {
        let i = self.knots.partition_point(|(t, _)| *t < at);
        self.knots.get(i).filter(|(t, _)| *t == at).map(|(_, k)| k)
    }

    #[inline]
    pub fn knot_starting_at(&self, at: Epoch) -> Option<&KinematicState<V>> {
        let i = self
            .knots
            .partition_point(|(t, _)| *t <= at)
            .checked_sub(1)?;
        self.knots.get(i).filter(|(t, _)| *t == at).map(|(_, k)| k)
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        self.knots
            .truncate(self.knots.partition_point(|(t, _)| *t <= at));
        self.remove_change_at_end();
    }

    #[inline]
    pub fn clear_after_inclusive(&mut self, at: Epoch) {
        self.knots
            .truncate(self.knots.partition_point(|(t, _)| *t < at));
        self.remove_change_at_end();
    }

    #[inline]
    pub fn clear_before(&mut self, at: Epoch) {
        self.knots
            .drain(..self.knots.partition_point(|(t, _)| *t < at));
        if self.knots.get(1).is_some_and(|(t, _)| *t == at) {
            self.knots.pop_front();
        }
    }

    #[inline]
    pub fn clear_before_inclusive(&mut self, at: Epoch) {
        self.knots
            .drain(..self.knots.partition_point(|(t, _)| *t <= at));
    }

    #[inline]
    pub fn append(&mut self, other: QuinticHermiteSpline<V>) {
        self.knots.extend(other.knots);
    }

    #[inline]
    pub fn prepend(&mut self, other: QuinticHermiteSpline<V>) {
        self.knots.reserve(other.knots.len());
        for knot in other.knots.into_iter().rev() {
            self.knots.push_front(knot);
        }
    }

    #[inline]
    pub fn join(&mut self, other: QuinticHermiteSpline<V>) {
        let start = other.start();
        let i = self.knots.partition_point(|(t, _)| *t < start);
        let change_at_start = self.knots.get(i + 1).is_some_and(|(t, _)| *t == start);
        self.knots.truncate(i + usize::from(change_at_start));
        self.knots.extend(other.knots);
        // A single knot after a change leaves the change at the end.
        self.remove_change_at_end();
    }

    #[inline]
    fn remove_change_at_end(&mut self) {
        let n = self.knots.len();
        if n >= 2 && self.knots[n - 2].0 == self.knots[n - 1].0 {
            self.knots.pop_back();
        }
    }
}

impl<V> QuinticHermiteSpline<V>
where
    V: Add<Output = V> + Sub<Output = V> + Mul<f64, Output = V> + PartialEq + Default + Copy,
{
    #[inline]
    pub fn segment_at(&self, at: Epoch) -> Option<QuinticHermiteSegment<V>> {
        let i = self
            .knots
            .partition_point(|(t, _)| *t <= at)
            .checked_sub(1)?;
        let (t1, k1) = self.knots[i];
        let (t2, k2) = match self.knots.get(i + 1) {
            Some(&next) => next,
            None if t1 == at => (t1, k1),
            None => return None,
        };
        Some(QuinticHermiteSegment::new(t1, k1, t2, k2))
    }

    #[inline]
    pub fn last_closed_segment(&self) -> Option<ClosedQuinticHermiteSegment<V>> {
        let i = self.knots.len().checked_sub(2)?;
        let (t1, k1) = self.knots[i];
        let (t2, k2) = self.knots[i + 1];
        Some(ClosedQuinticHermiteSegment::new(t1, k1, t2, k2))
    }

    #[inline]
    pub fn end_at(&mut self, at: Epoch) {
        let interpolated = self
            .segment_at(at)
            .filter(|segment| segment.start() < at)
            .and_then(|segment| segment.kinematic_state(at));
        match interpolated {
            Some(knot) => {
                self.knots
                    .truncate(self.knots.partition_point(|(t, _)| *t <= at));
                self.knots.push_back((at, knot));
            }
            None => self.clear_after(at),
        }
    }
}
