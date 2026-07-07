use deepsize::DeepSizeOf;
use ftime::{Duration, Epoch};
use std::collections::VecDeque;

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

pub const DIV: usize = 8;

#[derive(Clone, Debug, PartialEq)]
pub struct Polynomial<T>(smallvec::SmallVec<[T; DIV]>);

impl<T> DeepSizeOf for Polynomial<T> {
    #[inline]
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.0.capacity() * std::mem::size_of::<T>()
    }
}

impl<T> Polynomial<T> {
    #[inline]
    pub fn new(coeffs: smallvec::SmallVec<[T; DIV]>) -> Self {
        Self(coeffs)
    }

    #[inline]
    pub fn coeffs(&self) -> &smallvec::SmallVec<[T; DIV]> {
        &self.0
    }

    #[inline]
    pub fn eval(&self, t: f64) -> T
    where
        T: std::ops::Add<Output = T> + std::ops::Mul<f64, Output = T> + Default + Copy,
    {
        eval_slice_horner(&self.0, t)
    }

    #[inline]
    pub fn eval_and_deriv(&self, t: f64) -> (T, T)
    where
        T: std::ops::Add<Output = T> + std::ops::Mul<f64, Output = T> + Default + Copy,
    {
        let first = self.0.first().copied().unwrap_or_default();
        let last = self.0.last().copied().unwrap_or_default();

        let mut eval = last;
        let mut deriv = last;
        for ci in self.0.iter().skip(1).rev().skip(1) {
            eval = eval * t + *ci;
            deriv = deriv * t + eval;
        }
        eval = eval * t + first;

        (eval, deriv)
    }

    #[inline]
    pub fn trim(&mut self)
    where
        T: Default + PartialEq,
    {
        while let Some(true) = self.0.as_slice().last().map(|c| c == &T::default()) {
            let _ = self.0.pop();
        }
    }
}

#[inline]
pub fn eval_slice_horner<T, U>(coeffs: &[T], t: U) -> T
where
    T: std::ops::Add<Output = T> + std::ops::Mul<U, Output = T> + Default + Copy,
    U: Copy,
{
    let mut result = T::default();
    for ci in coeffs.iter().rev() {
        result = result * t + *ci;
    }

    result
}

#[derive(Clone, Debug)]
pub struct UniformSpline<V> {
    start: Epoch,
    interval: Duration,
    polynomials: VecDeque<Polynomial<V>>,
}

impl<V> DeepSizeOf for UniformSpline<V> {
    #[inline]
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.polynomials.deep_size_of_children(context)
    }
}

impl<V> BoundedTrajectory for UniformSpline<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.start
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.start + self.span()
    }

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        let local = time - self.start;
        local.is_positive() && local <= self.span()
    }

    #[inline]
    fn segment_count(&self) -> usize {
        self.polynomials.len()
    }
}

impl<V> EvaluateTrajectory for UniformSpline<V>
where
    V: std::ops::Add<Output = V>
        + std::ops::Mul<f64, Output = V>
        + std::ops::Div<f64, Output = V>
        + Default
        + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        self.evaluate(at, Polynomial::eval)
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        let (pos, vel) = self.evaluate(at, Polynomial::eval_and_deriv)?;
        // Polynomials are normalised to τ = t / interval so the derivative is dx/dτ.
        // As such, dx/dt = dx/dτ * dτ/dt = dx/dτ * d(t / interval)/dt = dx/dτ / interval.
        Some(StateVector::new(pos, vel / self.interval.as_seconds()))
    }
}

impl<V> UniformSpline<V> {
    #[inline]
    pub fn new(start: Epoch, interval: Duration) -> Self {
        Self {
            start,
            interval,
            polynomials: VecDeque::new(),
        }
    }

    #[inline]
    pub fn between(&self, start: Epoch, end: Epoch) -> Option<Self>
    where
        V: Clone,
    {
        if self.polynomials.is_empty() {
            return None;
        }

        let start = self.get_index_local_exclusive(start - self.start)?;
        let end = self.get_index_local_exclusive(end - self.start)?;

        Some(Self {
            start: self.start + self.interval * start as f64,
            interval: self.interval,
            polynomials: (start..end + 1)
                .filter_map(|i| self.polynomials.get(i).cloned())
                .collect(),
        })
    }

    #[inline]
    pub fn push_front(&mut self, polynomial: Polynomial<V>) {
        self.polynomials.push_front(polynomial);
        self.start -= self.interval;
    }

    #[inline]
    pub fn push_back(&mut self, polynomial: Polynomial<V>) {
        self.polynomials.push_back(polynomial);
    }

    #[inline]
    pub fn prepend(&mut self, trajectory: Self) {
        assert_eq!(self.start, trajectory.start + trajectory.span());
        assert_eq!(self.interval, trajectory.interval);

        self.start = trajectory.start;

        self.polynomials.reserve(trajectory.polynomials.len());
        for polynomial in trajectory.polynomials.into_iter().rev() {
            self.polynomials.push_front(polynomial);
        }
    }

    #[inline]
    pub fn append(&mut self, trajectory: Self) {
        assert_eq!(self.start + self.span(), trajectory.start);
        assert_eq!(self.interval, trajectory.interval);

        self.polynomials.extend(trajectory.polynomials);
    }

    #[inline]
    pub fn clear_before(&mut self, at: Epoch) {
        if let Some(idx) = self.get_index_exclusive(at + self.interval) {
            self.start += self.interval.scaled(idx as f64);
            self.polynomials.drain(0..idx);
        }
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        if let Some(idx) = self.get_index(at) {
            self.polynomials.truncate(idx);
        }
    }

    #[inline]
    pub fn evaluate<F, T>(&self, at: Epoch, eval: F) -> Option<T>
    where
        F: Fn(&Polynomial<V>, f64) -> T,
    {
        let (polynomial, t) = self.get_polynomial(at)?;
        Some(eval(polynomial, t))
    }

    #[inline]
    pub fn get_polynomial(&self, at: Epoch) -> Option<(&Polynomial<V>, f64)> {
        let local = at - self.start;
        // Return "previous" polynomial when at a knot, ensuring `self.end()` can be evaluated.
        let local_index = self.get_index_local_exclusive(local)?;
        let local_polynomial = local - self.interval * local_index as f64;
        let normalised = local_polynomial.as_seconds() / self.interval.as_seconds();

        Some((self.polynomials.get(local_index)?, normalised))
    }

    #[inline]
    pub fn get_index(&self, at: Epoch) -> Option<usize> {
        self.get_index_local(at - self.start)
    }

    #[inline]
    pub fn get_index_exclusive(&self, at: Epoch) -> Option<usize> {
        self.get_index_local_exclusive(at - self.start)
    }

    #[inline]
    pub fn index(&self, at: Epoch) -> usize {
        self.index_local(at - self.start)
    }

    #[inline]
    pub fn index_exclusive(&self, at: Epoch) -> usize {
        self.index_local_exclusive(at - self.start)
    }

    #[inline]
    fn get_index_local(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time >= self.span() {
            return None;
        }

        Some(self.index_local(time))
    }

    #[inline]
    fn get_index_local_exclusive(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time > self.span() {
            return None;
        }

        Some(self.index_local_exclusive(time))
    }

    #[inline]
    fn index_local(&self, time: Duration) -> usize {
        (time.as_seconds() / self.interval.as_seconds()) as usize
    }

    #[inline]
    fn index_local_exclusive(&self, time: Duration) -> usize {
        ((time.as_seconds() / self.interval.as_seconds()).ceil() as usize).saturating_sub(1)
    }

    #[inline]
    pub fn interval(&self) -> Duration {
        self.interval
    }

    #[inline]
    pub fn span(&self) -> Duration {
        self.interval.scaled(self.polynomials.len() as f64)
    }

    #[inline]
    pub fn polynomials(&self) -> &VecDeque<Polynomial<V>> {
        &self.polynomials
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CubicHermite<V> {
    bounds: (f64, f64),
    a0: V,
    a1: V,
    a2: V,
    a3: V,
}

impl<V> CubicHermite<V> {
    #[inline]
    pub fn new(bounds: (f64, f64), values: (V, V), derivatives: (V, V)) -> Self
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
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

        CubicHermite {
            bounds,
            a0,
            a1,
            a2,
            a3,
        }
    }

    #[inline]
    pub fn eval(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        (((self.a3 * dt + self.a2) * dt) + self.a1) * dt + self.a0
    }

    #[inline]
    pub fn eval_derivative(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        ((self.a3 * dt * 3.0 + self.a2 * 2.0) * dt) + self.a1
    }

    #[inline]
    pub fn eval_derivative2(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        self.a3 * dt * 6.0 + self.a2 * 2.0
    }
}

impl<V> EvaluateTrajectory for CubicHermite<V>
where
    V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        Some(self.eval(at.as_offset_seconds()))
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        Some(StateVector::new(
            self.eval(at.as_offset_seconds()),
            self.eval_derivative(at.as_offset_seconds()),
        ))
    }
}

impl<V> BoundedTrajectory for CubicHermite<V> {
    #[inline]
    fn start(&self) -> Epoch {
        Epoch::from_offset_seconds(self.bounds.0)
    }

    #[inline]
    fn end(&self) -> Epoch {
        Epoch::from_offset_seconds(self.bounds.1)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        1
    }
}

#[derive(Clone, Debug)]
pub struct CubicHermiteSpline<V>(Vec<(Epoch, StateVector<V>)>);

impl<V> Default for CubicHermiteSpline<V> {
    #[inline]
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<V> DeepSizeOf for CubicHermiteSpline<V> {
    #[inline]
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.0.capacity() * size_of::<(Epoch, StateVector<V>)>()
    }
}

impl<V> BoundedTrajectory for CubicHermiteSpline<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.0.first().map(|i| i.0).unwrap_or(Epoch::MIN)
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.0.last().map(|i| i.0).unwrap_or(Epoch::MAX)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        self.0.len().saturating_sub(1)
    }
}

impl<V> EvaluateTrajectory for CubicHermiteSpline<V>
where
    V: std::ops::Add<Output = V>
        + std::ops::Sub<Output = V>
        + std::ops::Mul<f64, Output = V>
        + PartialEq
        + Default
        + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<V> {
        match self.binary_search(at) {
            Ok(i) => Some(self.0[i].1.position),
            Err(i) => self.hermite3(i.checked_sub(1)?)?.position(at),
        }
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
        match self.binary_search(at) {
            Ok(i) => Some(self.0[i].1),
            Err(i) => self.hermite3(i.checked_sub(1)?)?.state_vector(at),
        }
    }
}

impl<V> CubicHermiteSpline<V> {
    #[inline]
    pub fn new(start: Epoch, position: V, velocity: V) -> Self {
        Self(vec![(start, StateVector::new(position, velocity))])
    }

    #[inline]
    pub fn push(&mut self, at: Epoch, position: V, velocity: V) {
        self.0.push((at, StateVector::new(position, velocity)));
    }

    #[inline]
    pub fn binary_search(&self, at: Epoch) -> Result<usize, usize> {
        self.0.binary_search_by(|(t, _)| t.cmp(&at))
    }

    #[inline]
    pub fn hermite3(&self, i: usize) -> Option<CubicHermite<V>>
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        let (t1, sv1) = self.0.get(i)?;
        let (t2, sv2) = self.0.get(i + 1)?;
        Some(CubicHermite::new(
            (t1.as_offset_seconds(), t2.as_offset_seconds()),
            (sv1.position, sv2.position),
            (sv1.velocity, sv2.velocity),
        ))
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        // TODO: Use binary search
        self.0.retain(|(k, _)| &at > k);
    }

    #[inline]
    pub fn append(&mut self, rhs: CubicHermiteSpline<V>) {
        self.0.extend(rhs.0);
    }

    #[inline]
    pub fn get(&self, at: Epoch) -> Option<&StateVector<V>> {
        self.binary_search(at).ok().map(|i| &self.0[i].1)
    }

    #[inline]
    pub fn points(&self) -> &[(Epoch, StateVector<V>)] {
        &self.0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct QuinticHermite<V> {
    bounds: (f64, f64),
    a0: V,
    a1: V,
    a2: V,
    a3: V,
    a4: V,
    a5: V,
}

impl<V> QuinticHermite<V> {
    #[inline]
    pub fn new(
        bounds: (f64, f64),
        values: (V, V),
        derivatives: (V, V),
        accelerations: (V, V),
    ) -> Self
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        let a0 = values.0;
        let a1 = derivatives.0;
        let a2 = accelerations.0 * 0.5;
        let dt = bounds.1 - bounds.0;

        let (a3, a4, a5) = if dt == 0.0
            && values.0 == values.1
            && derivatives.0 == derivatives.1
            && accelerations.0 == accelerations.1
        {
            (V::default(), V::default(), V::default())
        } else {
            let dt_recip = 1.0 / dt;
            let dt_recip_2 = dt_recip * dt_recip;
            let dt_recip_3 = dt_recip * dt_recip_2;
            let dt_recip_4 = dt_recip * dt_recip_3;
            let dt_recip_5 = dt_recip * dt_recip_4;

            let dp = values.1 - values.0;
            let dv0 = derivatives.0;
            let dv1 = derivatives.1;
            let da0 = accelerations.0;
            let da1 = accelerations.1;

            let c0 = dp * 10.0 - (dv0 * 6.0 + dv1 * 4.0) * dt + (da1 * 0.5 - da0 * 1.5) * (dt * dt);
            let c1 = dp * -15.0 + (dv0 * 8.0 + dv1 * 7.0) * dt + (da0 * 1.5 - da1) * (dt * dt);
            let c2 = dp * 6.0 - (dv0 + dv1) * 3.0 * dt + (da1 * 0.5 - da0 * 0.5) * (dt * dt);

            (c0 * dt_recip_3, c1 * dt_recip_4, c2 * dt_recip_5)
        };

        QuinticHermite {
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
    pub fn eval(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        (((((self.a5 * dt + self.a4) * dt) + self.a3) * dt + self.a2) * dt + self.a1) * dt + self.a0
    }

    #[inline]
    pub fn eval_derivative(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        ((((self.a5 * dt * 5.0 + self.a4 * 4.0) * dt) + self.a3 * 3.0) * dt + self.a2 * 2.0) * dt
            + self.a1
    }

    #[inline]
    pub fn eval_derivative2(&self, t: f64) -> V
    where
        V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
    {
        let dt = t - self.bounds.0;
        (((self.a5 * dt * 20.0 + self.a4 * 12.0) * dt) + self.a3 * 6.0) * dt + self.a2 * 2.0
    }
}

impl<V> EvaluateTrajectory for QuinticHermite<V>
where
    V: std::ops::Add<Output = V> + std::ops::Mul<f64, Output = V> + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<Self::Vector> {
        Some(self.eval(at.as_offset_seconds()))
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<Self::Vector>> {
        Some(StateVector::new(
            self.eval(at.as_offset_seconds()),
            self.eval_derivative(at.as_offset_seconds()),
        ))
    }
}

#[derive(Clone, Debug)]
pub struct QuinticHermiteSpline<V>(VecDeque<(Epoch, KinematicState<V>)>);

impl<V> Default for QuinticHermiteSpline<V> {
    #[inline]
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<V> DeepSizeOf for QuinticHermiteSpline<V> {
    #[inline]
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.0.capacity() * size_of::<(Epoch, KinematicState<V>)>()
    }
}

impl<V> BoundedTrajectory for QuinticHermiteSpline<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.0.front().map(|i| i.0).unwrap_or(Epoch::MIN)
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.0.back().map(|i| i.0).unwrap_or(Epoch::MAX)
    }

    #[inline]
    fn segment_count(&self) -> usize {
        self.0.len().saturating_sub(1)
    }
}

impl<V> EvaluateTrajectory for QuinticHermiteSpline<V>
where
    V: std::ops::Add<Output = V>
        + std::ops::Sub<Output = V>
        + std::ops::Mul<f64, Output = V>
        + PartialEq
        + Default
        + Copy,
{
    type Vector = V;

    #[inline]
    fn position(&self, at: Epoch) -> Option<V> {
        match self.binary_search(at) {
            Ok(i) => Some(self.0[i].1.position),
            Err(i) => self.hermite5(i.checked_sub(1)?)?.position(at),
        }
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
        match self.binary_search(at) {
            Ok(i) => {
                let (_, state) = self.0[i];
                Some(StateVector::new(state.position, state.velocity))
            }
            Err(i) => self.hermite5(i.checked_sub(1)?)?.state_vector(at),
        }
    }
}

impl<V> QuinticHermiteSpline<V> {
    #[inline]
    pub fn new(start: Epoch, position: V, velocity: V, acceleration: V) -> Self {
        Self([(start, KinematicState::new(position, velocity, acceleration))].into())
    }

    #[inline]
    pub fn push_front(&mut self, at: Epoch, position: V, velocity: V, acceleration: V) {
        self.0
            .push_front((at, KinematicState::new(position, velocity, acceleration)));
    }

    #[inline]
    pub fn push_back(&mut self, at: Epoch, position: V, velocity: V, acceleration: V) {
        self.0
            .push_back((at, KinematicState::new(position, velocity, acceleration)));
    }

    #[inline]
    pub fn binary_search(&self, at: Epoch) -> Result<usize, usize> {
        self.0.binary_search_by(|(t, ..)| t.cmp(&at))
    }

    #[inline]
    pub fn hermite5(&self, i: usize) -> Option<QuinticHermite<V>>
    where
        V: std::ops::Add<Output = V>
            + std::ops::Sub<Output = V>
            + std::ops::Mul<f64, Output = V>
            + PartialEq
            + Default
            + Copy,
    {
        let (t1, s1) = self.0.get(i)?;
        let (t2, s2) = self.0.get(i + 1)?;
        Some(QuinticHermite::new(
            (t1.as_offset_seconds(), t2.as_offset_seconds()),
            (s1.position, s2.position),
            (s1.velocity, s2.velocity),
            (s1.acceleration, s2.acceleration),
        ))
    }

    #[inline]
    pub fn clear_before(&mut self, at: Epoch) {
        let i = match self.binary_search(at) {
            Ok(i) => i + 1,
            Err(i) => i,
        };
        self.0.drain(..i);
    }

    #[inline]
    pub fn clear_after(&mut self, at: Epoch) {
        let i = match self.binary_search(at) {
            Ok(i) | Err(i) => i,
        };
        self.0.drain(i..);
    }

    #[inline]
    pub fn append(&mut self, trajectory: QuinticHermiteSpline<V>) {
        self.0.extend(trajectory.0);
    }

    #[inline]
    pub fn prepend(&mut self, trajectory: QuinticHermiteSpline<V>) {
        self.0.reserve(trajectory.0.len());
        for item in trajectory.0.into_iter().rev() {
            self.0.push_front(item);
        }
    }

    #[inline]
    pub fn get(&self, at: Epoch) -> Option<&KinematicState<V>> {
        self.binary_search(at).ok().map(|i| &self.0[i].1)
    }

    #[inline]
    pub fn points(&self) -> &VecDeque<(Epoch, KinematicState<V>)> {
        &self.0
    }
}
