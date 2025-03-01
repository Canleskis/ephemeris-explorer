use deepsize::DeepSizeOf;
use hifitime::{Duration, Epoch};
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

pub trait BoundedTrajectory {
    fn start(&self) -> Epoch;

    fn end(&self) -> Epoch;

    #[inline]
    fn contains(&self, time: Epoch) -> bool {
        time >= self.start() && time <= self.end()
    }

    fn len(&self) -> usize;

    #[inline]
    fn is_empty(&self) -> bool {
        self.len() == 0
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
    fn len(&self) -> usize {
        (**self).len()
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
            let mid1 = left + total_secs / 3;
            let mid2 = right - total_secs / 3;

            match distance(mid1) - distance(mid2) {
                d if d.abs() < precision || i > max_iterations => {
                    return Some(mid1 + (mid2 - mid1) / 2);
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
        let mut eval = T::default();
        for ci in self.0.iter().rev() {
            eval = eval * t + *ci;
        }

        eval
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
    segments: VecDeque<Polynomial<V>>,
}

impl<V> DeepSizeOf for UniformSpline<V> {
    #[inline]
    fn deep_size_of_children(&self, context: &mut deepsize::Context) -> usize {
        self.segments.deep_size_of_children(context)
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
    fn len(&self) -> usize {
        self.segments.len()
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
        self.evaluate(at, Polynomial::eval_and_deriv)
            .map(|(position, velocity)| {
                // Segment are normalised to τ = t / interval so the derivative is dx/dτ.
                // As such, dx/dt = dx/dτ * dτ/dt = dx/dτ * d(t / interval)/dt = dx/dτ / interval.
                StateVector::new(position, velocity / self.interval.to_seconds())
            })
    }
}

impl<V> UniformSpline<V> {
    pub fn new(start: Epoch, interval: Duration) -> Self {
        Self {
            start,
            interval,
            segments: VecDeque::new(),
        }
    }

    #[inline]
    pub fn between(&self, start: Epoch, end: Epoch) -> Option<Self>
    where
        V: Clone,
    {
        if self.segments.is_empty() {
            return None;
        }

        let start = self.index_local_exclusive(start - self.start)?;
        let end = self.index_local_exclusive(end - self.start)?;

        Some(Self {
            start: self.start + self.interval * start as i64,
            interval: self.interval,
            segments: (start..end + 1)
                .filter_map(|i| self.segments.get(i).cloned())
                .collect(),
        })
    }

    #[inline]
    pub fn push_front(&mut self, segment: Polynomial<V>) {
        self.segments.push_front(segment);
        self.start -= self.interval;
    }

    #[inline]
    pub fn push_back(&mut self, segment: Polynomial<V>) {
        self.segments.push_back(segment);
    }

    #[inline]
    pub fn prepend(&mut self, trajectory: Self) {
        assert_eq!(self.start, trajectory.start + trajectory.span());
        assert_eq!(self.interval, trajectory.interval);

        self.start = trajectory.start;

        self.segments.reserve(trajectory.segments.len());
        for segment in trajectory.segments.into_iter().rev() {
            self.segments.push_front(segment);
        }
    }

    #[inline]
    pub fn append(&mut self, trajectory: Self) {
        assert_eq!(self.start + self.span(), trajectory.start,);
        assert_eq!(self.interval, trajectory.interval);

        self.segments.extend(trajectory.segments);
    }

    #[inline]
    pub fn clear_before(&mut self, at: Epoch) {
        if let Some(idx) = self.index_exclusive(at + self.interval) {
            self.start += self.interval * idx as i64;
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
        F: Fn(&Polynomial<V>, f64) -> T,
    {
        let local = at - self.start;
        // Evaluate "previous" segment at a knot.
        let local_index = self.index_local_exclusive(local)?;
        let local_segment = local - self.interval * local_index as i64;
        let normalised = local_segment.to_seconds() / self.interval.to_seconds();

        Some(eval(self.segments.get(local_index)?, normalised))
    }

    #[inline]
    pub fn index(&self, at: Epoch) -> Option<usize> {
        self.index_local(at - self.start)
    }

    #[inline]
    pub fn index_exclusive(&self, at: Epoch) -> Option<usize> {
        self.index_local_exclusive(at - self.start)
    }

    #[inline]
    fn index_local(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time >= self.span() {
            return None;
        }

        Some((time.total_nanoseconds() / self.interval.total_nanoseconds()) as usize)
    }

    #[inline]
    fn index_local_exclusive(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time > self.span() {
            return None;
        }

        Some(
            (time.total_nanoseconds().saturating_sub(1) / self.interval.total_nanoseconds())
                as usize,
        )
    }

    #[inline]
    pub fn interval(&self) -> Duration {
        self.interval
    }

    #[inline]
    pub fn span(&self) -> Duration {
        self.interval * self.segments.len() as i64
    }

    #[inline]
    pub fn segments(&self) -> &VecDeque<Polynomial<V>> {
        &self.segments
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

#[derive(Clone, Debug)]
pub struct CubicHermiteSplineSamples<V>(Vec<(Epoch, StateVector<V>)>);

impl<V> DeepSizeOf for CubicHermiteSplineSamples<V> {
    fn deep_size_of_children(&self, _: &mut deepsize::Context) -> usize {
        self.0.capacity() * size_of::<(Epoch, StateVector<V>)>()
    }
}

impl<V> BoundedTrajectory for CubicHermiteSplineSamples<V> {
    #[inline]
    fn start(&self) -> Epoch {
        self.0.first().unwrap().0
    }

    #[inline]
    fn end(&self) -> Epoch {
        self.0.last().unwrap().0
    }

    #[inline]
    fn len(&self) -> usize {
        self.0.len().saturating_sub(1)
    }
}

impl<V> EvaluateTrajectory for CubicHermiteSplineSamples<V>
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
            Err(i) => {
                let &(t1, sv1) = self.0.get(i.checked_sub(1)?)?;
                let &(t2, sv2) = self.0.get(i)?;
                let hermite = CubicHermite::new(
                    (t1.to_tai_seconds(), t2.to_tai_seconds()),
                    (sv1.position, sv2.position),
                    (sv1.velocity, sv2.velocity),
                );

                Some(hermite.eval(at.to_tai_seconds()))
            }
        }
    }

    #[inline]
    fn state_vector(&self, at: Epoch) -> Option<StateVector<V>> {
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

impl<V> CubicHermiteSplineSamples<V> {
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
    fn hermite3(&self, i: usize) -> Option<CubicHermite<V>>
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
    pub fn extend(&mut self, rhs: CubicHermiteSplineSamples<V>) {
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
