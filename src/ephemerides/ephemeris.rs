use super::integration::IntegrationState;

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};
use particular::prelude::*;
use std::collections::VecDeque;

pub const MAX_DIV: usize = 9;
type SegmentStorage = poly_it::storage::smallvec::SmallVec<[f64; MAX_DIV - 1]>;

#[derive(Clone, Debug)]
pub struct Segment {
    pub x: poly_it::Polynomial<f64, SegmentStorage>,
    pub y: poly_it::Polynomial<f64, SegmentStorage>,
    pub z: poly_it::Polynomial<f64, SegmentStorage>,
}

impl serde::Serialize for Segment {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (self.x.coeffs(), self.y.coeffs(), self.z.coeffs()).serialize(serializer)
    }
}

impl Segment {
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

    pub fn size(&self) -> usize {
        std::mem::size_of_val(self.x.coeffs())
            + std::mem::size_of_val(self.y.coeffs())
            + std::mem::size_of_val(self.z.coeffs())
    }
}

#[inline]
fn fast_eval_and_deriv(coeffs: &[f64], x: f64) -> (f64, f64) {
    let first = *coeffs.first().unwrap();
    let last = *coeffs.last().unwrap();

    let mut eval = last;
    let mut deriv = last;
    for ci in coeffs.iter().skip(1).rev().skip(1) {
        eval = eval * x + ci;
        deriv = deriv * x + eval;
    }
    eval = eval * x + first;

    (eval, deriv)
}

#[inline]
pub fn interpolate(deg: usize, ts: &[f64], xs: &[f64]) -> poly_it::Polynomial<f64, SegmentStorage> {
    poly_it::Polynomial::least_squares_fit(
        deg,
        std::iter::zip(ts.iter().copied(), xs.iter().copied()),
    )
    .unwrap()
}

#[derive(Clone, Copy, Debug)]
pub struct SegmentPoints {
    index: usize,
    xs: [f64; MAX_DIV],
    ys: [f64; MAX_DIV],
    zs: [f64; MAX_DIV],
}

impl SegmentPoints {
    pub fn with(position: DVec3) -> Self {
        Self {
            index: 1,
            xs: [position.x; MAX_DIV],
            ys: [position.y; MAX_DIV],
            zs: [position.z; MAX_DIV],
        }
    }

    pub fn push(&mut self, position: DVec3) {
        assert!(self.index < MAX_DIV, "Too many points in segment.");
        self.xs[self.index] = position.x;
        self.ys[self.index] = position.y;
        self.zs[self.index] = position.z;
        self.index += 1;
    }

    pub fn len(&self) -> usize {
        self.index
    }

    pub fn is_full(&self) -> bool {
        self.len() == MAX_DIV
    }

    fn as_segment(&self, deg: usize) -> Segment {
        #[expect(dead_code)]
        const REV_TS: [f64; MAX_DIV] = {
            let mut out = [0.0; MAX_DIV];
            let mut i = 0;
            while i < MAX_DIV {
                out[i] = (MAX_DIV - i - 1) as f64 / (MAX_DIV - 1) as f64;
                i += 1;
            }
            out
        };

        const TS: [f64; MAX_DIV] = {
            let mut out = [0.0; MAX_DIV];
            let mut i = 0;
            while i < MAX_DIV {
                out[i] = (i) as f64 / (MAX_DIV - 1) as f64;
                i += 1;
            }
            out
        };

        Segment {
            x: interpolate(deg, &TS, &self.xs),
            y: interpolate(deg, &TS, &self.ys),
            z: interpolate(deg, &TS, &self.zs),
        }
    }
}

/// A structure that represents an ephemeris being built.
#[derive(Clone, Copy, Debug, Component, Position, Mass)]
pub struct EphemerisBuilder {
    pub time: Duration,
    pub deg: usize,
    pub velocity: DVec3,
    pub position: DVec3,
    pub mu: f64,
    pub points: SegmentPoints,
}

impl IntegrationState for EphemerisBuilder {
    fn position(&mut self) -> &mut DVec3 {
        &mut self.position
    }

    fn velocity(&mut self) -> &mut DVec3 {
        &mut self.velocity
    }

    fn step(&mut self, delta: Duration) {
        self.time += delta;
    }
}

impl EphemerisBuilder {
    pub fn new(deg: usize, position: DVec3, velocity: DVec3, mu: f64) -> Self {
        Self {
            time: Duration::ZERO,
            deg,
            points: SegmentPoints::with(position),
            position,
            mu,
            velocity,
        }
    }

    pub fn acceleration(&self, slice: &[Self], _: f64) -> DVec3 {
        Between(self, slice).brute_force(particular::gravity::newtonian::Acceleration::checked())
    }

    pub fn update_trajectory(&mut self, trajectory: &mut Trajectory) {
        let time = self.time.total_nanoseconds();
        let interval = trajectory.granule().total_nanoseconds() / (MAX_DIV - 1) as i128;
        if time % interval == 0 {
            self.points.push(self.position);
            if self.points.is_full() {
                trajectory
                    .segments
                    .push_back(self.points.as_segment(self.deg));
                self.time = Duration::ZERO;
                self.points = SegmentPoints::with(self.position);
            }
        }
    }
}

#[derive(Clone, Copy, Default, Debug)]
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

#[derive(Clone, Debug, Component, serde::Serialize)]
pub struct Trajectory {
    start: Epoch,
    granule: Duration,
    segments: VecDeque<Segment>,
}

impl Trajectory {
    pub fn new(start: Epoch, granule: Duration) -> Self {
        Self {
            start,
            granule,
            segments: VecDeque::new(),
        }
    }

    pub fn continued(&self) -> Self {
        Self {
            start: self.end(),
            segments: VecDeque::new(),
            ..*self
        }
    }

    // We could subtract the current trajectory by an identity trajectory to get a trajectory that
    // matches the requested span exactly but depending on the requested span this could result in
    // a big trajectory when we can simply use whole segments with a total span that might be larger
    // than the requested one.
    // pub fn between_exact(&self, start: Epoch, end: Epoch) -> Self {
    //     self - &Trajectory {
    //         start,
    //         granule: end - start,
    //         segments: VecDeque::from([Segment {
    //             x: Polynomial::new_no_trim(SegementStorage::new()),
    //             y: Polynomial::new_no_trim(SegementStorage::new()),
    //             z: Polynomial::new_no_trim(SegementStorage::new()),
    //         }]),
    //     }
    // }
    pub fn between(&self, start: Epoch, end: Epoch) -> Option<Self> {
        if self.is_empty() {
            return None;
        }

        let start = self.segment_index(start - self.start)?;
        let end = self.segment_index(end - self.start)?;

        Some(Self {
            start: self.start + self.granule * start as i64,
            granule: self.granule,
            segments: (start..end + 1)
                .filter_map(|i| self.segments.get(i).cloned())
                .collect(),
        })
    }

    pub fn split_off(&mut self, at: usize) -> Self {
        let old_start = self.start;
        self.start += self.granule * (self.segments.len() - at) as i64;
        Self {
            start: old_start + self.granule * at as i64,
            segments: self.segments.split_off(at),
            ..*self
        }
    }

    pub fn prepend(&mut self, data: Self) {
        assert_eq!(self.start, data.end());
        assert_eq!(self.granule, data.granule);

        self.start -= self.granule * data.segments.len() as i64;

        self.segments.reserve(data.segments.len());
        for segment in data.segments.into_iter().rev() {
            self.segments.push_front(segment);
        }
    }

    pub fn append(&mut self, data: Self) {
        assert_eq!(self.end(), data.start);
        assert_eq!(self.granule, data.granule);

        self.segments.extend(data.segments);
    }

    pub fn contains(&self, time: Epoch) -> bool {
        time >= self.start && time <= self.end()
    }

    pub fn evaluate_position(&self, at: Epoch) -> Option<DVec3> {
        self.evaluate(at, Segment::eval)
    }

    pub fn evaluate_state_vector(&self, at: Epoch) -> Option<StateVector<DVec3>> {
        self.evaluate(at, Segment::eval_and_deriv)
            .map(|(position, mut velocity)| {
                // Segment are normalised to τ = t / granule so the derivative is dx/dτ.
                // As such, dx/dt = dx/dτ * dτ/dt = dx/dτ / granule.
                velocity /= self.granule.to_seconds();

                StateVector { position, velocity }
            })
    }

    pub fn evaluate<F, T>(&self, at: Epoch, eval: F) -> Option<T>
    where
        F: Fn(&Segment, f64) -> T,
    {
        let local = at - self.start;
        let segment_index = self.segment_index(local)?;
        let local_segment = local - self.granule * segment_index as i64;
        let normalised = local_segment.to_seconds() / self.granule.to_seconds();

        Some(eval(self.segments.get(segment_index)?, normalised))
    }

    fn segment_index(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time > self.span() {
            return None;
        }

        Some(
            // We subtract 1 so that "previous" segments are used for times equal to multiples of the granule.
            (time.total_nanoseconds().saturating_sub(1) / self.granule.total_nanoseconds())
                as usize,
        )
    }

    fn segment_index_inclusive(&self, time: Duration) -> Option<usize> {
        if time.is_negative() || time >= self.span() {
            return None;
        }

        Some((time.total_nanoseconds() / self.granule.total_nanoseconds()) as usize)
    }

    pub fn start(&self) -> Epoch {
        self.start
    }

    pub fn granule(&self) -> Duration {
        self.granule
    }

    pub fn span(&self) -> Duration {
        self.granule * self.segments.len() as i64
    }

    pub fn end(&self) -> Epoch {
        self.start + self.span()
    }

    pub fn len(&self) -> usize {
        self.segments.len()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    pub fn segments(&self) -> &VecDeque<Segment> {
        &self.segments
    }

    pub fn size(&self) -> usize {
        self.segments.len() * self.segments.front().map(Segment::size).unwrap_or(0)
    }
}

// Unused because too slow. Faster to subtract the evaluated values.
// Also not thoroughly tested.
impl std::ops::Sub for &Trajectory {
    type Output = Trajectory;

    #[inline]
    fn sub(self, rhs: &Trajectory) -> Self::Output {
        let granule = gcd_duration(self.granule, rhs.granule);
        let start = self.start.max(rhs.start);

        let mut segments = VecDeque::new();

        let mut t = start;
        while let Some((a, b)) = self
            .segment_index_inclusive(t - self.start)
            .zip(rhs.segment_index_inclusive(t - rhs.start))
        {
            // Polynomials are defined in the range [0, 1] so we need to redefine them because
            // they might have different granules.
            let redef = |data: &Trajectory, idx| {
                let data_granule = data.granule.to_seconds();
                let sub = {
                    let mut coeffs = SegmentStorage::new();
                    coeffs.push(((t - data.start).to_seconds() % data_granule) / data_granule);
                    coeffs.push(granule.to_seconds() / data_granule);

                    Polynomial::new(coeffs)
                };

                Segment {
                    x: substitute(&data.segments[idx].x, &sub),
                    y: substitute(&data.segments[idx].y, &sub),
                    z: substitute(&data.segments[idx].z, &sub),
                }
            };

            let s = redef(self, a);
            let r = redef(rhs, b);
            let segment = Segment {
                x: s.x - r.x,
                y: s.y - r.y,
                z: s.z - r.z,
            };

            segments.push_back(segment);
            t += granule;
        }

        Trajectory {
            start,
            granule,
            segments,
        }
    }
}

// Unused because too slow. Faster to subtract the evaluated values.
// Also not thoroughly tested.
impl std::ops::Add for &Trajectory {
    type Output = Trajectory;

    #[inline]
    fn add(self, rhs: &Trajectory) -> Self::Output {
        let granule = gcd_duration(self.granule, rhs.granule);
        let start = self.start.max(rhs.start);

        let mut segments = VecDeque::new();

        let mut t = start;
        while let Some((a, b)) = self
            .segment_index_inclusive(t - self.start)
            .zip(rhs.segment_index_inclusive(t - rhs.start))
        {
            // Polynomials are defined in the range [0, 1] so we need to redefine them because
            // they might have different granules.
            let redef = |data: &Trajectory, idx| {
                let data_granule = data.granule.to_seconds();
                let sub = {
                    let mut coeffs = SegmentStorage::new();
                    coeffs.push(((t - data.start).to_seconds() % data_granule) / data_granule);
                    coeffs.push(granule.to_seconds() / data_granule);

                    Polynomial::new(coeffs)
                };

                Segment {
                    x: substitute(&data.segments[idx].x, &sub),
                    y: substitute(&data.segments[idx].y, &sub),
                    z: substitute(&data.segments[idx].z, &sub),
                }
            };

            let s = redef(self, a);
            let r = redef(rhs, b);
            let segment = Segment {
                x: s.x + r.x,
                y: s.y + r.y,
                z: s.z + r.z,
            };

            segments.push_back(segment);
            t += granule;
        }

        Trajectory {
            start,
            granule,
            segments,
        }
    }
}

impl std::ops::Mul<f64> for &Trajectory {
    type Output = Trajectory;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Trajectory {
            start: self.start,
            granule: self.granule,
            segments: self
                .segments
                .iter()
                .map(|segment| Segment {
                    x: polynomial_mul(segment.x.clone(), rhs),
                    y: polynomial_mul(segment.y.clone(), rhs),
                    z: polynomial_mul(segment.z.clone(), rhs),
                })
                .collect(),
        }
    }
}

impl std::ops::Div<f64> for &Trajectory {
    type Output = Trajectory;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Trajectory {
            start: self.start,
            granule: self.granule,
            segments: self
                .segments
                .iter()
                .map(|segment| Segment {
                    x: polynomial_div(segment.x.clone(), rhs),
                    y: polynomial_div(segment.y.clone(), rhs),
                    z: polynomial_div(segment.z.clone(), rhs),
                })
                .collect(),
        }
    }
}

fn gcd_duration(a: Duration, b: Duration) -> Duration {
    let mut a = a.total_nanoseconds();
    let mut b = b.total_nanoseconds();

    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }

    Duration::from_total_nanoseconds(a)
}

use poly_it::{
    num_traits::{One, Pow, Zero},
    storage::{Storage, StorageProvider},
    Polynomial,
};

#[inline]
fn polynomial_mul<T, S>(mut poly: Polynomial<T, S>, rhs: T) -> Polynomial<T, S>
where
    T: Clone + std::ops::Mul<T, Output = T>,
    S: Storage<T>,
{
    for coeff in poly.coeffs_mut() {
        *coeff = coeff.clone() * rhs.clone();
    }

    poly
}

#[inline]
fn polynomial_div<T, S>(mut poly: Polynomial<T, S>, rhs: T) -> Polynomial<T, S>
where
    T: Clone + std::ops::Div<T, Output = T>,
    S: Storage<T>,
{
    for coeff in poly.coeffs_mut() {
        *coeff = coeff.clone() / rhs.clone();
    }

    poly
}

#[inline]
fn polynomial_pow<T, S>(poly: Polynomial<T, S>, exp: usize) -> Polynomial<T, S>
where
    T: Zero + One + Clone + std::ops::Mul<T, Output = T>,
    S: Storage<T>,
{
    let mut coeffs = S::Provider::new().storage_with_capacity(poly.coeffs().len());
    coeffs.push(T::one());
    let mut result = Polynomial::new(coeffs);

    for _ in 0..exp {
        result = result * poly.clone();
    }

    result
}

#[inline]
fn substitute<T, S>(poly: &Polynomial<T, S>, a: &Polynomial<T, S>) -> Polynomial<T, S>
where
    T: Zero + One + Pow<i32, Output = T> + std::ops::Mul<T, Output = T> + Clone,
    S: Storage<T>,
{
    let coeffs = S::Provider::new().storage_with_capacity(poly.coeffs().len());
    let mut result = Polynomial::new(coeffs);

    for (i, coeff) in poly.coeffs().iter().enumerate() {
        let mut coef = S::Provider::new().storage_with_capacity(1);
        coef.push(coeff.clone());
        let coeff = Polynomial::new(coef);
        result = result + (coeff.clone() * polynomial_pow(a.clone(), i));
    }

    result
}
