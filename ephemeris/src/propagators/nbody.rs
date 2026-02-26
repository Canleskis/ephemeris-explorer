use crate::{
    BranchingPropagator, DirectionalPropagator, IncrementalPropagator, Iterable, Polynomial,
    Propagator,
    trajectory::{BoundedTrajectory, DIV, StateVector, UniformSpline},
};

use ftime::{Duration, Epoch};
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationPaired;

#[derive(Clone, Debug)]
pub struct NewtonianGravity {
    pub gravitational_parameters: Vec<f64>,
}

impl<T, V> SecondOrderODE<T, Vec<V>> for NewtonianGravity
where
    V: std::ops::AddAssign + Default + Copy,
    (V, f64): AccelerationPaired<Softening = f64, Output = V>,
{
    #[inline]
    fn eval(&mut self, _: T, y: &Vec<V>, ddy: &mut Vec<V>) -> Result<(), EvalFailed> {
        ddy.fill(V::default());
        for i in 0..y.len() {
            let mut output_i = V::default();
            let p_i = (y[i], self.gravitational_parameters[i]);

            for j in (i + 1)..y.len() {
                let p_j = (y[j], self.gravitational_parameters[j]);
                let computed = p_i.acceleration_paired(&p_j, &0.0);
                output_i += computed.0;
                ddy[j] += computed.1;
            }

            ddy[i] += output_i;
        }

        Ok(())
    }
}

pub type NBodyProblem<V> = ODEProblem<f64, SecondOrderState<Vec<V>>, NewtonianGravity>;

pub trait InterpolationAlgorithm<V> {
    type Error;

    fn interpolate(&self, ts: &[f64], xs: &[V]) -> Result<Polynomial<V>, Self::Error>;
}

#[derive(Clone, Copy, Debug)]
pub struct PolyonmialInterpolator<const LEN: usize, V> {
    index: usize,
    samples: [V; LEN],
}

impl<const LEN: usize, V> PolyonmialInterpolator<LEN, V> {
    #[inline]
    pub fn new(sample: &V) -> Self
    where
        V: Copy,
    {
        Self {
            index: 1,
            samples: [*sample; LEN],
        }
    }

    #[inline]
    pub fn push(&mut self, sample: &V)
    where
        V: Copy,
    {
        assert!(self.index < LEN, "too many points for sample");
        self.samples[self.index] = *sample;
        self.index += 1;
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.index
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == LEN
    }

    #[inline]
    pub fn try_to_polynomial<A>(
        &self,
        ts: [f64; LEN],
        algorithm: &A,
    ) -> Option<Result<Polynomial<V>, A::Error>>
    where
        A: InterpolationAlgorithm<V>,
    {
        if !self.is_full() {
            return None;
        }

        Some(algorithm.interpolate(&ts, &self.samples))
    }

    #[inline]
    pub fn finish(&mut self) {
        self.samples.swap(0, self.index - 1);
        self.index = 1;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Interpolation<const LEN: usize, V, A> {
    last_sample_time: Duration,
    sample_period: Duration,
    interpolator: PolyonmialInterpolator<LEN, V>,
    algorithm: A,
}

impl<const LEN: usize, V, A> Interpolation<LEN, V, A> {
    #[inline]
    pub fn time(&self) -> Duration {
        self.last_sample_time
            + self.sample_period * (self.interpolator.len().saturating_sub(1)) as f64
    }
}

pub trait Direction {
    fn signed_delta(&self) -> Duration;
}

#[derive(Clone, Copy, Debug)]
pub struct Forward {
    delta: Duration,
}

impl Forward {
    #[inline]
    pub fn new(delta: Duration) -> Self {
        Self { delta: delta.abs() }
    }
}

impl Direction for Forward {
    #[inline]
    fn signed_delta(&self) -> Duration {
        self.delta
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Backward {
    delta: Duration,
}

impl Backward {
    #[inline]
    pub fn new(delta: Duration) -> Self {
        Self { delta: delta.abs() }
    }
}

impl Direction for Backward {
    #[inline]
    fn signed_delta(&self) -> Duration {
        -self.delta
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NBodyPropagatorError {
    Integration(StepError),
    Interpolation,
}
impl std::fmt::Display for NBodyPropagatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NBodyPropagatorError::Integration(e) => write!(f, "integration failed: {e}"),
            NBodyPropagatorError::Interpolation => {
                write!(f, "failed to interpolate trajectory")
            }
        }
    }
}
impl std::error::Error for NBodyPropagatorError {}

impl From<StepError> for NBodyPropagatorError {
    #[inline]
    fn from(err: StepError) -> Self {
        NBodyPropagatorError::Integration(err)
    }
}

pub struct NBodyPropagator<D, V, M: Method<NBodyProblem<V>>, A> {
    integration: Integration<NBodyProblem<V>, M>,
    interpolation: Vec<Interpolation<{ DIV + 1 }, V, A>>,
    _marker: std::marker::PhantomData<D>,
}

impl<D, V, M, A> Clone for NBodyPropagator<D, V, M, A>
where
    V: Clone,
    A: Clone,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState + Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            integration: self.integration.clone(),
            interpolation: self.interpolation.clone(),
            ..*self
        }
    }
}

impl<D, V, M, A> NBodyPropagator<D, V, M, A>
where
    M: Method<NBodyProblem<V>>,
{
    #[inline]
    pub fn new(
        direction: D,
        initial_time: Epoch,
        initial_state: Vec<(StateVector<V>, f64, Duration, A)>,
    ) -> Self
    where
        V: Copy,
        D: Direction,
        M: NewMethod<FixedMethodParams<f64>>,
        M::Integrator: Integrator<NBodyProblem<V>>,
    {
        let (positions, velocities, gravitational_parameters, interpolation) = initial_state
            .into_iter()
            .map(|(sv, m, sample_period, algorithm)| {
                (
                    sv.position,
                    sv.velocity,
                    m,
                    Interpolation {
                        last_sample_time: Duration::ZERO,
                        sample_period,
                        interpolator: PolyonmialInterpolator::new(&sv.position),
                        algorithm,
                    },
                )
            })
            .collect();

        Self {
            integration: M::new(FixedMethodParams::new(
                direction.signed_delta().as_seconds(),
            ))
            .integrate(NBodyProblem {
                time: initial_time.as_offset_seconds(),
                bound: f64::INFINITY,
                state: SecondOrderState::new(positions, velocities),
                ode: NewtonianGravity {
                    gravitational_parameters,
                },
            }),
            interpolation,
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        Epoch::from_offset_seconds(self.integration.problem.time)
    }

    #[inline]
    pub fn delta(&self) -> Duration
    where
        M::Integrator: IntegratorState<Time = f64>,
    {
        Duration::from_seconds(self.integration.step_size())
    }

    #[inline]
    fn step_extend<E, F>(
        &mut self,
        trajectories: &mut [UniformSpline<V>],
        try_extend: F,
    ) -> Result<(), NBodyPropagatorError>
    where
        V: Copy,
        M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
        F: Fn(&mut UniformSpline<V>, &mut Interpolation<{ DIV + 1 }, V, A>) -> Result<(), E>,
    {
        self.integration.step()?;

        let delta_abs = self.delta().abs();
        for ((position, interp), trajectory) in self
            .integration
            .problem
            .state
            .y
            .iter()
            .zip(self.interpolation.iter_mut())
            .zip(trajectories.iter_mut())
        {
            interp.last_sample_time += delta_abs;
            if interp.last_sample_time == interp.sample_period {
                interp.last_sample_time = Duration::ZERO;
                interp.interpolator.push(position);
                try_extend(trajectory, interp).map_err(|_| NBodyPropagatorError::Interpolation)?;
            }
        }

        Ok(())
    }
}

impl<D, V, M, A> Propagator for NBodyPropagator<D, V, M, A>
where
    M: Method<NBodyProblem<V>>,
{
    type Trajectory = <Self::Trajectories as Iterable>::Item;

    type Trajectories = Vec<UniformSpline<V>>;
}

impl<V, M, A> IncrementalPropagator for NBodyPropagator<Forward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
{
    type Error = NBodyPropagatorError;

    #[inline]
    fn step(&mut self, trajectories: &mut Self::Trajectories) -> Result<(), Self::Error> {
        self.step_extend::<A::Error, _>(trajectories, |trajectory, interp| {
            let ts = std::array::from_fn(|i| i as f64 / DIV as f64);
            if let Some(polynomial) = interp.interpolator.try_to_polynomial(ts, &interp.algorithm) {
                trajectory.push_back(polynomial?);
                interp.interpolator.finish();
            }
            Ok(())
        })
    }
}

impl<V, M, A> DirectionalPropagator for NBodyPropagator<Forward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState + Integrator<NBodyProblem<V>>,
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

impl<V, M, A> BranchingPropagator for NBodyPropagator<Forward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState + Integrator<NBodyProblem<V>>,
{
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        self.interpolation
            .iter()
            .map(|interpolation| {
                UniformSpline::new(
                    self.time() - interpolation.time(),
                    interpolation.sample_period * DIV as f64,
                )
            })
            .collect()
    }
}

impl<V, M, A> IncrementalPropagator for NBodyPropagator<Backward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
{
    type Error = NBodyPropagatorError;

    #[inline]
    fn step(&mut self, trajectories: &mut Self::Trajectories) -> Result<(), Self::Error> {
        self.step_extend::<A::Error, _>(trajectories, |trajectory, interp| {
            let ts = std::array::from_fn(|i| 1.0 - i as f64 / DIV as f64);
            if let Some(polynomial) = interp.interpolator.try_to_polynomial(ts, &interp.algorithm) {
                trajectory.push_front(polynomial?);
                interp.interpolator.finish();
            }
            Ok(())
        })
    }
}

impl<V, M, A> DirectionalPropagator for NBodyPropagator<Backward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState + Integrator<NBodyProblem<V>>,
{
    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        rhs.cmp(lhs)
    }

    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        to - duration
    }

    #[inline]
    fn boundary(trajectory: &Self::Trajectory) -> Epoch {
        trajectory.start()
    }
}

impl<V, M, A> BranchingPropagator for NBodyPropagator<Backward, V, M, A>
where
    V: Copy,
    A: InterpolationAlgorithm<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState + Integrator<NBodyProblem<V>>,
{
    #[inline]
    fn branch(&self) -> Self::Trajectories {
        self.interpolation
            .iter()
            .map(|interpolation| {
                UniformSpline::new(
                    self.time() + interpolation.time(),
                    interpolation.sample_period * DIV as f64,
                )
            })
            .collect()
    }
}
