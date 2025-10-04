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

pub trait Interpolation<V> {
    fn interpolate(&self, ts: &[f64], xs: &[V]) -> Option<Polynomial<V>>;
}

#[derive(Clone, Copy, Debug)]
pub struct SegmentSamples<const LEN: usize, V> {
    index: usize,
    samples: [V; LEN],
}

impl<const LEN: usize, V> SegmentSamples<LEN, V> {
    #[inline]
    pub fn with(sample: &V) -> Self
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
        assert!(self.index < LEN, "too many points for segment");
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
    fn try_to_polynomial<A>(&self, ts: [f64; LEN], algorithm: &A) -> Option<Polynomial<V>>
    where
        A: Interpolation<V>,
    {
        algorithm.interpolate(&ts, &self.samples)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct InterpolationSamples<const LEN: usize, V, A> {
    last_sample_time: Duration,
    sample_period: Duration,
    samples: SegmentSamples<LEN, V>,
    algorithm: A,
}

impl<const LEN: usize, V, A> InterpolationSamples<LEN, V, A> {
    #[inline]
    pub fn time(&self) -> Duration {
        self.last_sample_time
            + self
                .sample_period
                .scaled((self.samples.len().saturating_sub(1)) as f64)
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
pub enum NBodyPropagationError {
    IntegrationError(StepError),
    InterpolationError,
}
impl std::fmt::Display for NBodyPropagationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NBodyPropagationError::IntegrationError(e) => write!(f, "integration failed: {e}"),
            NBodyPropagationError::InterpolationError => {
                write!(f, "failed to interpolate trajectory")
            }
        }
    }
}
impl std::error::Error for NBodyPropagationError {}

impl From<StepError> for NBodyPropagationError {
    #[inline]
    fn from(err: StepError) -> Self {
        NBodyPropagationError::IntegrationError(err)
    }
}

pub struct NBodyPropagator<D, V, M: Method<NBodyProblem<V>>, A> {
    integration: Integration<NBodyProblem<V>, M>,
    interpolation: Vec<InterpolationSamples<{ DIV + 1 }, V, A>>,
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
                    InterpolationSamples {
                        last_sample_time: Duration::ZERO,
                        sample_period,
                        samples: SegmentSamples::with(&sv.position),
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
        Epoch::from_offset(Duration::from_seconds(self.integration.problem.time))
    }

    #[inline]
    pub fn delta(&self) -> Duration
    where
        M::Integrator: IntegratorState<Time = f64>,
    {
        Duration::from_seconds(self.integration.step_size())
    }

    #[inline]
    fn extend_with<F>(
        &mut self,
        trajectories: &mut [UniformSpline<V>],
        add: F,
    ) -> Result<(), NBodyPropagationError>
    where
        V: Copy,
        M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
        F: Fn(&mut UniformSpline<V>, &InterpolationSamples<{ DIV + 1 }, V, A>) -> Result<(), ()>,
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
                interp.samples.push(position);
                if interp.samples.is_full() {
                    add(trajectory, interp)
                        .map_err(|_| NBodyPropagationError::InterpolationError)?;
                    interp.samples.index = 0;
                    interp.samples.push(position);
                }
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
    A: Interpolation<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
{
    type Error = NBodyPropagationError;

    #[inline]
    fn step(&mut self, trajectories: &mut Self::Trajectories) -> Result<(), Self::Error> {
        self.extend_with(trajectories, |trajectory, interp| {
            let polynomial = interp.samples.try_to_polynomial(
                std::array::from_fn(|i| i as f64 / DIV as f64),
                &interp.algorithm,
            );
            trajectory.push_back(polynomial.ok_or(())?);
            Ok(())
        })
    }
}

impl<V, M, A> DirectionalPropagator for NBodyPropagator<Forward, V, M, A>
where
    V: Copy,
    A: Interpolation<V>,
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
    A: Interpolation<V>,
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
                    interpolation.sample_period.scaled(DIV as f64),
                )
            })
            .collect()
    }

    #[inline]
    fn merge(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_after(rhs.start());
        lhs.append(rhs);
    }
}

impl<V, M, A> IncrementalPropagator for NBodyPropagator<Backward, V, M, A>
where
    V: Copy,
    A: Interpolation<V>,
    M: Method<NBodyProblem<V>>,
    M::Integrator: IntegratorState<Time = f64> + Integrator<NBodyProblem<V>>,
{
    type Error = NBodyPropagationError;

    #[inline]
    fn step(&mut self, trajectories: &mut Self::Trajectories) -> Result<(), Self::Error> {
        self.extend_with(trajectories, |trajectory, interp| {
            let segment = interp.samples.try_to_polynomial(
                std::array::from_fn(|i| 1.0 - i as f64 / DIV as f64),
                &interp.algorithm,
            );
            trajectory.push_front(segment.ok_or(())?);
            Ok(())
        })
    }
}

impl<V, M, A> DirectionalPropagator for NBodyPropagator<Backward, V, M, A>
where
    V: Copy,
    A: Interpolation<V>,
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
    A: Interpolation<V>,
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
                    interpolation.sample_period.scaled(DIV as f64),
                )
            })
            .collect()
    }

    #[inline]
    fn merge(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_before(rhs.end());
        lhs.prepend(rhs);
    }
}
