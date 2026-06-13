use crate::{
    Backward, BoundedTrajectory, DirectionalPropagator, DirectionalSolout, Forward,
    IncrementalPropagator, PropagationDirection, Propagator, UniformSpline,
    trajectory::{DIV, Polynomial, StateVector},
};

use ftime::{Duration, Epoch};
use integration::{Solout, prelude::*, problem::State};
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NBodyPropagatorError {
    Integration(StepError),
    Solout,
}
impl std::fmt::Display for NBodyPropagatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NBodyPropagatorError::Integration(e) => write!(f, "integration failed: {e}"),
            NBodyPropagatorError::Solout => write!(f, "solout exit"),
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

pub struct NBodyPropagator<D, V, M: Method<NBodyProblem<V>>, O: Solout<NBodyProblem<V>>> {
    integration: Integration<NBodyProblem<V>, M, O>,
    _marker: std::marker::PhantomData<D>,
}

impl<D, V, M, O> Clone for NBodyPropagator<D, V, M, O>
where
    V: Clone,
    M: Method<NBodyProblem<V>>,
    M::Integrator: Clone,
    O: Solout<NBodyProblem<V>> + Clone,
    O::Solution: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            integration: self.integration.clone(),
            ..*self
        }
    }
}

impl<D, V, M, O> NBodyPropagator<D, V, M, O>
where
    M: Method<NBodyProblem<V>>,
    O: Solout<NBodyProblem<V>>,
{
    #[inline]
    pub fn new(
        direction: D,
        initial_time: Epoch,
        positions: Vec<V>,
        velocities: Vec<V>,
        gravitational_parameters: Vec<f64>,
        solout: O,
    ) -> Self
    where
        D: PropagationDirection,
        M: NewMethod<FixedMethodParams<f64>>,
        M::Integrator: Integrator<NBodyProblem<V>>,
    {
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
            })
            .with_solout(solout),
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn with(
        direction: D,
        initial_time: Epoch,
        initial_state: Vec<(StateVector<V>, f64)>,
        solout: O,
    ) -> Self
    where
        D: PropagationDirection,
        M: NewMethod<FixedMethodParams<f64>>,
        M::Integrator: Integrator<NBodyProblem<V>>,
    {
        let (positions, velocities, gravitational_parameters) = initial_state
            .into_iter()
            .map(|(sv, m)| (sv.position, sv.velocity, m))
            .collect();

        Self::new(
            direction,
            initial_time,
            positions,
            velocities,
            gravitational_parameters,
            solout,
        )
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
    pub fn solout(&self) -> &O {
        &self.integration.solout
    }

    #[inline]
    pub fn solution(&self) -> &O::Solution {
        &self.integration.solution
    }
}

impl<D, V, M, O> Propagator for NBodyPropagator<D, V, M, O>
where
    M: Method<NBodyProblem<V>>,
    O: DirectionalSolout<NBodyProblem<V>, Direction = D>,
{
    type Solution = O::Solution;

    #[inline]
    fn take_solution(&mut self) -> Self::Solution {
        std::mem::replace(
            &mut self.integration.solution,
            self.integration
                .solout
                .new_solution(&self.integration.problem),
        )
    }
}

impl<D, V, M, O> IncrementalPropagator for NBodyPropagator<D, V, M, O>
where
    M: Method<NBodyProblem<V>>,
    M::Integrator: Integrator<NBodyProblem<V>>,
    O: DirectionalSolout<NBodyProblem<V>, Direction = D>,
{
    type Error = NBodyPropagatorError;

    #[inline]
    fn step(&mut self) -> Result<(), Self::Error> {
        if !self.integration.advance()? {
            return Err(NBodyPropagatorError::Solout);
        }

        Ok(())
    }
}

impl<D, V, M, O> DirectionalPropagator for NBodyPropagator<D, V, M, O>
where
    D: PropagationDirection,
    M: Method<NBodyProblem<V>>,
    O: DirectionalSolout<NBodyProblem<V>, Direction = D>,
{
    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        D::offset(to, duration)
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        D::distance(from, to)
    }

    #[inline]
    fn time(&self) -> Epoch {
        O::solution_time(&self.integration.solution)
    }

    #[inline]
    fn has_reached(&self, time: Epoch) -> bool {
        O::has_reached(&self.integration.solution, time)
    }
}

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
pub struct SplineInterpolator<V, A> {
    pub last_sample_time: Duration,
    pub sample_period: Duration,
    pub interpolator: PolyonmialInterpolator<{ DIV + 1 }, V>,
    pub algorithm: A,
}

impl<V, A> SplineInterpolator<V, A> {
    #[inline]
    pub fn time(&self) -> Duration {
        self.last_sample_time
            + self.sample_period * (self.interpolator.len().saturating_sub(1)) as f64
    }
}

#[derive(Clone, Debug)]
pub struct SplineInterpolators<D, V, A> {
    delta: Duration,
    interpolators: Vec<SplineInterpolator<V, A>>,
    _marker: std::marker::PhantomData<D>,
}

impl<D, V, A> SplineInterpolators<D, V, A> {
    #[inline]
    pub fn new(delta: Duration, interpolators: Vec<SplineInterpolator<V, A>>) -> Self {
        Self {
            delta,
            interpolators,
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn with(
        delta: Duration,
        initial_positions: Vec<V>,
        sample_periods: Vec<Duration>,
        algorithms: Vec<A>,
    ) -> Self
    where
        V: Copy,
    {
        Self {
            delta,
            interpolators: initial_positions
                .into_iter()
                .zip(sample_periods)
                .zip(algorithms)
                .map(
                    |((position, sample_period), algorithm)| SplineInterpolator {
                        last_sample_time: Duration::ZERO,
                        sample_period,
                        interpolator: PolyonmialInterpolator::new(&position),
                        algorithm,
                    },
                )
                .collect(),
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    fn solout_with<F>(
        &mut self,
        problem: &NBodyProblem<V>,
        solution: &mut [UniformSpline<V>],
        try_extend: F,
    ) -> bool
    where
        V: Copy,
        F: Fn(&mut UniformSpline<V>, &mut SplineInterpolator<V, A>) -> bool,
    {
        for ((position, interp), trajectory) in problem
            .state
            .y
            .iter()
            .zip(self.interpolators.iter_mut())
            .zip(solution.iter_mut())
        {
            interp.last_sample_time += self.delta;
            if interp.last_sample_time == interp.sample_period {
                interp.last_sample_time = Duration::ZERO;
                interp.interpolator.push(position);
                if !try_extend(trajectory, interp) {
                    return false;
                }
            }
        }

        true
    }
}

pub trait SplineBound<D, V> {
    fn bound(&self) -> Epoch;

    fn push_at_bound(&mut self, polynomial: Polynomial<V>);

    fn samples() -> [f64; 9];
}

impl<V> SplineBound<Forward, V> for UniformSpline<V> {
    #[inline]
    fn bound(&self) -> Epoch {
        self.end()
    }

    #[inline]
    fn push_at_bound(&mut self, polynomial: Polynomial<V>) {
        self.push_back(polynomial);
    }

    #[inline]
    fn samples() -> [f64; 9] {
        std::array::from_fn(|i| i as f64 / DIV as f64)
    }
}

impl<V> SplineBound<Backward, V> for UniformSpline<V> {
    #[inline]
    fn bound(&self) -> Epoch {
        self.start()
    }

    #[inline]
    fn push_at_bound(&mut self, polynomial: Polynomial<V>) {
        self.push_front(polynomial);
    }

    #[inline]
    fn samples() -> [f64; 9] {
        std::array::from_fn(|i| 1.0 - i as f64 / DIV as f64)
    }
}

impl<D, V, A> Solout<NBodyProblem<V>> for SplineInterpolators<D, V, A>
where
    D: PropagationDirection,
    V: Copy,
    A: InterpolationAlgorithm<V>,
    UniformSpline<V>: SplineBound<D, V>,
{
    type Solution = Vec<UniformSpline<V>>;

    #[inline]
    fn new_solution(&self, problem: &NBodyProblem<V>) -> Self::Solution {
        self.interpolators
            .iter()
            .map(|interpolator| {
                UniformSpline::new(
                    D::offset(
                        Epoch::from_offset_seconds(problem.time),
                        -interpolator.time(),
                    ),
                    interpolator.sample_period * DIV as f64,
                )
            })
            .collect()
    }

    #[inline]
    fn solout(&mut self, problem: &NBodyProblem<V>, solution: &mut Self::Solution) -> bool {
        self.solout_with(problem, solution, |trajectory, interp| {
            let ts = UniformSpline::<V>::samples();
            match interp
                .interpolator
                .try_to_polynomial(ts, &interp.algorithm)
                .transpose()
            {
                Ok(polynomial) => {
                    if let Some(polynomial) = polynomial {
                        trajectory.push_at_bound(polynomial);
                        interp.interpolator.finish();
                    }
                    true
                }
                Err(_) => false,
            }
        })
    }
}

impl<D, V, A> DirectionalSolout<NBodyProblem<V>> for SplineInterpolators<D, V, A>
where
    D: PropagationDirection,
    V: Copy,
    A: InterpolationAlgorithm<V>,
    UniformSpline<V>: SplineBound<D, V>,
{
    type Direction = D;

    #[inline]
    fn solution_time(solution: &Self::Solution) -> Epoch {
        solution
            .iter()
            .map(|traj| traj.bound())
            .min_by(D::cmp)
            .unwrap_or_else(|| D::offset(Epoch::default(), -Duration::MAX))
    }

    #[inline]
    fn has_reached(solution: &Self::Solution, time: Epoch) -> bool {
        solution
            .iter()
            .map(|traj| traj.bound())
            .all(|bound| D::cmp(&bound, &time).is_ge())
    }
}
