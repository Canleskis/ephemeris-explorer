use crate::{
    Backward, BoundedTrajectory, DirectionalPropagator, DirectionalSolout, Forward,
    IncrementalPropagator, PropagationDirection, Propagator,
};

use ftime::{Duration, Epoch};
use integration::{Solout, prelude::*, problem::State};

pub trait GravitationalForce<V> {
    fn accelerations_into(&mut self, time: Epoch, positions: &[V], accelerations: &mut [V]);
}

#[derive(Clone, Copy)]
pub struct GravityModel<F> {
    pub force_model: F,
}

impl<F> GravityModel<F> {
    #[inline]
    pub fn new(force_model: F) -> Self {
        Self { force_model }
    }

    #[inline]
    pub fn accelerations_into<V>(&mut self, time: Epoch, positions: &[V], accelerations: &mut [V])
    where
        F: GravitationalForce<V>,
    {
        self.force_model
            .accelerations_into(time, positions, accelerations);
    }
}

impl<V, F> SecondOrderODE<f64, Vec<V>> for GravityModel<F>
where
    F: GravitationalForce<V>,
{
    #[inline]
    fn eval(&mut self, t: f64, y: &Vec<V>, ddy: &mut Vec<V>) -> Result<(), EvalFailed> {
        self.accelerations_into(Epoch::from_offset_seconds(t), y, ddy);

        Ok(())
    }
}

pub type NBodyProblem<V, F> = ODEProblem<f64, SecondOrderState<Vec<V>>, GravityModel<F>>;

pub trait AccelerationPairedAssign {
    type Output;

    fn acceleration_paired_assign(
        &self,
        other: &Self,
        a1: &mut Self::Output,
        a2: &mut Self::Output,
    );
}

pub trait GravitySource: AccelerationPairedAssign {
    type Vector;

    type Scalar;

    fn init(position: Self::Vector, mu: Self::Scalar) -> Self;
}

pub struct NewtonianGravity<P: GravitySource> {
    pub gravitational_parameters: Vec<P::Scalar>,
}

impl<P: GravitySource> Clone for NewtonianGravity<P>
where
    P::Scalar: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            gravitational_parameters: self.gravitational_parameters.clone(),
        }
    }
}

impl<P: GravitySource> NewtonianGravity<P> {
    #[inline]
    pub fn new(gravitational_parameters: Vec<P::Scalar>) -> Self {
        Self {
            gravitational_parameters,
        }
    }
}

impl<V, S, P> GravitationalForce<V> for NewtonianGravity<P>
where
    P: GravitySource<Vector = V, Scalar = S, Output = V>,
    V: Copy,
    S: Copy,
{
    #[inline]
    fn accelerations_into(&mut self, _: Epoch, positions: &[V], accelerations: &mut [V]) {
        for i in 0..positions.len() {
            let pi = P::init(positions[i], self.gravitational_parameters[i]);

            #[expect(clippy::needless_range_loop)]
            for j in (i + 1)..positions.len() {
                let pj = P::init(positions[j], self.gravitational_parameters[j]);
                let (accelerations_l, accelerations_r) = accelerations.split_at_mut(j);
                pi.acceleration_paired_assign(
                    &pj,
                    &mut accelerations_l[i],
                    &mut accelerations_r[0],
                );
            }
        }
    }
}

pub struct CachedNewtonianGravity<P: GravitySource> {
    pub newtonian_gravity: NewtonianGravity<P>,
    pub time: Epoch,
    pub buffer: Vec<P::Output>,
}

impl<P: GravitySource> Clone for CachedNewtonianGravity<P>
where
    P::Output: Clone,
    P::Scalar: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            newtonian_gravity: self.newtonian_gravity.clone(),
            time: self.time,
            buffer: self.buffer.clone(),
        }
    }
}

impl<P: GravitySource> CachedNewtonianGravity<P> {
    #[inline]
    pub fn new(mus: Vec<P::Scalar>) -> Self
    where
        P::Output: Default,
    {
        Self {
            buffer: mus.iter().map(|_| Default::default()).collect(),
            newtonian_gravity: NewtonianGravity::new(mus),
            time: Epoch::NAN,
        }
    }

    #[inline]
    fn accelerations_into_inner(&mut self, time: Epoch, positions: &[P::Output])
    where
        P::Output: Default,
        NewtonianGravity<P>: GravitationalForce<P::Output>,
    {
        self.newtonian_gravity
            .accelerations_into(time, positions, self.buffer.zero());
    }
}

impl<V, S, P> GravitationalForce<V> for CachedNewtonianGravity<P>
where
    P: GravitySource<Vector = V, Scalar = S, Output = V>,
    V: Default + Copy,
    S: Copy,
{
    #[inline]
    fn accelerations_into(&mut self, time: Epoch, positions: &[V], accelerations: &mut [V]) {
        if time != self.time {
            self.accelerations_into_inner(time, positions);
            self.time = time;
        }

        accelerations.copy_from_slice(&self.buffer);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NBodyPropagatorError {
    Integration(StepError),
    Solout,
}
impl std::fmt::Display for NBodyPropagatorError {
    #[inline]
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

pub struct NBodyPropagator<V, F, M: Method<NBodyProblem<V, F>>, O: Solout<NBodyProblem<V, F>>> {
    integration: Integration<NBodyProblem<V, F>, M, O>,
}

impl<V, F, M, O> Clone for NBodyPropagator<V, F, M, O>
where
    V: Clone,
    F: Clone,
    M: Method<NBodyProblem<V, F>>,
    M::Integrator: Clone,
    O: Solout<NBodyProblem<V, F>> + Clone,
    O::Solution: Clone,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            integration: self.integration.clone(),
        }
    }
}

impl<V, F, M> NBodyPropagator<V, F, M, ()>
where
    M: Method<NBodyProblem<V, F>>,
{
    #[inline]
    pub fn new<D>(h: Duration, problem: NBodyProblem<V, F>) -> Self
    where
        D: PropagationDirection,
        M: NewMethod<FixedMethodParams<f64>>,
        M::Integrator: Integrator<NBodyProblem<V, F>>,
    {
        Self {
            integration: M::new(FixedMethodParams::new(h.as_seconds())).integrate(problem),
        }
    }

    #[inline]
    pub fn with_solout<O>(self, solout: O) -> NBodyPropagator<V, F, M, O>
    where
        O: Solout<NBodyProblem<V, F>>,
    {
        NBodyPropagator {
            integration: self.integration.with_solout(solout),
        }
    }
}

impl<V, F, M, O> NBodyPropagator<V, F, M, O>
where
    M: Method<NBodyProblem<V, F>>,
    O: Solout<NBodyProblem<V, F>>,
{
    #[inline]
    pub fn new_solout<D>(h: Duration, problem: NBodyProblem<V, F>, solout: O) -> Self
    where
        D: PropagationDirection,
        M: NewMethod<FixedMethodParams<f64>>,
        M::Integrator: Integrator<NBodyProblem<V, F>>,
    {
        NBodyPropagator::new::<D>(h, problem).with_solout(solout)
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
    pub fn problem(&self) -> &NBodyProblem<V, F> {
        &self.integration.problem
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

impl<V, F, M, O> Propagator for NBodyPropagator<V, F, M, O>
where
    M: Method<NBodyProblem<V, F>>,
    O: DirectionalSolout<NBodyProblem<V, F>>,
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

impl<V, F, M, O> IncrementalPropagator for NBodyPropagator<V, F, M, O>
where
    M: Method<NBodyProblem<V, F>>,
    M::Integrator: Integrator<NBodyProblem<V, F>>,
    O: DirectionalSolout<NBodyProblem<V, F>>,
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

impl<V, F, M, O> DirectionalPropagator for NBodyPropagator<V, F, M, O>
where
    M: Method<NBodyProblem<V, F>>,
    O: DirectionalSolout<NBodyProblem<V, F>>,
    O::Direction: PropagationDirection,
{
    #[inline]
    fn offset(to: Epoch, duration: Duration) -> Epoch {
        O::Direction::offset(to, duration)
    }

    #[inline]
    fn distance(from: Epoch, to: Epoch) -> Duration {
        O::Direction::distance(from, to)
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

pub trait BoundedSpline<D> {
    fn bound(&self) -> Epoch;
}

impl<T: BoundedTrajectory> BoundedSpline<Forward> for T {
    #[inline]
    fn bound(&self) -> Epoch {
        self.end()
    }
}

impl<T: BoundedTrajectory> BoundedSpline<Backward> for T {
    #[inline]
    fn bound(&self) -> Epoch {
        self.start()
    }
}

pub trait ExtendSpline<D, T> {
    fn push_at_bound(&mut self, item: T);
}

pub trait InterpolationTolerance<S> {
    fn is_valid(&mut self, samples: &[S]) -> bool;
}

pub trait InterpolationSample<V> {
    type Trajectory;

    fn new(time: Epoch, position: V, velocity: V, acceleration: V) -> Self;
}

#[derive(Clone, Debug)]
pub struct SplineInterpolator<D, S, T> {
    pub tolerance: T,
    pub samples: Vec<S>,
    pub _marker: std::marker::PhantomData<D>,
}

impl<D, S, T> SplineInterpolator<D, S, T> {
    #[inline]
    pub fn build<V>(&mut self, sample: S, trajectory: &mut S::Trajectory)
    where
        S: InterpolationSample<V> + Clone,
        S::Trajectory: ExtendSpline<D, S>,
        T: InterpolationTolerance<S>,
    {
        self.samples.push(sample);

        if !self.tolerance.is_valid(&self.samples) {
            let n = self.samples.len().saturating_sub(2).max(1);
            self.samples.drain(0..n);
            let sample = &self.samples[0];
            trajectory.push_at_bound(sample.clone());
        }
    }

    #[inline]
    pub fn first_sample(&self) -> &S {
        &self.samples[0]
    }

    #[inline]
    pub fn last_sample(&self) -> &S {
        &self.samples[self.samples.len() - 1]
    }
}

#[derive(Clone)]
pub struct SplineInterpolators<D, S, V, T> {
    pub interpolators: Vec<SplineInterpolator<D, S, T>>,
    pub accelerations_buffer: Vec<V>,
}

impl<D, S, V, T> SplineInterpolators<D, S, V, T> {
    #[inline]
    pub fn new<F>(problem: &mut NBodyProblem<V, F>, tolerance: T) -> Self
    where
        S: InterpolationSample<V>,
        V: Default + Copy,
        T: Clone,
        F: GravitationalForce<V>,
    {
        let time = Epoch::from_offset_seconds(problem.time);
        let mut accelerations_buffer = problem.state.y.to_vec().zeroed();
        problem
            .ode
            .accelerations_into(time, &problem.state.y, &mut accelerations_buffer);
        Self {
            interpolators: problem
                .state
                .y
                .iter()
                .zip(problem.state.dy.iter())
                .zip(accelerations_buffer.iter())
                .map(|((position, velocity), acceleration)| SplineInterpolator {
                    tolerance: tolerance.clone(),
                    samples: vec![S::new(time, *position, *velocity, *acceleration)],
                    _marker: std::marker::PhantomData,
                })
                .collect(),
            accelerations_buffer,
        }
    }
}

impl<D, S, V, T, F> Solout<NBodyProblem<V, F>> for SplineInterpolators<D, S, V, T>
where
    S: InterpolationSample<V> + Clone,
    S::Trajectory: ExtendSpline<D, S> + Default,
    V: Default + Copy,
    T: InterpolationTolerance<S>,
    F: GravitationalForce<V>,
{
    type Solution = Vec<S::Trajectory>;

    #[inline]
    fn new_solution(&self, _: &NBodyProblem<V, F>) -> Self::Solution {
        self.interpolators
            .iter()
            .map(|interpolator| {
                let mut trajectory = S::Trajectory::default();
                trajectory.push_at_bound(interpolator.first_sample().clone());
                trajectory
            })
            .collect()
    }

    #[inline]
    fn solout(&mut self, problem: &mut NBodyProblem<V, F>, trajs: &mut Self::Solution) -> bool {
        let time = Epoch::from_offset_seconds(problem.time);
        problem
            .ode
            .accelerations_into(time, &problem.state.y, self.accelerations_buffer.zero());
        for ((((position, velocity), acceleration), trajectory), interpolator) in problem
            .state
            .y
            .iter()
            .zip(problem.state.dy.iter())
            .zip(self.accelerations_buffer.iter())
            .zip(trajs.iter_mut())
            .zip(self.interpolators.iter_mut())
        {
            interpolator.build(
                S::new(time, *position, *velocity, *acceleration),
                trajectory,
            );
        }

        true
    }
}

impl<D, S, V, T, F> DirectionalSolout<NBodyProblem<V, F>> for SplineInterpolators<D, S, V, T>
where
    D: PropagationDirection,
    S: InterpolationSample<V> + Clone,
    S::Trajectory: ExtendSpline<D, S> + BoundedSpline<D> + Default,
    V: Default + Copy,
    T: InterpolationTolerance<S>,
    F: GravitationalForce<V>,
{
    type Direction = D;

    #[inline]
    fn solution_time(trajs: &Self::Solution) -> Epoch {
        trajs
            .iter()
            .map(|traj| traj.bound())
            .min_by(D::cmp)
            .unwrap_or_else(|| D::offset(Epoch::default(), -Duration::MAX))
    }

    #[inline]
    fn has_reached(trajs: &Self::Solution, time: Epoch) -> bool {
        trajs
            .iter()
            .map(|traj| traj.bound())
            .all(|bound| D::cmp(&bound, &time).is_ge())
    }
}
