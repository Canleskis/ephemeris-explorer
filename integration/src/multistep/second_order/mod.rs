mod cowell;

use cowell::{Cowell, CowellVelocityCoefficients};

use crate::{
    multistep::{LMInstance, LMState, buffer::LMBuffer},
    problem::{EvalFailed, Problem, SecondOrderODE, SecondOrderState, State},
    ratio::Ratio,
};
use num_traits::One;
use std::ops::{Add, Div, Mul, Sub};

pub trait ELM2Coefficients {
    const ALPHA: &'static [i128];

    const BETA_N: &'static [i128];

    const BETA_D: i128;
}

#[derive(Clone, Copy, Debug)]
pub struct StepOrder2<V> {
    pub state: SecondOrderState<V>,
    pub ddy: V,
}

#[derive(Clone, Debug)]
pub struct ELM2<C, const ORDER: usize, V> {
    i: u32,
    current_ddy: V,
    steps: LMBuffer<StepOrder2<V>>,
    sum1: V,
    sum2: V,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, const ORDER: usize, V> ELM2<C, ORDER, V> {
    /// Sets up the front of the buffer to be the last step
    #[inline]
    fn prepare_next_step(&mut self) {
        self.steps.rotate_right();
        std::mem::swap(&mut self.steps.front_mut().ddy, &mut self.current_ddy);
    }
}

impl<C, const ORDER: usize, V> LMState for ELM2<C, ORDER, V> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, const ORDER: usize, P, V> LMInstance<P> for ELM2<C, ORDER, V>
where
    C: ELM2Coefficients,
    V: State + Clone,
    P: Problem<Variable = V::Variable, State = SecondOrderState<V>>,
    P::Variable: Mul<P::Time, Output = P::Variable>
        + Div<P::Time, Output = P::Variable>
        + Add<Output = P::Variable>
        + Sub<Output = P::Variable>
        + Default
        + Copy,
    P::Time:
        Mul<Ratio, Output = P::Time> + Add<Output = P::Time> + Mul<Output = P::Time> + One + Copy,
    P::ODE: SecondOrderODE<P::Time, V>,
    Cowell<ORDER>: CowellVelocityCoefficients,
{
    const ORDER: usize = ORDER;

    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            current_ddy: problem.as_ref().state.dy.clone().zeroed(),
            steps: (0..ORDER - 1)
                .map(|_| StepOrder2 {
                    state: problem.as_ref().state.clone(),
                    ddy: problem.as_ref().state.dy.clone().zeroed(),
                })
                .collect(),
            sum1: problem.as_ref().state.dy.clone().zeroed(),
            sum2: problem.as_ref().state.dy.clone().zeroed(),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();
        self.sum1.zero();
        self.sum2.zero();
        for (j, (y, ddy)) in std::iter::once(&problem.state.y)
            .chain(self.steps.iter().map(|s| &s.state.y))
            .zip(std::iter::once(&self.current_ddy).chain(self.steps.iter().map(|s| &s.ddy)))
            .enumerate()
        {
            for ((sum1, sum2), (y, ddy)) in self
                .sum1
                .iter_mut()
                .zip(self.sum2.iter_mut())
                .zip(y.iter().zip(ddy.iter()))
            {
                *sum1 = *sum1 + *y * (P::Time::one() * Ratio::from_int(-C::ALPHA[j + 1]));
                *sum2 = *sum2 + *ddy * (P::Time::one() * Ratio::from_int(C::BETA_N[j + 1]));
            }
        }

        self.prepare_next_step();
        std::mem::swap(&mut self.steps.front_mut().state, &mut problem.state);

        for (y, (sum1, sum2)) in problem
            .state
            .y
            .iter_mut()
            .zip(self.sum1.iter().zip(self.sum2.iter()))
        {
            *y = *sum1 + *sum2 * (h * h * Ratio::from_recip(C::BETA_D));
        }
        problem.time = problem.time + h;
        problem
            .ode
            .eval(problem.time, &problem.state.y, self.current_ddy.zero())?;
        Cowell::update_velocity(self, problem, h);

        self.i += 1;

        Ok(())
    }

    #[inline]
    fn advance_with<E, F>(&mut self, problem: &mut P, mut f: F) -> Result<(), E>
    where
        F: FnMut(&mut P) -> Result<(), E>,
        E: From<EvalFailed>,
    {
        self.prepare_next_step();
        self.steps
            .front_mut()
            .state
            .clone_from(&problem.as_ref().state);

        f(problem)?;

        let problem = problem.as_mut();
        problem
            .ode
            .eval(problem.time, &problem.state.y, &mut self.current_ddy)?;

        Ok(())
    }
}
