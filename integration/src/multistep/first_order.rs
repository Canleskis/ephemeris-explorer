use crate::{
    multistep::{LMInstance, LMState, buffer::LMBuffer},
    problem::{EvalFailed, FirstOrderODE, Problem, State},
    ratio::Ratio,
};
use num_traits::One;
use std::ops::{Add, Mul};

pub trait ELM1Coefficients {
    const ALPHA: &'static [i128];

    const BETA_N: &'static [i128];

    const BETA_D: i128;
}

#[derive(Clone, Copy, Debug)]
pub struct StepOrder1<V> {
    pub y: V,
    pub dy: V,
}

#[derive(Clone, Debug)]
pub struct ELM1<C, const ORDER: usize, V> {
    i: u32,
    current_dy: V,
    steps: LMBuffer<StepOrder1<V>>,
    sum1: V,
    sum2: V,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, const ORDER: usize, V> ELM1<C, ORDER, V> {
    /// Sets up the front of the buffer to be the last step
    #[inline]
    fn prepare_next_step(&mut self) {
        self.steps.rotate_right();
        std::mem::swap(&mut self.steps.front_mut().dy, &mut self.current_dy);
    }
}

impl<C, const ORDER: usize, V> LMState for ELM1<C, ORDER, V> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, const ORDER: usize, V, P> LMInstance<P> for ELM1<C, ORDER, P::State>
where
    C: ELM1Coefficients,
    V: State + Clone,
    P: Problem<Variable = V::Variable, State = V>,
    P::Variable: Mul<P::Time, Output = P::Variable> + Add<Output = P::Variable> + Default + Copy,
    P::Time: Mul<Ratio, Output = P::Time> + Add<Output = P::Time> + One + Copy,
    P::ODE: FirstOrderODE<P::Time, V>,
{
    const ORDER: usize = ORDER;

    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            current_dy: problem.as_ref().state.clone().zeroed(),
            steps: (0..ORDER - 1)
                .map(|_| StepOrder1 {
                    y: problem.as_ref().state.clone(),
                    dy: problem.as_ref().state.clone().zeroed(),
                })
                .collect(),
            sum1: problem.as_ref().state.clone().zeroed(),
            sum2: problem.as_ref().state.clone().zeroed(),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();
        self.sum1.zero();
        self.sum2.zero();
        for (j, (y, dy)) in std::iter::once(&problem.state)
            .chain(self.steps.iter().map(|s| &s.y))
            .zip(std::iter::once(&self.current_dy).chain(self.steps.iter().map(|s| &s.dy)))
            .enumerate()
        {
            for ((sum1, sum2), (y, dy)) in self
                .sum1
                .iter_mut()
                .zip(self.sum2.iter_mut())
                .zip(y.iter().zip(dy.iter()))
            {
                *sum1 = *sum1 + *y * (P::Time::one() * Ratio::from_int(-C::ALPHA[j + 1]));
                *sum2 = *sum2 + *dy * (P::Time::one() * Ratio::from_int(C::BETA_N[j + 1]));
            }
        }

        self.prepare_next_step();
        // We no longer need the previous state to compute the next one.
        std::mem::swap(&mut self.steps.front_mut().y, &mut problem.state);

        for (y, (sum1, sum2)) in problem
            .state
            .iter_mut()
            .zip(self.sum1.iter().zip(self.sum2.iter()))
        {
            *y = *sum1 + *sum2 * (h * Ratio::from_recip(C::BETA_D));
        }
        problem.time = problem.time + h;
        problem
            .ode
            .eval(problem.time, &problem.state, self.current_dy.zero())?;

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
        // We do not know if `f` needs the previous state to compute the next one, so we clone.
        self.steps.front_mut().y.clone_from(&problem.as_ref().state);

        f(problem)?;

        let problem = problem.as_mut();
        problem
            .ode
            .eval(problem.time, &problem.state, &mut self.current_dy)?;

        Ok(())
    }
}
