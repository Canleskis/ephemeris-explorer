use crate::{
    problem::{EvalFailed, Problem, SecondOrderODE, SecondOrderState, State},
    ratio::Ratio,
    runge_kutta::{RKEmbedded, RKInstance, RKState},
};
use std::ops::{Add, Mul};

/// Coefficients for Explicit Runge-Kutta-Nyström (ERKN) integrators, more commonly known as the
/// Butcher tableau.
///
/// ERKN methods are integrators for second-order ordinary differential equations of the form:  
/// `y'' = f(t, y)`
#[doc(alias = "Butcher Tableau")]
pub trait ERKNCoefficients {
    const FSAL: bool;

    const ORDER: u16;

    const A: &'static [&'static [Ratio]];

    const BP: &'static [Ratio];

    const BV: &'static [Ratio];

    const C: &'static [Ratio];
}

pub trait EERKNCoefficients: ERKNCoefficients {
    const ORDER_EMBEDDED: u16;

    const EP: &'static [Ratio];

    const EV: &'static [Ratio];
}

#[derive(Clone, Copy, Debug)]
pub struct ERKN<C, K, V> {
    pub i: u32,
    pub dk: K,
    yi: V,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, K, V> RKState for ERKN<C, K, V> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, const STAGES: usize, V, P> RKInstance<P> for ERKN<C, [V; STAGES], V>
where
    C: ERKNCoefficients,
    V: State + Clone,
    P: Problem<Variable = V::Variable, State = SecondOrderState<V>>,
    P::Variable: Mul<P::Time, Output = P::Variable> + Add<Output = P::Variable> + Default + Copy,
    P::Time: Mul<Ratio, Output = P::Time> + Add<Output = P::Time> + Mul<Output = P::Time> + Copy,
    P::ODE: SecondOrderODE<P::Time, V>,
{
    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            dk: std::array::from_fn(|_| problem.as_ref().state.dy.clone()),
            yi: problem.as_ref().state.y.clone(),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();

        // let mut yi = self.y.clone();
        for s in 0..STAGES {
            if C::FSAL && s == 0 && self.i > 0 {
                self.dk.swap(s, STAGES - 1);
                continue;
            }

            // tᵢ = t₀ + hcᵢ
            let ti = problem.time + h * C::C[s];
            // yᵢ₊₁ = y₀ + hcᵢy₀' + h²∑ⱼāᵢⱼkⱼ'
            self.yi.clone_from(&problem.state.y);
            for (yi, dy) in self.yi.iter_mut().zip(problem.state.dy.iter()) {
                *yi = *yi + *dy * (h * C::C[s]);
            }
            for j in 0..s {
                for (yi, kp) in self.yi.iter_mut().zip(self.dk[j].iter()) {
                    *yi = *yi + *kp * (h * h * C::A[s][j]);
                }
            }
            // kᵢ' = f(tᵢ, yᵢ)
            problem.ode.eval(ti, &self.yi, self.dk[s].zero())?; // TODO: Set kᵢ to zeroes.
        }

        // yₙ₊₁ = yₙ + hyₙ' + h²∑ᵢb̄ᵢkᵢ'
        // yₙ₊₁' = yₙ' + h∑ᵢbᵢkᵢ'
        for (y, dy) in problem.state.y.iter_mut().zip(problem.state.dy.iter()) {
            *y = *y + *dy * h;
        }
        for i in 0..STAGES {
            for ((y, dy), kp) in problem
                .state
                .y
                .iter_mut()
                .zip(problem.state.dy.iter_mut())
                .zip(self.dk[i].iter())
            {
                *y = *y + *kp * (h * h * C::BP[i]);
                *dy = *dy + *kp * (h * C::BV[i]);
            }
        }

        problem.time = problem.time + h;
        self.i += 1;

        Ok(())
    }
}

impl<C, const STAGES: usize, T, V> RKEmbedded<T> for ERKN<C, [V; STAGES], V>
where
    C: EERKNCoefficients,
    V: State + Clone,
    V::Variable: Mul<T, Output = V::Variable> + Add<Output = V::Variable> + Default + Copy,
    T: Mul<Ratio, Output = T> + Mul<Output = T> + Copy,
{
    type State = SecondOrderState<V>;

    const LOWER_ORDER: u16 = match C::ORDER < C::ORDER_EMBEDDED {
        true => C::ORDER,
        false => C::ORDER_EMBEDDED,
    };

    #[inline]
    fn error(&self, h: T, error: &mut SecondOrderState<V>) {
        error.y.zero();
        error.dy.zero();
        // eₙ₊₁ = h²∑ᵢ(b̄ᵢ - b̄ᵢ*)kᵢ'
        // eₙ₊₁' = h∑ᵢ(bᵢ - bᵢ*)kᵢ'
        for i in 0..STAGES {
            for ((ey, edy), kp) in error
                .y
                .iter_mut()
                .zip(error.dy.iter_mut())
                .zip(self.dk[i].iter())
            {
                *ey = *ey + *kp * (h * h * C::EP[i]);
                *edy = *edy + *kp * (h * C::EV[i]);
            }
        }
    }

    #[inline]
    fn undo_step(&mut self, prev: &Self) {
        self.i = prev.i;
        if C::FSAL {
            self.dk[STAGES - 1].clone_from(&prev.dk[STAGES - 1]);
        }
    }
}
