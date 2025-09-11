use crate::{
    problem::{EvalFailed, FirstOrderODE, Problem, State},
    ratio::Ratio,
    runge_kutta::{RKEmbedded, RKInstance, RKState},
};
use std::ops::{Add, Mul};

/// Coefficients for Explicit Runge-Kutta (ERK) integrators, more commonly known as the Butcher
/// tableau.
///
/// ERK methods are integrators for first-order ordinary differential equations of the form:  
/// `y' = f(t, y)`
#[doc(alias = "Butcher Tableau")]
pub trait ERKCoefficients {
    const FSAL: bool;

    const ORDER: u16;

    const A: &'static [&'static [Ratio]];

    const B: &'static [Ratio];

    const C: &'static [Ratio];
}

/// Coefficients for Embedded Explicit Runge-Kutta integrators (EERK), defined by error
/// coefficients as well as the Butcher tableau.
///
/// These coefficients are determined by an _embedded_ Runge-Kutta method that is used to estimate
/// the local error of the solution, which is then used to adapt the step size.
pub trait EERKCoefficients: ERKCoefficients {
    const ORDER_EMBEDDED: u16;

    /// Error coefficients, defined as `eᵢ = bᵢ - bᵢ*`.
    const E: &'static [Ratio];
}

#[derive(Clone, Copy)]
pub struct ERK<C, K> {
    pub i: u32,
    pub k: K,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, K> RKState for ERK<C, K> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, const STAGES: usize, V, P> RKInstance<P> for ERK<C, [P::State; STAGES]>
where
    C: ERKCoefficients,
    V: State + Clone,
    P: Problem<Variable = V::Variable, State = V>,
    P::Variable: Mul<P::Time, Output = P::Variable> + Add<Output = P::Variable> + Default + Copy,
    P::Time: Mul<Ratio, Output = P::Time> + Add<Output = P::Time> + Copy,
    P::ODE: FirstOrderODE<P::Time, V>,
{
    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            k: std::array::from_fn(|_| problem.as_ref().state.clone()),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();
        let mut yi = problem.state.clone();
        for s in 0..STAGES {
            if C::FSAL && s == 0 && self.i > 0 {
                self.k.swap(s, STAGES - 1);
                continue;
            }

            // tᵢ = t₀ + hcᵢ
            let ti = problem.time + h * C::C[s];
            // yᵢ₊₁ = y₀ + h∑ⱼaᵢⱼkⱼ
            yi.clone_from(&problem.state);
            for j in 0..s {
                for (yi, k) in yi.iter_mut().zip(self.k[j].iter()) {
                    *yi = *yi + *k * (h * C::A[s][j]);
                }
            }
            // kᵢ = f(tᵢ, yᵢ)
            problem.ode.eval(ti, &yi, self.k[s].zero())?;
        }

        // yₙ₊₁ = yₙ + h∑ᵢbᵢkᵢ
        for i in 0..STAGES {
            for (y, k) in problem.state.iter_mut().zip(self.k[i].iter()) {
                *y = *y + *k * (h * C::B[i]);
            }
        }

        problem.time = problem.time + h;
        self.i += 1;

        Ok(())
    }
}

impl<C, const STAGES: usize, T, V> RKEmbedded<T> for ERK<C, [V; STAGES]>
where
    C: EERKCoefficients,
    V: State + Clone,
    V::Variable: Mul<T, Output = V::Variable> + Add<Output = V::Variable> + Default + Copy,
    T: Mul<Ratio, Output = T> + Copy,
{
    type State = V;

    const LOWER_ORDER: u16 = match C::ORDER < C::ORDER_EMBEDDED {
        true => C::ORDER,
        false => C::ORDER_EMBEDDED,
    };

    #[inline]
    fn error(&self, h: T, error: &mut Self::State) {
        error.zero();
        // eₙ₊₁ = h∑ᵢeᵢkᵢ = h∑ᵢ(bᵢ - bᵢ*)kᵢ
        for i in 0..STAGES {
            for (e, k) in error.iter_mut().zip(self.k[i].iter()) {
                *e = *e + *k * (h * C::E[i]);
            }
        }
    }

    #[inline]
    fn undo_step(&mut self, prev: &Self) {
        self.i = prev.i;
        if C::FSAL {
            self.k[STAGES - 1].clone_from(&prev.k[STAGES - 1]);
        }
    }
}
