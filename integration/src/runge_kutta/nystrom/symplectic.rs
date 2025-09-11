use crate::{
    problem::{EvalFailed, Problem, SecondOrderODE, SecondOrderState, State},
    ratio::Ratio,
    runge_kutta::{RKInstance, RKState},
};
use std::ops::{Add, Mul};

/// Coefficients for Symplectic Runge-Kutta-Nyström (SRKN) integrators.
///
/// SRKN methods are integrators for second-order ordinary differential equations of the form:  
/// `y'' = f(t, y)`
///
/// SRKN methods are a specialized case of Partitioned Runge-Kutta (PRK) methods and so coefficients
/// for PRK methods can be adapted to SRKN methods.
///
/// The coefficients are expressed in a simplified form of two Butcher tableaus, where
/// aᵢⱼ = bⱼ for i ≥ j, called A here, and
/// āᵢⱼ = b̄ⱼ for i > j, called B here.
#[doc(alias = "Butcher Tableau")]
pub trait SRKNCoefficients {
    const FSAL: bool;

    const A: &[Ratio];

    const B: &[Ratio];

    #[inline]
    fn c(i: usize) -> Ratio {
        Self::A[..i]
            .iter()
            .fold(Ratio::ZERO, |acc, b| acc.const_add(*b))
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SRKN<C, V> {
    pub i: u32,
    pub ddy: V,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, V> RKState for SRKN<C, V> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, V, P> RKInstance<P> for SRKN<C, V>
where
    C: SRKNCoefficients,
    V: State + Clone,
    P: Problem<Variable = V::Variable, State = SecondOrderState<V>>,
    P::Variable: Mul<P::Time, Output = P::Variable> + Add<Output = P::Variable> + Default + Copy,
    P::Time: Mul<Ratio, Output = P::Time> + Add<Output = P::Time> + Copy,
    P::ODE: SecondOrderODE<P::Time, V>,
{
    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            ddy: problem.as_ref().state.dy.clone().zeroed(),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: P::Time, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();
        // Detailed source and proof for the formulas used can be found here:
        // https://en.wikipedia.org/wiki/Symplectic_integrator#Methods_for_constructing_symplectic_algorithms
        for s in 0..C::A.len() {
            if !C::FSAL || s > 0 || self.i == 0 {
                // tᵢ = t₀ + cᵢ * h
                let t_stage = problem.time + h * C::c(s);
                // yᵢ'' = f(tᵢ, yᵢ)
                problem
                    .ode
                    .eval(t_stage, &problem.state.y, self.ddy.zero())?;
            }

            // yᵢ₊₁' = yᵢ' + bᵢ * h * yᵢ₊₁''
            // yᵢ₊₁ = yᵢ + aᵢ * h * yᵢ₊₁'
            for ((y, dy), ddy) in problem
                .state
                .y
                .iter_mut()
                .zip(problem.state.dy.iter_mut())
                .zip(self.ddy.iter())
            {
                *dy = *dy + *ddy * (h * C::B[s]);
                *y = *y + *dy * (h * C::A[s]);
            }
        }

        problem.time = problem.time + h;
        self.i += 1;

        Ok(())
    }
}
