use crate::{
    EvalFailed, FirstOrderODE, Problem, State,
    runge_kutta::{RKEmbedded, RKInstance, RKState},
};

/// Coefficients for Explicit Runge-Kutta (ERK) integrators, more commonly known as the Butcher
/// tableau.
///
/// ERK methods are integrators for first-order ordinary differential equations of the form:  
/// `y' = f(t, y)`
#[doc(alias = "Butcher Tableau")]
pub trait ERKCoefficients {
    const FSAL: bool;

    const ORDER: u32;

    const A: &'static [&'static [f64]];

    const B: &'static [f64];

    const C: &'static [f64];
}

/// Coefficients for Embedded Explicit Runge-Kutta integrators (EERK), defined by error
/// coefficients as well as the Butcher tableau.
///
/// These coefficients are determined by an _embedded_ Runge-Kutta method that is used to estimate
/// the local error of the solution, which is then used to adapt the step size.
pub trait EERKCoefficients: ERKCoefficients {
    const ORDER_EMBEDDED: u32;

    /// Error coefficients, defined as `eᵢ = bᵢ - bᵢ*`.
    const E: &'static [f64];
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

impl<C, const STAGES: usize, P> RKInstance<P> for ERK<C, [P::State; STAGES]>
where
    C: ERKCoefficients,
    P: Problem,
    P::State: State + Clone,
    P::ODE: FirstOrderODE<P::State>,
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
    fn advance(&mut self, h: f64, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();
        let mut yi = problem.state.clone();
        for s in 0..STAGES {
            if C::FSAL && s == 0 && self.i > 0 {
                self.k.swap(s, STAGES - 1);
                continue;
            }

            // tᵢ = t₀ + hcᵢ
            let ti = problem.time + C::C[s] * h;
            // yᵢ₊₁ = y₀ + h∑ⱼaᵢⱼkⱼ
            yi.clone_from(&problem.state);
            for j in 0..s {
                for (yi, k) in yi.iter_mut().zip(self.k[j].iter()) {
                    *yi = *yi + *k * (C::A[s][j] * h);
                }
            }
            // kᵢ = f(tᵢ, yᵢ)
            problem.ode.eval(ti, &yi, &mut self.k[s])?;
        }

        // yₙ₊₁ = yₙ + h∑ᵢbᵢkᵢ
        for i in 0..STAGES {
            for (y, k) in problem.state.iter_mut().zip(self.k[i].iter()) {
                *y = *y + *k * (C::B[i] * h);
            }
        }

        problem.time += h;
        self.i += 1;

        Ok(())
    }
}

impl<C, const STAGES: usize, V> RKEmbedded for ERK<C, [V; STAGES]>
where
    C: EERKCoefficients,
    V: State + Clone,
{
    type State = V;

    const LOWER_ORDER: u32 = match C::ORDER < C::ORDER_EMBEDDED {
        true => C::ORDER,
        false => C::ORDER_EMBEDDED,
    };

    #[inline]
    fn error(&self, h: f64, error: &mut Self::State) {
        error.zero();
        // eₙ₊₁ = h∑ᵢeᵢkᵢ = h∑ᵢ(bᵢ - bᵢ*)kᵢ
        for i in 0..STAGES {
            for (e, k) in error.iter_mut().zip(self.k[i].iter()) {
                *e = *e + *k * (C::E[i] * h);
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
