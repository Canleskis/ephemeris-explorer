use crate::{
    EvalFailed, Problem, SecondOrderODE, SecondOrderState, State,
    runge_kutta::{RKEmbedded, RKInstance, RKState},
};

/// Coefficients for Explicit Runge-Kutta-Nyström (ERKN) integrators, more commonly known as the
/// Butcher tableau.
///
/// ERKN methods are integrators for second-order ordinary differential equations of the form:  
/// `y'' = f(t, y)`
#[doc(alias = "Butcher Tableau")]
pub trait ERKNCoefficients {
    const FSAL: bool;

    const ORDER: u32;

    const A: &'static [&'static [f64]];

    const BP: &'static [f64];

    const BV: &'static [f64];

    const C: &'static [f64];
}

pub trait EERKNCoefficients: ERKNCoefficients {
    const ORDER_EMBEDDED: u32;

    const EP: &'static [f64];

    const EV: &'static [f64];
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

impl<C, const STAGES: usize, P, V> RKInstance<P> for ERKN<C, [V; STAGES], V>
where
    C: ERKNCoefficients,
    P: Problem<State = SecondOrderState<V>>,
    P::ODE: SecondOrderODE<V>,
    V: State + Clone,
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
    fn advance(&mut self, h: f64, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();

        // let mut yi = self.y.clone();
        for s in 0..STAGES {
            if C::FSAL && s == 0 && self.i > 0 {
                self.dk.swap(s, STAGES - 1);
                continue;
            }

            // tᵢ = t₀ + hcᵢ
            let ti = problem.time + C::C[s] * h;
            // yᵢ₊₁ = y₀ + hcᵢy₀' + h²∑ⱼāᵢⱼkⱼ'
            self.yi.clone_from(&problem.state.y);
            for (yi, dy) in self.yi.iter_mut().zip(problem.state.dy.iter()) {
                *yi = *yi + *dy * (C::C[s] * h);
            }
            for j in 0..s {
                for (yi, kp) in self.yi.iter_mut().zip(self.dk[j].iter()) {
                    *yi = *yi + *kp * (C::A[s][j] * h * h);
                }
            }
            // kᵢ' = f(tᵢ, yᵢ)
            problem.ode.eval(ti, &self.yi, &mut self.dk[s])?; // TODO: Set kᵢ to zeroes.
        }

        // yₙ₊₁ = yₙ + hyₙ' + h²∑ᵢb̄ᵢkᵢ'
        // yₙ₊₁' = yₙ' + h∑ᵢbᵢkᵢ'
        for (y, dy) in problem.state.y.iter_mut().zip(problem.state.dy.iter()) {
            *y = *y + (*dy * h);
        }
        for i in 0..STAGES {
            for ((y, dy), kp) in problem
                .state
                .y
                .iter_mut()
                .zip(problem.state.dy.iter_mut())
                .zip(self.dk[i].iter())
            {
                *y = *y + *kp * (C::BP[i] * h * h);
                *dy = *dy + *kp * (C::BV[i] * h);
            }
        }

        problem.time += h;
        self.i += 1;

        Ok(())
    }
}

impl<C, const STAGES: usize, V> RKEmbedded for ERKN<C, [V; STAGES], V>
where
    C: EERKNCoefficients,
    V: State + Clone,
{
    type State = SecondOrderState<V>;

    const LOWER_ORDER: u32 = match C::ORDER < C::ORDER_EMBEDDED {
        true => C::ORDER,
        false => C::ORDER_EMBEDDED,
    };

    #[inline]
    fn error(&self, h: f64, error: &mut SecondOrderState<V>) {
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
                *ey = *ey + *kp * (C::EP[i] * h * h);
                *edy = *edy + *kp * (C::EV[i] * h);
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
