use crate::{
    EvalFailed, Problem, SecondOrderODEGeneral, SecondOrderState, State,
    runge_kutta::{RKEmbedded, RKInstance, RKState},
};

/// Coefficients for Explicit Runge-Kutta-Nyström Generalized (ERKNG) integrators, more commonly
/// known as the Butcher tableau.
///
/// ERKNG methods are integrators for second-order ordinary differential equations of the form:  
/// `y'' = f(t, y, y')`
#[doc(alias = "Butcher Tableau")]
pub trait ERKNGCoefficients {
    const FSAL: bool;

    const ORDER: u32;

    const AP: &'static [&'static [f64]];

    const AV: &'static [&'static [f64]];

    const BP: &'static [f64];

    const BV: &'static [f64];

    const C: &'static [f64];
}

pub trait EERKNGCoefficients: ERKNGCoefficients {
    const ORDER_EMBEDDED: u32;

    const EP: &'static [f64];

    const EV: &'static [f64];
}

#[derive(Clone, Copy, Debug)]
pub struct ERKNG<C, K, V> {
    pub i: u32,
    pub dk: K,
    yi: V,
    dyi: V,
    _phantom: std::marker::PhantomData<C>,
}

impl<C, K, V> RKState for ERKNG<C, K, V> {
    #[inline]
    fn step_count(&self) -> u32 {
        self.i
    }
}

impl<C, const STAGES: usize, P, V> RKInstance<P> for ERKNG<C, [V; STAGES], V>
where
    C: ERKNGCoefficients,
    P: Problem<State = SecondOrderState<V>>,
    P::ODE: SecondOrderODEGeneral<V>,
    V: State + Clone,
{
    #[inline]
    fn from_problem(problem: &P) -> Self {
        Self {
            i: 0,
            dk: std::array::from_fn(|_| problem.as_ref().state.dy.clone()),
            yi: problem.as_ref().state.y.clone(),
            dyi: problem.as_ref().state.dy.clone(),
            _phantom: std::marker::PhantomData,
        }
    }

    #[inline]
    fn advance(&mut self, h: f64, problem: &mut P) -> Result<(), EvalFailed> {
        let problem = problem.as_mut();

        // let mut yi = rk.y.clone();
        // let mut dyi = rk.dy.clone();
        for s in 0..STAGES {
            if C::FSAL && s == 0 && self.i > 0 {
                self.dk.swap(s, STAGES - 1);
                continue;
            }

            // tᵢ = t₀ + hcᵢ
            let ti = problem.time + C::C[s] * h;
            // yᵢ₊₁ = y₀ + hcᵢy₀' + h²∑ⱼāᵢⱼkⱼ'
            // yᵢ₊₁' = y₀' + h∑ⱼaᵢⱼkⱼ'
            self.yi.clone_from(&problem.state.y);
            for (yi, dy) in self.yi.iter_mut().zip(problem.state.dy.iter()) {
                *yi = *yi + *dy * (C::C[s] * h);
            }
            self.dyi.clone_from(&problem.state.dy);
            for j in 0..s {
                for ((yi, dyi), kp) in self
                    .yi
                    .iter_mut()
                    .zip(self.dyi.iter_mut())
                    .zip(self.dk[j].iter())
                {
                    *yi = *yi + *kp * (C::AP[s][j] * h * h);
                    *dyi = *dyi + *kp * (C::AV[s][j] * h);
                }
            }
            // kᵢ' = f(tᵢ, yᵢ)
            problem.ode.eval(ti, &self.yi, &self.dyi, &mut self.dk[s])?;
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

impl<C, const STAGES: usize, V> RKEmbedded for ERKNG<C, [V; STAGES], V>
where
    C: EERKNGCoefficients,
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
