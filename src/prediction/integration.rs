use bevy::math::DVec3;

#[derive(Debug, Clone)]
pub enum StepError {
    StepSizeUnderflow,
    MaxIterationsReached,
    MaxTimeReached,
    FailedEvaluation,
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::StepSizeUnderflow => write!(f, "step size underflow"),
            Self::MaxIterationsReached => write!(f, "max iterations reached"),
            Self::MaxTimeReached => write!(f, "max time reached"),
            Self::FailedEvaluation => write!(f, "failed evaluation"),
        }
    }
}

impl std::error::Error for StepError {}

pub trait IntegrationState {
    fn velocity(&mut self) -> &mut DVec3;

    fn position(&mut self) -> &mut DVec3;
}

#[derive(Default, Clone)]
#[expect(clippy::upper_case_acronyms)]
pub struct PEFRL<S> {
    time: f64,
    delta: f64,
    state: Vec<S>,
    buf: Vec<DVec3>,
}

impl<S> PEFRL<S> {
    const LAMBDA: f64 = -0.2123418310626054;

    const XI: f64 = 0.1786178958448091;

    const CHI: f64 = -0.0662645826698185;

    pub fn new(initial_time: f64, delta: f64, inital_state: Vec<S>) -> Self {
        Self {
            time: initial_time,
            delta,
            buf: Vec::with_capacity(inital_state.len()),
            state: inital_state,
        }
    }

    #[inline]
    pub fn time(&self) -> f64 {
        self.time
    }

    #[inline]
    pub fn delta(&self) -> f64 {
        self.delta
    }

    #[inline]
    pub fn state(&self) -> &[S] {
        &self.state
    }
}

impl<S: IntegrationState> PEFRL<S> {
    #[inline]
    pub fn step<F>(&mut self, evaluate: F) -> Result<(), StepError>
    where
        F: Fn(f64, &[S], &mut Vec<DVec3>),
    {
        let dt = self.delta;
        let dt_half = dt / 2.0;
        let h1 = (1.0 - 2.0 * Self::LAMBDA) * dt_half;
        let h2 = Self::LAMBDA * dt;

        for state in self.state.iter_mut() {
            let velocity = *state.velocity();
            *state.position() += Self::XI * dt * velocity;
        }

        self.buf.clear();
        evaluate(Self::XI * dt, &self.state, &mut self.buf);
        for (state, acceleration) in self.state.iter_mut().zip(self.buf.iter()) {
            *state.velocity() += h1 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::CHI * dt * velocity;
        }

        self.buf.clear();
        evaluate((Self::XI + Self::CHI) * dt, &self.state, &mut self.buf);
        for (state, acceleration) in self.state.iter_mut().zip(self.buf.iter()) {
            *state.velocity() += h2 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += (1.0 - 2.0 * (Self::CHI + Self::XI)) * dt * velocity;
        }

        self.buf.clear();
        evaluate(
            (1.0 - Self::XI - Self::CHI) * dt,
            &self.state,
            &mut self.buf,
        );
        for (state, acceleration) in self.state.iter_mut().zip(self.buf.iter()) {
            *state.velocity() += h2 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::CHI * dt * velocity;
        }

        self.buf.clear();
        evaluate((1.0 - Self::XI) * dt, &self.state, &mut self.buf);
        for (state, acceleration) in self.state.iter_mut().zip(self.buf.iter()) {
            *state.velocity() += h1 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::XI * dt * velocity;
        }

        self.time += dt;

        Ok(())
    }
}

/// Used for adaptive step size control
#[derive(Debug, Clone, Copy)]
pub struct Controller {
    alpha: f64,
    beta: f64,
    facc1: f64,
    facc2: f64,
    fac_old: f64,
    reject: bool,
    safety_factor: f64,
}

impl Controller {
    pub fn new(alpha: f64, beta: f64, fac_max: f64, fac_min: f64, safety_factor: f64) -> Self {
        Self {
            alpha,
            beta,
            facc1: 1.0 / fac_min,
            facc2: 1.0 / fac_max,
            fac_old: 1.0E-4,
            reject: false,
            safety_factor,
        }
    }

    /// Determines if the step must be accepted or rejected and adapts the step size accordingly.
    #[inline]
    pub fn accept(&mut self, err: f64, h: f64, h_max: f64, h_new: &mut f64) -> bool {
        let fac11 = err.powf(self.alpha);
        let mut fac = fac11 * self.fac_old.powf(-self.beta);
        fac = (self.facc2).max((self.facc1).min(fac / self.safety_factor));
        *h_new = h / fac;

        if err <= 1.0 {
            // Accept step
            self.fac_old = err.max(1.0E-4);

            if h_new.abs() > h_max {
                *h_new = h_max;
            }
            if self.reject {
                *h_new = h_new.abs().min(h.abs());
            }

            self.reject = false;
            true
        } else {
            // Reject step
            *h_new = h / ((self.facc1).min(fac11 / self.safety_factor));
            self.reject = true;
            false
        }
    }
}

/// Functions to retrieve the coefficients for the Dormand-Prince method of order 5(4) with dense output of order 4.
mod butcher_tableau {
    #[expect(clippy::type_complexity)]
    const A: ([f64; 1], [f64; 2], [f64; 3], [f64; 4], [f64; 5], [f64; 6]) = (
        [1.0 / 5.0],
        [3.0 / 40.0, 9.0 / 40.0],
        [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0],
        [
            19372.0 / 6561.0,
            -25360.0 / 2187.0,
            64448.0 / 6561.0,
            -212.0 / 729.0,
        ],
        [
            9017.0 / 3168.0,
            -355.0 / 33.0,
            46732.0 / 5247.0,
            49.0 / 176.0,
            -5103.0 / 18656.0,
        ],
        [
            35.0 / 384.0,
            0.0,
            500.0 / 1113.0,
            125.0 / 192.0,
            -2187.0 / 6784.0,
            11.0 / 84.0,
        ],
    );
    const C: [f64; 7] = [0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0];
    const D: [f64; 7] = [
        -12715105075.0 / 11282082432.0,
        0.0,
        87487479700.0 / 32700410799.0,
        -10690763975.0 / 1880347072.0,
        701980252875.0 / 199316789632.0,
        -1453857185.0 / 822651844.0,
        69997945.0 / 29380423.0,
    ];
    const E: [f64; 7] = [
        71.0 / 57600.0,
        0.0,
        -71.0 / 16695.0,
        71.0 / 1920.0,
        -686.0 / 13487.0,
        22.0 / 525.0,
        -1.0 / 40.0,
    ];

    /// Returns the _a<sub>ij</sub>_ coefficient of the Runge-Kutta matrix.
    #[inline]
    pub const fn a(i: usize, j: usize) -> f64 {
        match i - 2 {
            0 => A.0[j - 1],
            1 => A.1[j - 1],
            2 => A.2[j - 1],
            3 => A.3[j - 1],
            4 => A.4[j - 1],
            5 => A.5[j - 1],
            _ => panic!("index out of bounds"),
        }
    }

    /// Returns the _c<sub>i</sub>_ coefficient.
    #[inline]
    pub const fn c(i: usize) -> f64 {
        C[i - 1]
    }

    /// Returns the _d<sub>i</sub>_ coefficient.
    #[expect(unused)]
    #[inline]
    pub const fn d(i: usize) -> f64 {
        D[i - 1]
    }

    /// Returns the _e<sub>i</sub>_ coefficient.
    #[inline]
    pub const fn e(i: usize) -> f64 {
        E[i - 1]
    }
}

/// Structure containing the parameters for the numerical integration.
#[derive(Debug, Clone, Copy)]
pub struct DormandPrince5<const DIM: usize, V> {
    time: f64,
    state: V,
    k: Option<[V; 7]>,
    rtol: f64,
    atol: f64,
    delta: f64,
    delta_new: f64,
    delta_max: f64,
    n: u32,
    n_max: u32,
    controller: Controller,
}

// Implementation largely based on the `ode_solvers` crate and adapted to be iteratable.
impl<const DIM: usize, V> DormandPrince5<DIM, V> {
    pub fn new(initial_time: f64, initial_state: V, rtol: f64, atol: f64, n_max: u32) -> Self {
        Self {
            time: initial_time,
            state: initial_state,
            k: None,
            rtol,
            atol,
            delta: 0.0,
            delta_new: 0.0,
            delta_max: f64::MAX,
            n: 0,
            n_max,
            controller: Controller::new(0.2 - 0.04 * 0.75, 0.04, 10.0, 0.2, 0.9),
        }
    }

    #[inline]
    pub fn time(&self) -> f64 {
        self.time
    }

    #[inline]
    pub fn rtol(&self) -> f64 {
        self.rtol
    }

    #[inline]
    pub fn atol(&self) -> f64 {
        self.atol
    }

    #[inline]
    pub fn n_max(&self) -> u32 {
        self.n_max
    }

    #[expect(unused)]
    #[inline]
    pub fn delta(&self) -> f64 {
        self.delta
    }

    #[inline]
    pub fn state(&self) -> &V {
        &self.state
    }

    #[expect(unused)]
    #[inline]
    pub fn state_mut(&mut self) -> &mut V {
        &mut self.state
    }

    #[inline]
    pub fn set_n_max(&mut self, n_max: u32) {
        self.n_max = n_max;
    }

    #[inline]
    pub fn reset(&mut self)
    where
        V: Clone,
    {
        *self = Self::new(
            self.time,
            self.state.clone(),
            self.rtol,
            self.atol,
            self.n_max,
        );
    }

    /// Compute the initial stepsize
    #[inline]
    fn init_dt<F>(&self, delta_max: f64, mut evaluate: F) -> Result<f64, ()>
    where
        V: std::ops::Index<usize, Output = f64>
            + std::ops::Add<Output = V>
            + std::ops::Mul<f64, Output = V>
            + Default
            + Copy,
        F: FnMut(f64, &V, &mut V) -> Result<(), ()>,
    {
        let mut f0 = V::default();
        evaluate(self.time, &self.state, &mut f0)?;

        // Compute the norm of y0 and f0
        let mut d0 = 0.0;
        let mut d1 = 0.0;
        for i in 0..DIM {
            let y_i = self.state[i];
            let sci = self.atol + y_i.abs() * self.rtol;
            d0 += (y_i / sci) * (y_i / sci);
            let f0_i = f0[i];
            d1 += (f0_i / sci) * (f0_i / sci);
        }

        // Compute h0
        let tol = 1.0E-10;
        let mut h0 = if d0 < tol || d1 < tol {
            1.0E-6
        } else {
            0.01 * (d0 / d1).sqrt()
        };

        h0 = h0.min(delta_max);

        let y1 = self.state + f0 * h0;
        let mut f1 = V::default();
        evaluate(self.time + h0, &y1, &mut f1)?;

        // Compute the norm of f1-f0 divided by h0
        let mut d2 = 0.0;
        for i in 0..DIM {
            let f0_i = f0[i];
            let f1_i = f1[i];
            let y_i = self.state[i];
            let sci = self.atol + y_i.abs() * self.rtol;
            d2 += ((f1_i - f0_i) / sci) * ((f1_i - f0_i) / sci);
        }
        d2 = d2.sqrt() / h0;

        let h1 = if d1.sqrt().max(d2.abs()) <= 1.0E-15 {
            1.0E-6_f64.max(h0.abs() * 1.0E-3)
        } else {
            (0.01 / (d1.sqrt().max(d2))).powf(1.0 / 5.0)
        };

        Ok((100.0 * h0.abs()).min(h1.min(delta_max)))
    }

    #[inline]
    pub fn step<F>(&mut self, bound: f64, mut evaluate: F) -> Result<(), StepError>
    where
        V: std::ops::Index<usize, Output = f64>
            + std::ops::Add<Output = V>
            + std::ops::Mul<f64, Output = V>
            + Default
            + Copy,
        F: FnMut(f64, &V, &mut V) -> Result<(), ()>,
    {
        loop {
            // Check if step number is within allowed range
            if self.n > self.n_max {
                return Err(StepError::MaxIterationsReached);
            }

            if (self.time + 1.01 * self.delta - bound) >= 0.0 {
                self.delta = bound - self.time;
                if self.delta == 0.0 {
                    return Err(StepError::MaxTimeReached);
                }
            } else if self.delta == 0.0 {
                self.delta = self
                    .init_dt(self.delta_max.min(bound - self.time), &mut evaluate)
                    .map_err(|_| StepError::FailedEvaluation)?;
            }

            if 0.1 * self.delta.abs() <= f64::EPSILON * self.time.abs() {
                return Err(StepError::StepSizeUnderflow);
            }

            let k = match self.k {
                Some(ref mut k) => k,
                None => {
                    let mut k = [V::default(); 7];
                    evaluate(self.time, &self.state, &mut k[0])
                        .map_err(|_| StepError::FailedEvaluation)?;
                    self.k.insert(k)
                }
            };

            // 6 Stages
            let mut y_next = V::default();
            for s in 1..7 {
                y_next = self.state;
                for (j, k_value) in k.iter().enumerate().take(s) {
                    y_next = y_next + *k_value * self.delta * butcher_tableau::a(s + 1, j + 1);
                }
                evaluate(
                    self.time + self.delta * butcher_tableau::c(s + 1),
                    &y_next,
                    &mut k[s],
                )
                .map_err(|_| StepError::FailedEvaluation)?;
            }
            k[1] = k[6];

            // Compute error estimate
            k[3] = (k[0] * butcher_tableau::e(1)
                + k[2] * butcher_tableau::e(3)
                + k[3] * butcher_tableau::e(4)
                + k[4] * butcher_tableau::e(5)
                + k[5] * butcher_tableau::e(6)
                + k[1] * butcher_tableau::e(7))
                * self.delta;

            // Compute error
            let mut err = 0.0;
            for i in 0..DIM {
                let y_i = self.state[i];
                let y_next_i = y_next[i];
                let sc_i = self.atol + y_i.abs().max(y_next_i.abs()) * self.rtol;
                let err_est_i = k[3][i];
                err += (err_est_i / sc_i) * (err_est_i / sc_i);
            }
            err = (err / DIM as f64).sqrt();

            let delta_old = self.delta;
            self.delta = self.delta_new;
            self.n += 1;

            // Step size control
            if self
                .controller
                .accept(err, delta_old, self.delta_max, &mut self.delta_new)
            {
                k[0] = k[1];
                self.state = y_next;
                self.time += delta_old;

                return Ok(());
            }
        }
    }
}
