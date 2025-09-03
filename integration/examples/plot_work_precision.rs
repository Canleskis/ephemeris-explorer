use bevy_math::DVec3;
use integration::prelude::*;
use particular::gravity::newtonian::AccelerationAt;
use particular::prelude::*;
use plotters::prelude::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct StateVector {
    pub position: DVec3,
    pub velocity: DVec3,
}

impl StateVector {
    #[inline]
    pub fn new(position: DVec3, velocity: DVec3) -> Self {
        Self { position, velocity }
    }

    #[inline]
    pub fn splat(v: DVec3) -> Self {
        Self {
            position: v,
            velocity: v,
        }
    }

    #[inline]
    pub fn abs(&self) -> Self {
        Self {
            position: self.position.abs(),
            velocity: self.velocity.abs(),
        }
    }

    #[inline]
    pub fn component_mul(&self, other: &Self) -> Self {
        StateVector {
            position: self.position * other.position,
            velocity: self.velocity * other.velocity,
        }
    }

    #[inline]
    pub fn component_div(&self, other: &Self) -> Self {
        StateVector {
            position: self.position / other.position,
            velocity: self.velocity / other.velocity,
        }
    }

    #[inline]
    pub fn max(&self, rhs: &Self) -> Self {
        Self {
            position: self.position.max(rhs.position),
            velocity: self.velocity.max(rhs.velocity),
        }
    }

    #[inline]
    pub fn max_element(&self) -> f64 {
        self.position.max_element().max(self.velocity.max_element())
    }
}

impl std::ops::Add for StateVector {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
        }
    }
}

impl std::ops::Mul<f64> for StateVector {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            position: self.position * rhs,
            velocity: self.velocity * rhs,
        }
    }
}

#[derive(Clone, Copy, Mass)]
pub struct Body {
    pub state_vector: StateVector,
    pub mu: f64,
}

impl Body {
    #[inline]
    pub fn motion_equation_order_1(&self) -> impl FirstOrderODE<[StateVector; 1]> + Clone {
        move |_: f64, [y]: &[StateVector; 1], [dy]: &mut [StateVector; 1]| {
            dy.position = y.velocity;
            dy.velocity = self.acceleration_at::<false>(&y.position, &0.0);

            Ok(())
        }
    }

    #[inline]
    pub fn motion_equation_order_2(&self) -> impl SecondOrderODE<[DVec3; 1]> + Clone {
        |_: f64, [y]: &[DVec3; 1], [ddy]: &mut [DVec3; 1]| {
            *ddy = self.acceleration_at::<false>(y, &0.0);

            Ok(())
        }
    }

    #[inline]
    pub fn motion_equation_order_2_general(
        &self,
    ) -> impl SecondOrderODEGeneral<[DVec3; 1]> + Clone {
        |_, [y]: &[DVec3; 1], _: &[DVec3; 1], [ddy]: &mut [DVec3; 1]| {
            *ddy = self.acceleration_at::<false>(y, &0.0);

            Ok(())
        }
    }
}

impl Position for Body {
    type Vector = DVec3;

    #[inline]
    fn position(&self) -> DVec3 {
        self.state_vector.position
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OrbitalElements {
    pub a: f64,     // Semi-major axis
    pub e: f64,     // Eccentricity
    pub i: f64,     // Inclination
    pub omega: f64, // Longitude of the ascending node
    pub w: f64,     // Argument of periapsis
    pub t: f64,     // Mean anomaly
}

impl OrbitalElements {
    pub fn from_state(state_vector: StateVector, mu: f64) -> Self {
        let r = state_vector.position.length();
        let DVec3 { x, y, z } = state_vector.position;
        let v = state_vector.velocity.length();
        let h = state_vector.position.cross(state_vector.velocity);
        let e = v * v / 2.0 - mu / r;
        let a = -mu / (2.0 * e);
        let e = (1.0 - h.length_squared() / (mu * a)).sqrt();
        let p = a * (1.0 - e * e);
        let ta = ((p / mu).sqrt() * state_vector.velocity.dot(state_vector.position)).atan2(p - r);
        let i = (h.z / h.length()).acos();
        let omega = h.x.atan2(-h.y);
        let u = (z / i.sin()).atan2(x * omega.cos() + y * omega.sin());
        let w = u - ta;
        let ea = 2.0 * (((1.0 - e) / (1.0 + e)).sqrt() * (ta / 2.0).tan()).atan();
        let n = (mu / (a * a * a)).sqrt();
        let t = -1.0 / n * (ea - e * ea.sin());

        Self {
            a,
            e,
            i,
            omega,
            w,
            t,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Orbit {
    mu: f64,
    elements: OrbitalElements,
}

impl Orbit {
    pub fn from_state(state_vector: StateVector, mu: f64) -> Self {
        Self {
            mu,
            elements: OrbitalElements::from_state(state_vector, mu),
        }
    }

    pub fn period(&self) -> f64 {
        std::f64::consts::TAU
            * ((self.elements.a * self.elements.a * self.elements.a) / self.mu).sqrt()
    }

    pub fn at(&self, time: f64) -> StateVector {
        const MAX_ITERATIONS: usize = 100;
        const TOLERANCE: f64 = 1e-12;

        let OrbitalElements {
            a,
            e,
            i,
            omega,
            w,
            t,
        } = self.elements;

        let n = (self.mu / (a * a * a)).sqrt();
        let m = (n * (time - t)) % std::f64::consts::TAU;
        let mut ea = std::f64::consts::PI;
        for _ in 0..MAX_ITERATIONS {
            let ea_n = (ea - e * ea.sin() - m) / (1.0 - e * ea.cos());
            ea -= ea_n;
            if ea_n.abs() < TOLERANCE {
                break;
            }
        }

        let ta = ((1.0 - e * e).sqrt() * ea.sin()).atan2(ea.cos() - e);
        let r = a * (1.0 - e * ea.cos());

        let x = r * (omega.cos() * (w + ta).cos() - omega.sin() * (w + ta).sin() * i.cos());
        let y = r * (omega.sin() * (w + ta).cos() + omega.cos() * (w + ta).sin() * i.cos());
        let z = r * (w + ta).sin() * i.sin();

        StateVector::new(DVec3::new(x, y, z), DVec3::ZERO)
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Earth
    let main = Body {
        state_vector: StateVector::new(DVec3::ZERO, DVec3::ZERO),
        mu: 398600.43550702266,
    };
    let radius = 6_378.0;

    let p = radius + 300.0;
    let v = (main.mu / p).sqrt() + 0.5;
    let initial_position = DVec3::new(0.0, 0.0, p);
    let initial_velocity = DVec3::new(0.0, v, 0.0);
    let initial_state = StateVector::new(initial_position, initial_velocity);
    println!(
        "Initial position: {}\nInitial velocity: {}",
        initial_state.position, initial_state.velocity
    );
    let orbit = Orbit::from_state(initial_state, main.mu);
    println!("Orbital period: {} minutes", orbit.period() / 60.0);

    let d = orbit.period() * 100.0;
    println!("Integration time: {} days", d / 86400.0);

    println!("Orbital elements: {:?}", orbit.elements);

    let clas = WPFramework::new(
        ODEProblem::new(
            0.0,
            d,
            [initial_state],
            Tracked::new(main.motion_equation_order_1()),
        ),
        |t| [orbit.at(t)],
        |[state]: &[StateVector; 1], [exact]: &[StateVector; 1]| {
            state.position.distance(exact.position)
        },
    );

    let nyst = WPFramework::new(
        ODEProblem::new(
            clas.problem.time,
            clas.problem.bound,
            SecondOrderState::new([initial_position], [initial_velocity]),
            Tracked::new(main.motion_equation_order_2()),
        ),
        |t| SecondOrderState::new([orbit.at(t).position], [orbit.at(t).velocity]),
        |state: &SecondOrderState<[DVec3; 1]>, exact: &SecondOrderState<[DVec3; 1]>| {
            state.y[0].distance(exact.y[0])
        },
    );

    let nystg = WPFramework::new(
        ODEProblem::new(
            nyst.problem.time,
            nyst.problem.bound,
            nyst.problem.state,
            Tracked::new(main.motion_equation_order_2_general()),
        ),
        nyst.exact,
        nyst.absolute_error,
    );

    plot_work_precision(
        "adaptive_rk.png",
        &[
            ("CashKarp45", clas.compute(CashKarp45::idx)),
            ("DormandPrince54", clas.compute(DormandPrince54::idx)),
            ("DormandPrince87", clas.compute(DormandPrince87::idx)),
            ("Fehlberg45", clas.compute(Fehlberg45::idx)),
            ("Tsitouras75", clas.compute(Tsitouras75::idx)),
            ("Verner87", clas.compute(Verner87::idx)),
            ("Tsitouras75N", nyst.compute(Tsitouras75Nystrom::idx)),
            ("Fine45", nystg.compute(Fine45::idx)),
        ],
    )?;

    plot_work_precision(
        "fixed_rk.png",
        &[
            ("BlanesMoan6B", nyst.compute(BlanesMoan6B::idx)),
            ("BlanesMoan11B", nyst.compute(BlanesMoan11B::idx)),
            ("BlanesMoan14A", nyst.compute(BlanesMoan14A::idx)),
            ("ForestRuth", nyst.compute(ForestRuth::idx)),
            ("McLachlanO4", nyst.compute(McLachlanO4::idx)),
            ("McLachlanSS17", nyst.compute(McLachlanSS17::idx)),
            ("Pefrl", nyst.compute(Pefrl::idx)),
            ("Ruth", nyst.compute(Ruth::idx)),
        ],
    )?;

    Ok(())
}

const MIN_ERROR: f64 = 0.01;
const MAX_ERROR: f64 = 1_000_000.0;

const MIN_EVALS: u32 = 10_000;
const MAX_EVALS: u32 = 1_000_000;

fn plot_work_precision<T: AsRef<std::path::Path>>(
    path: T,
    data: &[(&str, Vec<(f64, u32)>)],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(&path, (1280, 720)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .margin((1).percent())
        .set_label_area_size(LabelAreaPosition::Left, (8).percent())
        .set_label_area_size(LabelAreaPosition::Bottom, (4).percent())
        .build_cartesian_2d(
            (MIN_ERROR..MAX_ERROR).log_scale(),
            (MIN_EVALS..MAX_EVALS).log_scale(),
        )?;

    chart
        .configure_mesh()
        .x_desc("Error")
        .y_desc("Evaluations")
        .draw()?;

    for (i, (name, data)) in data.iter().enumerate() {
        let color = Palette99::pick(i).mix(0.9);

        chart
            .draw_series(LineSeries::new(data.iter().cloned(), color.stroke_width(2)))?
            .label(*name)
            .legend(move |(x, y)| Rectangle::new([(x, y - 2), (x + 20, y + 2)], color.filled()));
        chart.draw_series(PointSeries::<_, _, Circle<_, _>, _>::new(
            data.iter().cloned(),
            3,
            ShapeStyle::from(color).filled(),
        ))?;
    }

    chart.configure_series_labels().border_style(BLACK).draw()?;

    root.present().unwrap();

    Ok(())
}

trait FromIndexFixed: NewMethod<f64> + Sized {
    #[inline]
    fn idx(i: i32, n: i32) -> Self {
        Self::new(1.0 * (1 << (n - i)) as f64)
    }
}
impl<M: NewMethod<f64>> FromIndexFixed for M {}

trait FromIndexAdaptive<V: Default>: NewMethod<AdaptiveRKParams<Tolerance<f64, V>>> + Sized {
    #[inline]
    fn idx(i: i32, _: i32) -> Self {
        let tol = 10f64.powi(3 - i);
        Self::new(AdaptiveRKParams::new(Tolerance::absolute(tol), u32::MAX))
    }
}
impl<V: Default, M: NewMethod<AdaptiveRKParams<Tolerance<f64, V>>>> FromIndexAdaptive<V> for M {}

#[derive(Clone, Copy)]
struct WPFramework<P, E, A> {
    problem: P,
    exact: E,
    absolute_error: A,
}

#[derive(Clone, Copy)]
struct Tracked<ODE> {
    ode: ODE,
    evaluations: u32,
}

impl<ODE> Tracked<ODE> {
    fn new(f: ODE) -> Self {
        Self {
            ode: f,
            evaluations: 0,
        }
    }
}

impl<V, ODE: FirstOrderODE<V>> FirstOrderODE<V> for Tracked<ODE> {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, dy: &mut V) -> Result<(), EvalFailed> {
        self.ode.eval(t, y, dy)?;
        self.evaluations += 1;
        Ok(())
    }
}

impl<V, ODE: SecondOrderODE<V>> SecondOrderODE<V> for Tracked<ODE> {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self.ode.eval(t, y, ddy)?;
        self.evaluations += 1;
        Ok(())
    }
}

impl<V, ODE: SecondOrderODEGeneral<V>> SecondOrderODEGeneral<V> for Tracked<ODE> {
    #[inline]
    fn eval(&mut self, t: f64, y: &V, dy: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self.ode.eval(t, y, dy, ddy)?;
        self.evaluations += 1;
        Ok(())
    }
}

impl<P, E, A, ODE> WPFramework<P, E, A>
where
    P: Problem<ODE = Tracked<ODE>>,
{
    fn new(problem: P, exact: E, absolute_error: A) -> Self {
        Self {
            problem,
            exact,
            absolute_error,
        }
    }

    #[inline]
    fn compute<S>(&self, fac: impl Fn(i32, i32) -> S) -> Vec<(f64, u32)>
    where
        P: Clone,
        E: Fn(f64) -> P::State,
        A: Fn(&P::State, &P::State) -> f64,
        S: Method<P>,
        S::Integrator: IntegratorState + Integrator<P>,
    {
        let n = 16;
        (0..n)
            .rev()
            .map(|i| {
                let method = fac(i, n);
                let mut integrator = method.integrate(self.problem.clone());

                let mut error = 0f64;
                loop {
                    let step_result = integrator.step();

                    let time = integrator.problem.as_ref().time;
                    let exact = (self.exact)(time);
                    error = error.max(
                        (self.absolute_error)(&integrator.problem.as_ref().state, &exact) * 1e3,
                    );
                    match step_result {
                        Ok(()) => {}
                        Err(StepError::BoundReached) => break,
                        Err(e) => {
                            panic!("Integration failed for {}: {e}", std::any::type_name::<S>())
                        }
                    }
                    if time >= integrator.problem.as_ref().bound {
                        break;
                    }
                }

                (error, integrator.problem.as_ref().ode.evaluations)
            })
            // Ignore large errors that don't fit in the plot
            .filter(|&(error, evaluations)| error > MIN_ERROR && evaluations < MAX_EVALS)
            // Stop once the data doesn't fit in the plot
            .take_while(|&(error, evaluations)| error < MAX_ERROR && evaluations > MIN_EVALS)
            .collect::<Vec<_>>()
    }
}

#[derive(Clone, Copy)]
pub struct Tolerance<T, V> {
    pub absolute: T,
    pub relative: T,
    prev_state: V,
}

impl<T, V> Tolerance<T, V> {
    #[inline]
    pub fn new(absolute: T, relative: T, prev_state: V) -> Self {
        Self {
            absolute,
            relative,
            prev_state,
        }
    }

    #[inline]
    pub fn absolute(absolute: T) -> Self
    where
        T: Default,
        V: Default,
    {
        Self {
            absolute,
            relative: T::default(),
            prev_state: V::default(),
        }
    }

    #[inline]
    pub fn relative(relative: T, initial_state: V) -> Self
    where
        T: Default,
    {
        Self {
            absolute: T::default(),
            relative,
            prev_state: initial_state,
        }
    }
}

impl ErrorCriterion<StateVector> for Tolerance<StateVector, StateVector> {
    #[inline]
    fn err_over_tol(&mut self, state: &StateVector, error: &StateVector) -> f64 {
        let max = state.abs().max(&self.prev_state.abs());
        let tol = self.absolute + self.relative.component_mul(&max);
        self.prev_state = *state;

        error.component_div(&tol).abs().max_element()
    }
}

impl ErrorCriterion<[StateVector; 1]> for Tolerance<f64, [StateVector; 1]> {
    #[inline]
    fn err_over_tol(&mut self, [state]: &[StateVector; 1], [error]: &[StateVector; 1]) -> f64 {
        let prev_state = self.prev_state[0];
        self.prev_state[0] = *state;
        Tolerance::new(
            StateVector::splat(DVec3::splat(self.absolute)),
            StateVector::splat(DVec3::splat(self.relative)),
            prev_state,
        )
        .err_over_tol(state, error)
    }
}

impl ErrorCriterion<SecondOrderState<[DVec3; 1]>> for Tolerance<f64, SecondOrderState<[DVec3; 1]>> {
    #[inline]
    fn err_over_tol(
        &mut self,
        state: &SecondOrderState<[DVec3; 1]>,
        error: &SecondOrderState<[DVec3; 1]>,
    ) -> f64 {
        let prev_state = StateVector::new(self.prev_state.y[0], self.prev_state.dy[0]);
        self.prev_state = *state;
        Tolerance::new(
            StateVector::splat(DVec3::splat(self.absolute)),
            StateVector::splat(DVec3::splat(self.relative)),
            prev_state,
        )
        .err_over_tol(
            &StateVector::new(state.y[0], state.dy[0]),
            &StateVector::new(error.y[0], error.dy[0]),
        )
    }
}
