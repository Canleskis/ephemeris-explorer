# integration

A Rust library for building and using numerical ODE integrators through a trait-driven API. The focus is on flexibility and composability, as such some defaults usually expected from numerical integration libraries might be missing.

It provides generic implementations for:

- Runge-Kutta (RK)
- Runge-Kutta-Nystrom (RKN)
- Generalized Runge-Kutta-Nystrom (RKGN)
- Symplectic Runge-Kutta-Nystrom (SRKN)
- First-order and second-order explicit linear multistep methods

## API Concepts

- The `Problem` trait represents an integration problem by providing access to an `ODEProblem`, which stores the current time, integration bound, state, and ODE.
- The `Method` trait defines how a numerical method initializes its corresponding integrator for a given problem.
- The `Integrator` trait defines the stepping interface.
- The `Integration` struct is a convenience wrapper that owns both a problem and an integrator, exposing more ergonomic stepping methods.
- The `FixedMethodParams` and `AdaptiveMethodParams` structs provide shared configuration for fixed-step and adaptive-step methods.

## Examples

### Fixed-Step RK4

```rust
use integration::prelude::*;

// y' = -y, y(0) = 1
let ode = |_: f64, y: &[f64; 1], dy: &mut [f64; 1]| {
    dy[0] = -y[0];
    Ok(())
};

let problem = ODEProblem::new(0.0_f64, 5.0_f64, [1.0_f64], ode);
let expected_y = (-5.0_f64).exp();

let method = RK4::new(FixedMethodParams::new(0.001));
let mut integration = method.integrate(problem);

let (t_end, [y_end]) = integration.solve()?;
println!("t = {t_end}, y = {y_end}");
println!("Expected y = {expected_y}");

assert!((y_end - expected_y).abs() < 1e-6);
```

### Adaptive Dormand-Prince 5(4)

```rust
use integration::prelude::*;

// y' = -y, y(0) = 1
let ode = |_: f64, y: &[f64; 1], dy: &mut [f64; 1]| {
    dy[0] = -y[0];
    Ok(())
};

// Relative and absolute scalar tolerance model.
let tol = |atol, rtol| {
    move |state: &[f64; 1], err: &[f64; 1]| err[0].abs() / (atol + rtol * state[0].abs())
};

let problem = ODEProblem::new(0.0_f64, 5.0_f64, [1.0_f64], ode);
let expected_y = (-5.0_f64).exp();

let params = AdaptiveMethodParams::new(tol(1e-8, 1e-10), 10_000)
    .h_init(1e-3)
    .h_max(0.2);
let method = DormandPrince54::new(params);
let mut integration = method.integrate(problem);
let (t_end, [y_end]) = integration.solve()?;

println!("t = {t_end}, y = {y_end}");
println!("Expected y = {expected_y}");

assert!((y_end - expected_y).abs() < 1e-6);
```

## Included Methods

Aliases exposed under `integration::methods` include:

- `RK4`
- `CashKarp45`
- `DormandPrince54`
- `DormandPrince87`
- `Fehlberg45`
- `Tsitouras75`
- `Verner87`
- `Verner98`
- `Fine45`
- `Tsitouras75Nystrom`
- `BlanesMoan6B`
- `BlanesMoan11B`
- `BlanesMoan14A`
- `ForestRuth`
- `McLachlanO4`
- `McLachlanSS17`
- `Pefrl`
- `Ruth`
- `AdamsBashforth2`
- `AdamsBashforth3`
- `AdamsBashforth4`
- `AdamsBashforth5`
- `AdamsBashforth6`
- `QuinlanTremaine12`
- `Stormer13`

For low-level composition and custom coefficients, see the `methods::coeffs`, `runge_kutta`, and `multistep` modules.

## License

This project is licensed under either of Apache License, Version 2.0 or MIT license, at your option.
