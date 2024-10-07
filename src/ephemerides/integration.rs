use bevy::math::DVec3;
use hifitime::Duration;

pub trait IntegrationState {
    fn velocity(&mut self) -> &mut DVec3;

    fn position(&mut self) -> &mut DVec3;

    fn step(&mut self, delta: Duration);
}

#[derive(Default, Clone, Copy)]
pub struct SymplecticEuler;

#[derive(Default, Clone, Copy)]
pub struct Perfl;

impl Perfl {
    const LAMBDA: f64 = -0.2123418310626054;

    const XI: f64 = 0.1786178958448091;

    const CHI: f64 = -0.0662645826698185;

    #[expect(clippy::needless_range_loop)]
    pub fn integrate<S, F>(delta: Duration, states: &mut [S], evaluate: F)
    where
        S: IntegrationState,
        F: Fn(&S, &[S], f64) -> DVec3,
    {
        let dt = delta.to_seconds();
        let dt_half = dt / 2.0;
        let h1 = (1.0 - 2.0 * Self::LAMBDA) * dt_half;
        let h2 = Self::LAMBDA * dt;

        for i in 0..states.len() {
            let velocity = *states[i].velocity();
            *states[i].position() += Self::XI * dt * velocity;
        }

        for i in 0..states.len() {
            let acc = evaluate(&states[i], states, h1);
            *states[i].velocity() += h1 * acc;
        }

        for i in 0..states.len() {
            let velocity = *states[i].velocity();
            *states[i].position() += Self::CHI * dt * velocity;
        }

        for i in 0..states.len() {
            let acc = evaluate(&states[i], states, h2);
            *states[i].velocity() += h2 * acc;
        }

        for i in 0..states.len() {
            let velocity = *states[i].velocity();
            *states[i].position() += (1.0 - 2.0 * (Self::CHI + Self::XI)) * dt * velocity;
        }

        for i in 0..states.len() {
            let acc = evaluate(&states[i], states, h2);
            *states[i].velocity() += h2 * acc;
        }

        for i in 0..states.len() {
            let velocity = *states[i].velocity();
            *states[i].position() += Self::CHI * dt * velocity;
        }

        for i in 0..states.len() {
            let acc = evaluate(&states[i], states, h1);
            *states[i].velocity() += h1 * acc;
        }

        for i in 0..states.len() {
            let velocity = *states[i].velocity();
            *states[i].position() += Self::XI * dt * velocity;

            states[i].step(delta);
        }
    }
}
