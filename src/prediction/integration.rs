use bevy::math::DVec3;
use hifitime::Duration;

pub trait IntegrationState {
    fn velocity(&mut self) -> &mut DVec3;

    fn position(&mut self) -> &mut DVec3;

    fn step(&mut self, delta: Duration);
}

#[derive(Default, Clone, Copy)]
#[expect(clippy::upper_case_acronyms)]
pub struct PEFRL;

impl PEFRL {
    const LAMBDA: f64 = -0.2123418310626054;

    const XI: f64 = 0.1786178958448091;

    const CHI: f64 = -0.0662645826698185;

    pub fn integrate<S, F>(delta: Duration, states: &mut [S], accelerations_into: F)
    where
        S: IntegrationState,
        F: Fn(&mut Vec<DVec3>, &[S], f64),
    {
        let dt = delta.to_seconds();
        let dt_half = dt / 2.0;
        let h1 = (1.0 - 2.0 * Self::LAMBDA) * dt_half;
        let h2 = Self::LAMBDA * dt;

        for state in states.iter_mut() {
            let velocity = *state.velocity();
            *state.position() += Self::XI * dt * velocity;
        }

        let mut acc = Vec::with_capacity(states.len());
        accelerations_into(&mut acc, states, Self::XI * dt);
        for (state, acceleration) in states.iter_mut().zip(acc.iter()) {
            *state.velocity() += h1 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::CHI * dt * velocity;
        }

        acc.clear();
        accelerations_into(&mut acc, states, (Self::XI + Self::CHI) * dt);
        for (state, acceleration) in states.iter_mut().zip(acc.iter()) {
            *state.velocity() += h2 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += (1.0 - 2.0 * (Self::CHI + Self::XI)) * dt * velocity;
        }

        acc.clear();
        accelerations_into(&mut acc, states, (1.0 - Self::XI - Self::CHI) * dt);
        for (state, acceleration) in states.iter_mut().zip(acc.iter()) {
            *state.velocity() += h2 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::CHI * dt * velocity;
        }

        acc.clear();
        accelerations_into(&mut acc, states, (1.0 - Self::XI) * dt);
        for (state, acceleration) in states.iter_mut().zip(acc.iter()) {
            *state.velocity() += h1 * *acceleration;
            let velocity = *state.velocity();
            *state.position() += Self::XI * dt * velocity;
    
            state.step(delta);
        }
    }
}
