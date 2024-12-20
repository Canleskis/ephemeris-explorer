use crate::prediction::{
    ConstantAcceleration, DiscreteStates, DiscreteStatesBuilder, ExtendPredictionEvent,
    ReferenceFrame, Trajectory, TrajectoryData,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BurnFrame {
    Frenet,
    Cartesian,
}

impl std::fmt::Display for BurnFrame {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            BurnFrame::Frenet => write!(f, "Frenet"),
            BurnFrame::Cartesian => write!(f, "Cartesian"),
        }
    }
}

impl BurnFrame {
    pub fn values() -> [Self; 2] {
        [BurnFrame::Frenet, BurnFrame::Cartesian]
    }

    pub fn as_reference_frame(&self, reference: Entity) -> ReferenceFrame {
        match self {
            BurnFrame::Frenet => ReferenceFrame::Frenet(reference),
            BurnFrame::Cartesian => ReferenceFrame::Cartesian,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Burn {
    pub id: uuid::Uuid,
    pub start: Epoch,
    pub duration: Duration,
    pub acceleration: DVec3,
    pub reference: Entity,
    pub frame: BurnFrame,
    pub enabled: bool,
    pub overlaps: bool,
}

impl Burn {
    pub fn new(start: Epoch, reference: Entity) -> Self {
        Self {
            id: uuid::Uuid::new_v4(),
            start,
            duration: Duration::from_seconds(60.0),
            acceleration: DVec3::ZERO,
            reference,
            frame: BurnFrame::Frenet,
            enabled: true,
            overlaps: false,
        }
    }

    #[expect(unused)]
    pub fn with(
        start: Epoch,
        duration: Duration,
        acceleration: DVec3,
        reference: Entity,
        reference_frame: BurnFrame,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4(),
            start,
            duration,
            acceleration,
            reference,
            frame: reference_frame,
            enabled: true,
            overlaps: false,
        }
    }

    pub fn end(&self) -> Epoch {
        self.start + self.duration
    }

    pub fn frame(&self) -> ReferenceFrame {
        self.frame.as_reference_frame(self.reference)
    }

    pub fn overlaps_with(&self, other: &Burn) -> bool {
        self.enabled && other.enabled && self.start < other.end() && self.end() > other.start
    }
}

#[derive(Component, Clone)]
pub struct FlightPlan {
    pub end: Epoch,
    pub burns: Vec<Burn>,
}

impl FlightPlan {
    pub fn new(end: Epoch) -> Self {
        Self {
            end,
            burns: Vec::new(),
        }
    }

    #[expect(unused)]
    pub fn with(end: Epoch, burns: Vec<Burn>) -> Self {
        Self { end, burns }
    }

    #[inline]
    pub fn compute_overlaps(&mut self) {
        for i in 0..self.burns.len() {
            self.burns[i].overlaps = self
                .burns
                .iter()
                .enumerate()
                .any(|(j, burn)| i != j && burn.overlaps_with(&self.burns[i]));
        }
    }
}

// To make change detection optional and immediate, we use this event and an observer.
#[derive(Clone, Copy, Event)]
pub struct FlightPlanChanged;

pub struct FlightPlanPlugin;

impl Plugin for FlightPlanPlugin {
    fn build(&self, app: &mut App) {
        app.observe(compute_flight_plan);
    }
}

fn first_missing(b1: &[Burn], b2: &[Burn]) -> Option<Epoch> {
    b1.iter()
        .filter(|previous| !b2.iter().any(|burn| burn.id == previous.id))
        .map(|previous| previous.start)
        .min()
        .map(|t| t - Duration::EPSILON)
}

fn compute_flight_plan(
    trigger: Trigger<FlightPlanChanged>,
    mut commands: Commands,
    mut query: Query<(&mut FlightPlan, &mut DiscreteStatesBuilder, &Trajectory)>,
    mut previous: Local<Option<FlightPlan>>,
) {
    let entity = trigger.entity();
    let Ok((mut flight_plan, mut builder, trajectory)) = query.get_mut(entity) else {
        return;
    };
    let trajectory = trajectory.downcast_ref::<DiscreteStates>();
    let min = trajectory.start();
    let max = trajectory.end();

    flight_plan.compute_overlaps();

    let previous = previous.replace(flight_plan.clone());
    // Restart from the last point which would result in the same integrator steps.
    let last_valid = previous
        .map(|previous| {
            let end = (flight_plan.end != previous.end).then_some(flight_plan.end);
            let added = first_missing(&flight_plan.burns, &previous.burns);
            let removed = first_missing(&previous.burns, &flight_plan.burns);
            let changes = flight_plan
                .burns
                .iter()
                .zip(previous.burns.iter())
                .filter_map(|(burn, previous)| {
                    let start = burn.start.min(previous.start);
                    if (burn.start != previous.start
                        || burn.enabled != previous.enabled
                        || burn.overlaps != previous.overlaps)
                        && (burn.enabled || previous.enabled)
                        && (!burn.overlaps || !previous.overlaps)
                        && (burn.start >= min || previous.start >= min)
                        && (burn.start <= max || previous.start <= max)
                    {
                        Some(start - Duration::EPSILON)
                    } else if (burn.duration != previous.duration
                        || burn.acceleration != previous.acceleration
                        || burn.reference != previous.reference
                        || burn.frame != previous.frame)
                        && burn.enabled
                        && !burn.overlaps
                    {
                        Some(start)
                    } else {
                        None
                    }
                })
                .min();

            [end, added, removed, changes].into_iter().flatten().max()
        })
        .unwrap_or(Some(min));

    let Some(last_valid) = last_valid else {
        return;
    };
    // TODO: We could theoretically restart from the point before the new start time in some
    // cases, but the integrator needs the previous error of that point to calculate the new
    // step size, which we don't store. So for now we just restart from the previous integrator
    // reset, which is the end of the previous burn.
    // We could fix this by changing how the integrator computes the error.
    // Note: Each key in the manoeuvres map is an integrator reset.
    let restart = builder
        .manoeuvres()
        .range(..=last_valid)
        .next_back()
        .map_or(min, |(t, _)| *t);

    // let last_valid = trigger.event().start;

    // let restart = flight_plan
    //     .burns
    //     .iter()
    //     .rev()
    //     .filter(|burn| burn.enabled && !burn.overlaps)
    //     .flat_map(|burn| [burn.end(), burn.start])
    //     .filter(|t| *t <= max)
    //     .filter(|t| builder.manoeuvres().contains_key(t))
    //     .find(|t| *t <= last_valid)
    //     .unwrap_or(min);

    // println!("Received valid: {}", last_valid);
    // println!("Received restart: {}", restart);

    let Some(&restart_sv) = trajectory.get(restart) else {
        bevy::log::error!("Someting went wrong when trying to restart the prediction");
        return;
    };

    // println!("Restarting prediction from {}", restart);

    builder.clear_burns();
    builder.set_initial_state(restart, restart_sv);
    for burn in &flight_plan.burns {
        if !burn.enabled || burn.overlaps {
            continue;
        }
        builder.insert_manoeuvre(ConstantAcceleration::new(
            burn.start,
            burn.duration,
            burn.acceleration / 1e3,
            burn.frame(),
        ));
    }

    if flight_plan.end >= restart {
        commands.trigger(ExtendPredictionEvent::<DiscreteStatesBuilder>::with(
            [entity],
            Duration::EPSILON.max(flight_plan.end - restart),
            100,
        ));
    }
}
