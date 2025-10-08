use crate::{
    dynamics::{ConstantThrust, CubicHermiteSplineSamples, ReferenceFrame, SpacecraftPropagator},
    prediction::{ExtendPredictionEvent, PredictionContext, Trajectory},
};

use bevy::math::DVec3;
use bevy::prelude::*;
use ephemeris::BoundedTrajectory;
use ftime::{Duration, Epoch};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BurnFrame {
    Relative,
    Inertial,
}

impl std::fmt::Display for BurnFrame {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            BurnFrame::Relative => write!(f, "Relative"),
            BurnFrame::Inertial => write!(f, "Inertial"),
        }
    }
}

impl BurnFrame {
    #[inline]
    pub fn values() -> [Self; 2] {
        [BurnFrame::Relative, BurnFrame::Inertial]
    }
}

#[derive(Clone, Copy, Debug)]
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
    #[inline]
    pub fn new(start: Epoch, frame: BurnFrame, reference: Entity) -> Self {
        Self {
            id: uuid::Uuid::new_v4(),
            start,
            duration: Duration::from_seconds(60.0),
            acceleration: DVec3::ZERO,
            reference,
            frame,
            enabled: true,
            overlaps: false,
        }
    }

    #[inline]
    pub fn with(
        start: Epoch,
        duration: Duration,
        acceleration: DVec3,
        reference: Entity,
        frame: BurnFrame,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4(),
            start,
            duration,
            acceleration,
            reference,
            frame,
            enabled: true,
            overlaps: false,
        }
    }

    #[inline]
    pub fn active(&self) -> bool {
        self.enabled && !self.overlaps
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        self.start + self.duration
    }

    #[inline]
    pub fn delta_v(&self) -> f64 {
        self.acceleration.length() * self.duration.as_seconds()
    }

    #[inline]
    pub fn try_reference_frame(&self, query: &Query<&Trajectory>) -> Option<ReferenceFrame> {
        match self.frame {
            BurnFrame::Relative => Some(ReferenceFrame::relative(
                query.get(self.reference).ok()?.clone(),
            )),
            BurnFrame::Inertial => Some(ReferenceFrame::inertial()),
        }
    }

    #[inline]
    pub fn overlaps_with(&self, other: &Burn) -> bool {
        self.enabled && other.enabled && self.start < other.end() && self.end() > other.start
    }
}

#[derive(Clone, Default, Component, Deref, DerefMut)]
struct LastFlightPlan(Option<FlightPlan>);

#[derive(Component, Clone)]
#[require(LastFlightPlan)]
pub struct FlightPlan {
    pub end: Epoch,
    pub max_iterations: usize,
    pub burns: Vec<Burn>,
}

impl FlightPlan {
    pub fn new(end: Epoch, max_iterations: usize, burns: Vec<Burn>) -> Self {
        Self {
            end,
            max_iterations,
            burns,
        }
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

    pub fn total_delta_v(&self) -> f64 {
        self.burns
            .iter()
            .filter(|burn| burn.active())
            .map(Burn::delta_v)
            .sum()
    }
}

// To make change detection optional and immediate, we use this event and an observer.
#[derive(Clone, Copy, Event)]
pub struct FlightPlanChanged;

#[derive(Clone, Copy, Debug)]
pub enum BurnChange {
    SetEnabled(bool),
    SetStart(Epoch),
    SetDuration(Duration),
    SetAcceleration(DVec3),
    SetReferenceFrame(Entity, BurnFrame),
}

#[derive(Clone, Copy, Debug)]
pub enum FlightPlanChange {
    AddBurn(Burn),
    RemoveBurn(uuid::Uuid),
    UpdateBurn(uuid::Uuid, BurnChange),
    SetEnd(Epoch),
    SetMaxIterations(usize),
}

#[derive(Default)]
pub struct FlightPlanPlugin;

impl Plugin for FlightPlanPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(apply_flight_plan);
    }
}

fn first_missing(b1: &[Burn], b2: &[Burn]) -> Option<RestartBound> {
    b1.iter()
        .filter(|burn1| b2.iter().all(|burn2| burn2.id != burn1.id))
        .map(|previous| RestartBound::Excluded(previous.start))
        .min()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RestartBound {
    Excluded(Epoch),
    Included(Epoch),
}

impl std::fmt::Display for RestartBound {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            RestartBound::Excluded(t) => write!(f, "Excluded({t})"),
            RestartBound::Included(t) => write!(f, "Included({t})"),
        }
    }
}

impl PartialOrd for RestartBound {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RestartBound {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match (self, other) {
            (RestartBound::Excluded(t1), RestartBound::Included(t2)) => match t1.cmp(t2) {
                std::cmp::Ordering::Equal => std::cmp::Ordering::Greater,
                ord => ord,
            },
            (RestartBound::Included(t1), RestartBound::Excluded(t2)) => match t1.cmp(t2) {
                std::cmp::Ordering::Equal => std::cmp::Ordering::Less,
                ord => ord,
            },
            (RestartBound::Excluded(t1), RestartBound::Excluded(t2))
            | (RestartBound::Included(t1), RestartBound::Included(t2)) => t1.cmp(t2),
        }
    }
}

fn apply_flight_plan(
    trigger: Trigger<FlightPlanChanged>,
    mut commands: Commands,
    mut query: Query<(
        &mut FlightPlan,
        &mut LastFlightPlan,
        &mut PredictionContext<SpacecraftPropagator>,
        &Trajectory,
    )>,
    query_trajectory: Query<&Trajectory>,
) {
    let entity = trigger.target();
    let Ok((mut flight_plan, mut last, mut prediction, trajectory)) = query.get_mut(entity) else {
        return;
    };
    let trajectory = trajectory.read();
    let trajectory = trajectory
        .as_any()
        .downcast_ref::<CubicHermiteSplineSamples>()
        .unwrap();
    let min = trajectory.start();
    let max = trajectory.end();

    flight_plan.compute_overlaps();

    let last = last.replace(flight_plan.clone());
    // Restart from the last point which would result in the same integrator steps.
    let last_valid = last
        .map(|last| {
            let end =
                (flight_plan.end != last.end).then_some(RestartBound::Included(flight_plan.end));
            let iterations = (flight_plan.max_iterations != last.max_iterations)
                .then_some(RestartBound::Included(min));
            let added = first_missing(&flight_plan.burns, &last.burns);
            let removed = first_missing(&last.burns, &flight_plan.burns);
            let burns = flight_plan
                .burns
                .iter()
                .zip(last.burns.iter())
                .filter_map(|(burn, last)| {
                    let start = burn.start.min(last.start);

                    if (burn.start < min && last.start < min)
                        || (burn.start > max && last.start > max)
                    {
                        None
                    } else if (burn.start != last.start
                        || burn.enabled != last.enabled
                        || burn.overlaps != last.overlaps)
                        && (burn.enabled || last.enabled)
                        && (!burn.overlaps || !last.overlaps)
                    {
                        Some(RestartBound::Excluded(start))
                    } else if (burn.duration != last.duration
                        || burn.acceleration != last.acceleration
                        || burn.reference != last.reference
                        || burn.frame != last.frame)
                        && burn.enabled
                        && !burn.overlaps
                    {
                        Some(RestartBound::Included(start))
                    } else {
                        None
                    }
                })
                .min();

            [end, iterations, added, removed, burns]
                .into_iter()
                .flatten()
                .min()
        })
        .map_or(Some(RestartBound::Included(min)), |last_valid| {
            last_valid.min(Some(RestartBound::Included(max)))
        });

    let Some(last_valid) = last_valid else {
        return;
    };

    let propagator = &mut prediction.propagator;

    let restart_epoch = match last_valid {
        RestartBound::Excluded(t) => propagator.schedule().last_event_at_exclusive(t),
        RestartBound::Included(t) => propagator.schedule().last_event_at(t),
    }
    .time()
    .max(min);

    let Some(&restart_sv) = trajectory.get(restart_epoch) else {
        bevy::log::error!(
            "something went wrong when trying to compute the flight plan from {}",
            restart_epoch
        );
        return;
    };

    propagator.clear_manoeuvres();
    for burn in &flight_plan.burns {
        if !burn.enabled || burn.overlaps || burn.start < min {
            continue;
        }
        propagator.insert_manoeuvre(ConstantThrust::new(
            burn.start,
            burn.duration,
            burn.acceleration / 1e3,
            burn.try_reference_frame(&query_trajectory)
                .expect("invalid burn reference entity"),
        ));
    }
    propagator.set_initial_state(restart_epoch, restart_sv);
    propagator.set_max_iterations(flight_plan.max_iterations);

    if flight_plan.end >= restart_epoch {
        commands.trigger(ExtendPredictionEvent::<SpacecraftPropagator>::with(
            [entity],
            Duration::from_seconds(0.001).max(flight_plan.end - restart_epoch),
            200,
        ));
    }
}
