use crate::{
    dynamics::{
        ConstantThrust, CubicHermiteSplineSamples, DEFAULT_ADAPTIVE_PARAMS, ReferenceFrame,
        SpacecraftPropagatorSoiDetection, SpacecraftTrajectory, Timeline, Trajectory,
    },
    prediction::{
        ComputePrediction, PredictingWith, PredictionContext, PredictionSystems, Synchronisation,
    },
};

use bevy::math::DVec3;
use bevy::prelude::*;
use ephemeris::BoundedTrajectory;
use ftime::{Duration, Epoch};
use integration::AdaptiveMethodParams;

type SpacecraftPredictionContext = PredictionContext<SpacecraftTrajectory>;

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Burn {
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

    #[inline]
    pub fn to_constant_thurst(&self, query: &Query<&Trajectory>) -> (Epoch, Epoch, ConstantThrust) {
        (
            self.start,
            self.end(),
            ConstantThrust::new(
                self.acceleration / 1e3,
                self.try_reference_frame(query)
                    .expect("invalid burn reference entity"),
            ),
        )
    }
}

#[derive(Component, Clone)]
pub struct FlightPlan {
    pub end: Epoch,
    pub max_iterations: usize,
    pub burns: indexmap::IndexMap<uuid::Uuid, Burn>,
}

impl FlightPlan {
    pub fn new(end: Epoch, max_iterations: usize, burns: Vec<Burn>) -> Self {
        Self {
            end,
            max_iterations,
            burns: burns
                .into_iter()
                .map(|burn| (uuid::Uuid::new_v4(), burn))
                .collect(),
        }
    }

    #[inline]
    pub fn compute_overlaps(&mut self) {
        for i in 0..self.burns.len() {
            self.burns[i].overlaps = self
                .burns
                .values()
                .enumerate()
                .any(|(j, burn)| i != j && burn.overlaps_with(&self.burns[i]));
        }
    }

    #[inline]
    pub fn total_delta_v(&self) -> f64 {
        self.burns
            .values()
            .filter(|burn| burn.active())
            .map(Burn::delta_v)
            .sum()
    }

    #[inline]
    pub fn generate_timeline(&self, query: &Query<&Trajectory>) -> Timeline {
        Timeline::new(
            self.burns
                .values()
                .filter(|burn| burn.active())
                .map(|burn| burn.to_constant_thurst(query))
                .collect(),
        )
    }
}

#[derive(Component)]
pub struct FlightPlanDependency;

// To make change detection optional but immediate, we use this event.
#[derive(Clone, Copy, EntityEvent)]
pub struct FlightPlanChanged(pub Entity);

#[derive(Default)]
pub struct FlightPlanPlugin;

impl Plugin for FlightPlanPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(apply_flight_plan).add_systems(
            PreUpdate,
            trigger_on_trajectory_updates.after(PredictionSystems),
        );
    }
}

fn apply_flight_plan(
    trigger: On<FlightPlanChanged>,
    mut commands: Commands,
    mut query: Query<(&mut FlightPlan, &SpacecraftPredictionContext, &Trajectory)>,
    query_trajectory: Query<&Trajectory>,
) {
    let entity = trigger.event_target();
    let Ok((mut flight_plan, PredictionContext { propagator, .. }, trajectory)) =
        query.get_mut(entity)
    else {
        return;
    };
    let trajectory = trajectory.read();
    let trajectory = trajectory
        .as_any()
        .downcast_ref::<CubicHermiteSplineSamples>()
        .unwrap();

    if !propagator.environment().is_valid_at(trajectory.start()) {
        return;
    };

    flight_plan.compute_overlaps();
    let timeline = flight_plan.generate_timeline(&query_trajectory);

    // This determines the epoch from which we restart the propagation.
    //
    // Propagation is divided into segments that span from one manoeuvre event to the next.
    // Propagation can only be restarted at the start of a segment.
    //
    // We want to restart from the latest possible time that is unaffected by the changes.
    // Specifically:
    // 1. If `max_iterations` changed, we recompute the entire propagation.
    // 2. Otherwise, we find the first point where the new timeline differs from the old one. We
    //    restart from the last common event before this difference.
    // 3. If no change was previously detected, we restart from the last event preceding the end of
    //    the flight plan (this ensures the prediction reaches the flight plan end).
    let restart_epoch = {
        if propagator.max_iterations() != flight_plan.max_iterations {
            trajectory.start()
        } else {
            timeline
                .divergence_time_before(
                    propagator.timeline(),
                    flight_plan.end.min(trajectory.end()),
                )
                .max(trajectory.start())
        }
    };

    let Some(&restart_sv) = trajectory.get(restart_epoch) else {
        error!("something went wrong when trying to compute the flight plan from {restart_epoch}",);
        return;
    };
    debug!("restarting propagation from {}", restart_epoch);

    let propagator = SpacecraftPropagatorSoiDetection::new(
        restart_epoch,
        restart_sv,
        AdaptiveMethodParams {
            n_max: flight_plan.max_iterations as _,
            ..DEFAULT_ADAPTIVE_PARAMS
        },
        propagator.environment().clone(),
        timeline,
    );

    commands.trigger(ComputePrediction::<SpacecraftTrajectory>::extend(
        entity,
        propagator,
        flight_plan.end - restart_epoch,
        Synchronisation::hertz(1000),
    ));
}

fn trigger_on_trajectory_updates(
    mut commands: Commands,
    query: Query<(), (Changed<Trajectory>, With<FlightPlanDependency>)>,
    query_flight_plan: Query<
        (
            Entity,
            &Trajectory,
            &FlightPlan,
            &SpacecraftPredictionContext,
        ),
        Without<PredictingWith<SpacecraftTrajectory>>,
    >,
) {
    if query.is_empty() {
        return;
    }

    for (entity, traj, flight_plan, PredictionContext { propagator, .. }) in
        query_flight_plan.iter()
    {
        // Only trigger if the trajectory hasn't previously reached the end of the flight plan and
        // if the context allows for the prediction to start being computed.
        if traj.end() < flight_plan.end && propagator.environment().is_valid_at(traj.start()) {
            // TODO: Remove this guard once we can restart the prediction from the end.
            if !propagator.environment().is_valid_at(flight_plan.end) {
                continue;
            }

            commands.trigger(FlightPlanChanged(entity));
        }
    }
}
