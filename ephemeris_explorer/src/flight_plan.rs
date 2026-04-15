use crate::{
    dynamics::{
        AbsTol, Bodies, ConstantThrust, CubicHermiteSpline, PredictionTrajectory, ReferenceFrame,
        SpacecraftPropagator, SpacecraftPropagatorSoiDetection, SpacecraftTrajectory, StateVector,
        Timeline, Trajectory,
    },
    prediction::{
        ComputePrediction, InPredictionWith, PredictionPropagator, PredictionSystems,
        Synchronisation,
    },
};

use bevy::math::DVec3;
use bevy::prelude::*;
use ephemeris::BoundedTrajectory;
use ftime::{Duration, Epoch};
use integration::AdaptiveMethodParams;

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
    pub fn is_active(&self) -> bool {
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
    pub fn reference_frame(&self) -> ReferenceFrame {
        match self.frame {
            BurnFrame::Relative => ReferenceFrame::relative(self.reference),
            BurnFrame::Inertial => ReferenceFrame::inertial(),
        }
    }

    #[inline]
    pub fn overlaps_with(&self, other: &Burn) -> bool {
        self.enabled && other.enabled && self.start < other.end() && self.end() > other.start
    }

    #[inline]
    pub fn to_constant_thurst(&self) -> (Epoch, Epoch, ConstantThrust) {
        (
            self.start,
            self.end(),
            ConstantThrust::new(self.acceleration, self.reference_frame()),
        )
    }
}

macro_rules! integration_methods {
    ($($variant:ident => $display:expr),+ $(,)?) => {
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub enum IntegrationMethod {
            $($variant),+
        }

        impl IntegrationMethod {
            #[inline]
            pub fn values() -> &'static [Self] {
                &[$(IntegrationMethod::$variant),+]
            }

            #[inline]
            pub fn as_propagator(
                &self,
                time: Epoch,
                sv: StateVector,
                parameters: AdaptiveMethodParams<f64, AbsTol, f64>,
                context: Bodies,
                timeline: Timeline,
            ) -> SpacecraftPropagator {
                match self {
                    $(IntegrationMethod::$variant => SpacecraftPropagator::$variant(
                        SpacecraftPropagatorSoiDetection::new(time, sv, parameters, context, timeline)
                    )),+
                }
            }
        }

        impl PartialEq<SpacecraftPropagator> for IntegrationMethod {
            #[inline]
            fn eq(&self, propagator: &SpacecraftPropagator) -> bool {
                match propagator {
                    $(SpacecraftPropagator::$variant(..) => matches!(self, IntegrationMethod::$variant),)+
                }
            }
        }

        impl std::fmt::Display for IntegrationMethod {
            #[inline]
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                match self {
                    $(IntegrationMethod::$variant => write!(f, $display)),+
                }
            }
        }
    };

}

integration_methods! {
    CashKarp45 => "Cash-Karp 4(5)",
    DormandPrince54 => "Dormand-Prince 5(4)",
    DormandPrince87 => "Dormand-Prince 8(7)",
    Fehlberg45 => "Fehlberg 4(5)",
    Tsitouras75 => "Tsitouras 7(5)",
    Verner87 => "Verner 8(7)",
    Verner98 => "Verner 9(8)",
    Fine45 => "Fine 4(5)",
}

// TODO: Add context to flight plan.
#[derive(Component, Clone)]
pub struct FlightPlan {
    pub method: IntegrationMethod,
    pub parameters: AdaptiveMethodParams<f64, AbsTol, f64>,
    pub synchronisation: Synchronisation,
    pub end: Epoch,
    pub burns: indexmap::IndexMap<uuid::Uuid, Burn>,
    pub context: Bodies,
}

impl FlightPlan {
    #[inline]
    pub fn new(
        end: Epoch,
        parameters: AdaptiveMethodParams<f64, AbsTol, f64>,
        burns: Vec<Burn>,
        context: Bodies,
    ) -> Self {
        Self {
            method: IntegrationMethod::Verner87,
            parameters,
            synchronisation: Synchronisation::hertz(1000),
            end,
            burns: burns
                .into_iter()
                .map(|burn| (uuid::Uuid::new_v4(), burn))
                .collect(),
            context,
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
            .filter(|burn| burn.is_active())
            .map(Burn::delta_v)
            .sum()
    }

    #[inline]
    pub fn generate_timeline(&self) -> Timeline {
        Timeline::new(
            self.burns
                .values()
                .filter(|burn| burn.is_active())
                .map(|burn| burn.to_constant_thurst())
                .collect(),
        )
    }

    #[inline]
    pub fn build_propagator_at(&self, time: Epoch, sv: StateVector) -> SpacecraftPropagator {
        self.method.as_propagator(
            time,
            sv,
            self.parameters,
            self.context.clone(),
            self.generate_timeline(),
        )
    }

    #[inline]
    pub fn restart_propagator(
        &self,
        previous: &SpacecraftPropagator,
        trajectory: &CubicHermiteSpline,
    ) -> Option<SpacecraftPropagator> {
        let timeline = self.generate_timeline();

        // This determines the epoch from which we restart the propagation.
        //
        // Propagation is divided into segments that span from one manoeuvre event to the next.
        // Propagation can only be restarted at the start of a segment.
        //
        // We want to restart from the latest possible time that is unaffected by the changes.
        // Specifically:
        // 1. If an integration parameter changed, we recompute the entire propagation.
        // 2. Otherwise, we find the first point where the new timeline differs from the old one. We
        //    restart from the last common event before this difference.
        // 3. If no change was previously detected, we restart from the last event preceding the end
        //    of the flight plan.
        let restart_epoch = if &self.method != previous
            || &self.parameters.tol != previous.tolerance()
            || self.parameters.n_max as usize != previous.max_iterations()
        {
            trajectory.start()
        } else {
            timeline
                .divergence_time_before(previous.timeline(), self.end.min(trajectory.end()))
                .max(trajectory.start())
        };

        // Try to get the state vector at the restart epoch and return the new propagator
        trajectory.get(restart_epoch).map(|&restart_sv| {
            self.method.as_propagator(
                restart_epoch,
                restart_sv,
                self.parameters,
                previous.context().clone(),
                timeline,
            )
        })
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
    mut query: Query<(
        &mut FlightPlan,
        &PredictionPropagator<SpacecraftTrajectory>,
        &Trajectory,
    )>,
) {
    let entity = trigger.event_target();
    let Ok((mut flight_plan, propagator, trajectory)) = query.get_mut(entity) else {
        return;
    };
    let PredictionTrajectory::CubicHermiteSpline(trajectory) = &*trajectory.read() else {
        unreachable!()
    };

    if !propagator.context().is_valid_at(trajectory.start()) {
        return;
    };

    flight_plan.compute_overlaps();
    let Some(propagator) = flight_plan.restart_propagator(propagator, trajectory) else {
        error!("could not restart propagation after the flight plan changed");
        return;
    };
    let restart_epoch = propagator.time();

    commands.trigger(
        ComputePrediction::<SpacecraftTrajectory>::extend(
            entity,
            flight_plan.end - restart_epoch,
            flight_plan.synchronisation,
        )
        .with_propagator(propagator),
    );
}

#[expect(clippy::type_complexity)]
fn trigger_on_trajectory_updates(
    mut commands: Commands,
    query: Query<(), (Changed<Trajectory>, With<FlightPlanDependency>)>,
    query_flight_plan: Query<
        (
            Entity,
            &Trajectory,
            &FlightPlan,
            &PredictionPropagator<SpacecraftTrajectory>,
        ),
        Without<InPredictionWith<SpacecraftTrajectory>>,
    >,
) {
    if query.is_empty() {
        return;
    }

    for (entity, traj, flight_plan, propagator) in query_flight_plan.iter() {
        // Only trigger if the trajectory hasn't previously reached the end of the flight plan and
        // if the context allows for the prediction to start being computed.
        if traj.end() < flight_plan.end && propagator.context().is_valid_at(traj.start()) {
            // TODO: Remove this guard once we can restart the prediction from the end.
            if !propagator.context().is_valid_at(flight_plan.end) {
                continue;
            }

            commands.trigger(FlightPlanChanged(entity));
        }
    }
}
