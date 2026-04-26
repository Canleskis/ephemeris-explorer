use crate::{
    MainState,
    dynamics::Trajectory,
    simulation::{SimulationTime, SimulationTimeSystems},
};

use bevy::prelude::*;
use ephemeris::BoundedTrajectory;

#[derive(Clone, Copy, Component)]
pub struct OutOfBounds;

pub struct OutOfBoundsPlugin;

impl Plugin for OutOfBoundsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            First,
            find_out_of_bounds
                .after(SimulationTimeSystems)
                .run_if(in_state(MainState::Running)),
        )
        .add_observer(on_add_out_of_bounds)
        .add_observer(on_remove_out_of_bounds);
    }
}

fn find_out_of_bounds(
    sim_time: Res<SimulationTime>,
    query: Query<(Entity, &Trajectory, Has<OutOfBounds>)>,
    mut commands: Commands,
) {
    for (entity, trajectory, has_out_of_bounds) in query {
        let is_in_bounds = trajectory.contains(sim_time.current());
        if is_in_bounds && has_out_of_bounds {
            commands.entity(entity).remove::<OutOfBounds>();
        } else if !is_in_bounds && !has_out_of_bounds {
            commands.entity(entity).insert(OutOfBounds);
        }
    }
}

fn on_add_out_of_bounds(add: On<Add, OutOfBounds>, mut query: Query<&mut Visibility>) {
    if let Ok(mut visiblity) = query.get_mut(add.entity) {
        visiblity.set_if_neq(Visibility::Hidden);
    }
}

fn on_remove_out_of_bounds(remove: On<Remove, OutOfBounds>, mut query: Query<&mut Visibility>) {
    if let Ok(mut visiblity) = query.get_mut(remove.entity) {
        visiblity.set_if_neq(Visibility::Inherited);
    }
}
