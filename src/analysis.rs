use crate::{
    hierarchy::{AddOrbit, Orbiting},
    prediction::{Trajectory, TrajectoryData},
    time::SimulationTime,
    MainState,
};
use bevy::math::DVec3;
use bevy::prelude::*;

#[derive(Component)]
pub enum SphereOfInfluence {
    Fixed(f64),
    #[expect(unused)]
    Computed(f64),
}

impl SphereOfInfluence {
    pub fn radius(&self) -> f64 {
        match self {
            Self::Fixed(radius) | Self::Computed(radius) => *radius,
        }
    }

    pub fn approximate(a: f64, m: f64, parent: f64) -> Self {
        Self::Fixed(a * (m / parent).powf(2.0 / 5.0))
    }
}

#[derive(Default)]
pub struct OrbitalAnalysisPlugin;

impl Plugin for OrbitalAnalysisPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            sphere_of_influence_to_hierarchy.run_if(in_state(MainState::Running)),
        );
    }
}

pub fn under_soi<'a, I>(iter: I, position: DVec3) -> Option<Entity>
where
    I: IntoIterator<Item = (Entity, DVec3, &'a SphereOfInfluence)>,
{
    iter.into_iter()
        .map(|(soi_entity, soi_position, soi)| {
            (soi_entity, position.distance(soi_position), soi.radius())
        })
        .filter(|(_, soi_distance, soi_radius)| soi_distance < soi_radius)
        .min_by(|(_, a, _), (_, b, _)| a.total_cmp(b))
        .map(|(entity, _, _)| entity)
}

fn sphere_of_influence_to_hierarchy(
    mut commands: Commands,
    sim_time: Res<SimulationTime>,
    query: Query<(Entity, &Trajectory, &Orbiting), Without<SphereOfInfluence>>,
    query_soi: Query<(Entity, &Trajectory, &SphereOfInfluence)>,
) {
    for (entity, trajectory, orbiting) in query.iter() {
        let Some(position) = trajectory.position(sim_time.current()) else {
            continue;
        };

        let new_orbiting = under_soi(
            query_soi.iter().filter_map(|(entity, trajectory, soi)| {
                Some((entity, trajectory.position(sim_time.current())?, soi))
            }),
            position,
        );
        if let Some(new_orbiting) = new_orbiting {
            if **orbiting == new_orbiting {
                continue;
            }
            commands.queue(AddOrbit {
                orbiting: new_orbiting,
                body: entity,
            });
        }
    }
}
