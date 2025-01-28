use crate::prediction::{
    RelativeTrajectory, RelativeTrajectoryTranslated, Trajectory, TrajectoryData,
};

use bevy::prelude::*;
use hifitime::Epoch;

pub trait CompoundTrajectoryData: Component {
    type QueryData<'w>: bevy::ecs::query::QueryData;

    type Trajectory<'w>: TrajectoryData;

    /// Fetches and builds the actual trajectory.
    fn fetch<'w>(&self, data: &'w Query<Self::QueryData<'_>>) -> Option<Self::Trajectory<'w>>;
}

#[derive(Debug, Clone, Copy, Component)]
pub struct TrajectoryReference {
    pub entity: Entity,
    pub reference: Option<Entity>,
}

impl TrajectoryReference {
    #[expect(unused)]
    #[inline]
    pub fn new(entity: Entity, reference: Option<Entity>) -> Self {
        Self { entity, reference }
    }
}

impl CompoundTrajectoryData for TrajectoryReference {
    type QueryData<'w> = &'w Trajectory;

    type Trajectory<'w> = RelativeTrajectory<&'w Trajectory>;

    #[inline]
    fn fetch<'w>(&self, data: &'w Query<Self::QueryData<'_>>) -> Option<Self::Trajectory<'w>> {
        Some(RelativeTrajectory {
            trajectory: data.get(self.entity).ok()?,
            reference: self.reference.and_then(|r| data.get(r).ok()),
        })
    }
}

#[derive(Debug, Clone, Copy, Component)]
pub struct TrajectoryReferenceTranslated {
    pub relative: TrajectoryReference,
    pub translation_epoch: Epoch,
}

impl TrajectoryReferenceTranslated {
    #[inline]
    pub fn new(entity: Entity, reference: Option<Entity>, translation_epoch: Epoch) -> Self {
        Self {
            relative: TrajectoryReference { entity, reference },
            translation_epoch,
        }
    }
}

impl CompoundTrajectoryData for TrajectoryReferenceTranslated {
    type QueryData<'w> = &'w Trajectory;

    type Trajectory<'w> = RelativeTrajectoryTranslated<&'w Trajectory>;

    #[inline]
    fn fetch<'w>(&self, data: &'w Query<Self::QueryData<'_>>) -> Option<Self::Trajectory<'w>> {
        let trajectory = self.relative.fetch(data)?;

        Some(RelativeTrajectoryTranslated {
            trajectory,
            translation: trajectory
                .reference
                .and_then(|r| r.position(self.translation_epoch))
                .unwrap_or_default(),
        })
    }
}
