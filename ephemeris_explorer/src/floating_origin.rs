pub use big_space::{
    bundles::{BigGridBundle, BigSpaceRootBundle},
    commands::GridCommands,
    floating_origins::{BigSpace, FloatingOrigin},
    grid::{
        Grid,
        cell::CellCoord,
        local_origin::{Grids, LocalFloatingOrigin},
    },
    plugin::{BigSpaceCorePlugin, BigSpacePropagationPlugin, BigSpaceValidationPlugin},
    world_query::{
        CellTransform, CellTransformItem, CellTransformOwned, CellTransformReadOnly,
        CellTransformReadOnlyItem,
    },
};

use crate::dynamics::StateVector;
use bevy::math::DVec3;

pub trait GridExt {
    fn to_global_point(&self, point: DVec3) -> DVec3;

    fn to_global_vector(&self, vector: DVec3) -> DVec3;

    fn to_global_sv(&self, sv: StateVector) -> StateVector;
}

impl GridExt for Grid {
    #[inline]
    fn to_global_point(&self, point: DVec3) -> DVec3 {
        let origin = self.local_floating_origin();
        origin
            .grid_transform()
            .transform_point3(point - self.cell_to_float(&origin.cell()))
    }

    #[inline]
    fn to_global_vector(&self, vector: DVec3) -> DVec3 {
        self.local_floating_origin()
            .grid_transform()
            .transform_vector3(vector)
    }

    #[inline]
    fn to_global_sv(&self, sv: StateVector) -> StateVector {
        StateVector {
            position: self.to_global_point(sv.position),
            velocity: self.to_global_vector(sv.velocity),
        }
    }
}
