#![expect(unused)]

pub use big_space::floating_origins::{BigSpace, FloatingOrigin};

pub type Precision = i32;
pub type FloatingOriginPlugin = big_space::plugin::BigSpacePlugin<Precision>;
pub type BigSpaceRootBundle = big_space::bundles::BigSpaceRootBundle<Precision>;
pub type BigGridBundle = big_space::bundles::BigGridBundle<Precision>;
pub type LocalFloatingOrigin = big_space::grid::local_origin::LocalFloatingOrigin<Precision>;
pub type GridCell = big_space::grid::cell::GridCell<Precision>;
pub type Grid = big_space::grid::Grid<Precision>;
pub type GridCommands<'a> = big_space::commands::GridCommands<'a, Precision>;
pub type GridTransform = big_space::world_query::GridTransform<Precision>;
pub type GridTransformItem<'w> = big_space::world_query::GridTransformItem<'w, Precision>;
pub type GridTransformReadOnly = big_space::world_query::GridTransformReadOnly<Precision>;
pub type GridTransformReadOnlyItem<'w> =
    big_space::world_query::GridTransformReadOnlyItem<'w, Precision>;
pub type GridTransformOwned = big_space::world_query::GridTransformOwned<Precision>;
