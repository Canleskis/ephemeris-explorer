#![expect(unused)]

pub use big_space::{BigSpace, FloatingOrigin};

pub type Precision = i32;
pub type FloatingOriginPlugin = big_space::BigSpacePlugin<Precision>;
pub type BigSpaceRootBundle = big_space::BigSpaceRootBundle<Precision>;
pub type BigReferenceFrameBundle = big_space::BigReferenceFrameBundle<Precision>;
pub type LocalFloatingOrigin =
    big_space::reference_frame::local_origin::LocalFloatingOrigin<Precision>;
pub type GridCell = big_space::GridCell<Precision>;
pub type ReferenceFrame = big_space::reference_frame::ReferenceFrame<Precision>;
pub type ReferenceFrameCommands<'a> = big_space::commands::ReferenceFrameCommands<'a, Precision>;
pub type GridTransform = big_space::world_query::GridTransform<Precision>;
pub type GridTransformItem<'w> = big_space::world_query::GridTransformItem<'w, Precision>;
pub type GridTransformReadOnly = big_space::world_query::GridTransformReadOnly<Precision>;
pub type GridTransformReadOnlyItem<'w> =
    big_space::world_query::GridTransformReadOnlyItem<'w, Precision>;
pub type GridTransformOwned = big_space::world_query::GridTransformOwned<Precision>;
