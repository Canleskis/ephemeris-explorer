//! Similar to `bevy_hierarchy` but simply for an informative relationship.

use bevy::ecs::world::Command;
use bevy::prelude::*;
use smallvec::SmallVec;

#[derive(Component, Debug, Eq, PartialEq, Deref)]
pub struct Parent(Entity);

#[derive(Component, Debug, Default)]
pub struct Children(SmallVec<[Entity; 8]>);

impl std::ops::Deref for Children {
    type Target = [Entity];

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0[..]
    }
}

impl<'a> IntoIterator for &'a Children {
    type Item = <Self::IntoIter as Iterator>::Item;

    type IntoIter = std::slice::Iter<'a, Entity>;

    #[inline(always)]
    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

/// Command that adds a child to an entity.
#[derive(Debug)]
pub struct AddChild {
    /// Parent entity to add the child to.
    pub parent: Entity,
    /// Child entity to add.
    pub child: Entity,
}

// Rough implementation for the needs of the project.

impl Command for AddChild {
    fn apply(self, world: &mut World) {
        if self.child == self.parent {
            panic!("Cannot add entity as a child of itself.");
        }

        world.entity_mut(self.child).insert(Parent(self.parent));

        let mut parent = world.entity_mut(self.parent);
        if let Some(mut children_component) = parent.get_mut::<Children>() {
            children_component.0.retain(|value| self.child != *value);
            children_component.0.push(self.child);
        } else {
            parent.insert(Children(SmallVec::from_slice(&[self.child])));
        }
    }
}
