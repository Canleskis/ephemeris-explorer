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

        update_old_parent(world, self.child, self.parent);

        let mut parent = world.entity_mut(self.parent);
        if let Some(mut children_component) = parent.get_mut::<Children>() {
            children_component.0.retain(|value| self.child != *value);
            children_component.0.push(self.child);
        } else {
            parent.insert(Children(SmallVec::from_slice(&[self.child])));
        }
    }
}

fn update_parent(world: &mut World, child: Entity, new_parent: Entity) -> Option<Entity> {
    let mut child = world.entity_mut(child);
    if let Some(mut parent) = child.get_mut::<Parent>() {
        let previous = parent.0;
        *parent = Parent(new_parent);
        Some(previous)
    } else {
        child.insert(Parent(new_parent));
        None
    }
}

fn remove_from_children(world: &mut World, parent: Entity, child: Entity) {
    let Ok(mut parent) = world.get_entity_mut(parent) else {
        return;
    };
    let Some(mut children) = parent.get_mut::<Children>() else {
        return;
    };
    children.0.retain(|x| *x != child);
    if children.is_empty() {
        parent.remove::<Children>();
    }
}

fn update_old_parent(world: &mut World, child: Entity, parent: Entity) {
    let previous = update_parent(world, child, parent);
    if let Some(previous_parent) = previous {
        // Do nothing if the child was already parented to this entity.
        if previous_parent == parent {
            return;
        }
        remove_from_children(world, previous_parent, child);
    }
}
