//! Similar to `bevy_hierarchy` but simply for an informative relationship.

use bevy::ecs::query::{QueryData, QueryFilter};
use bevy::prelude::*;
use smallvec::SmallVec;

#[derive(Component, Debug, Eq, PartialEq, Deref)]
pub struct Orbiting(Entity);

#[derive(Component, Debug, Default)]
pub struct OrbitedBy(SmallVec<[Entity; 8]>);

impl std::ops::Deref for OrbitedBy {
    type Target = [Entity];

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0[..]
    }
}

impl<'a> IntoIterator for &'a OrbitedBy {
    type Item = <Self::IntoIter as Iterator>::Item;

    type IntoIter = std::slice::Iter<'a, Entity>;

    #[inline(always)]
    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

#[derive(Debug)]
pub struct AddOrbit {
    pub orbiting: Entity,
    pub body: Entity,
}

// Rough implementation for the needs of the project.

impl Command for AddOrbit {
    fn apply(self, world: &mut World) {
        if self.body == self.orbiting {
            panic!("Cannot add entity as a child of itself.");
        }

        update_old_orbiting(world, self.body, self.orbiting);

        let mut orbiting = world.entity_mut(self.orbiting);
        if let Some(mut orbited_by_component) = orbiting.get_mut::<OrbitedBy>() {
            orbited_by_component.0.retain(|value| self.body != *value);
            orbited_by_component.0.push(self.body);
        } else {
            orbiting.insert(OrbitedBy(SmallVec::from_slice(&[self.body])));
        }
    }
}

fn update_orbiting(world: &mut World, child: Entity, new_orbiting: Entity) -> Option<Entity> {
    let mut body = world.entity_mut(child);
    if let Some(mut orbiting) = body.get_mut::<Orbiting>() {
        let previous = orbiting.0;
        *orbiting = Orbiting(new_orbiting);
        Some(previous)
    } else {
        body.insert(Orbiting(new_orbiting));
        None
    }
}

fn remove_from_orbited_by(world: &mut World, orbiting: Entity, body: Entity) {
    let Ok(mut orbiting) = world.get_entity_mut(orbiting) else {
        return;
    };
    let Some(mut orbited) = orbiting.get_mut::<OrbitedBy>() else {
        return;
    };
    orbited.0.retain(|x| *x != body);
    if orbited.is_empty() {
        orbiting.remove::<OrbitedBy>();
    }
}

fn update_old_orbiting(world: &mut World, body: Entity, orbiting: Entity) {
    let previous = update_orbiting(world, body, orbiting);
    if let Some(previous_orbiting) = previous {
        if previous_orbiting == orbiting {
            return;
        }
        remove_from_orbited_by(world, previous_orbiting, body);
    }
}

/// An [`Iterator`] of [`Entity`]s over the descendants of an [`Entity`].
///
/// Traverses the hierarchy breadth-first.
pub struct OrbitingDescendantIter<'w, 's, D: QueryData, F: QueryFilter>
where
    D::ReadOnly: QueryData<Item<'w> = &'w OrbitedBy>,
{
    orbited_by_query: &'w Query<'w, 's, D, F>,
    vecdeque: std::collections::VecDeque<Entity>,
}

impl<'w, 's, D: QueryData, F: QueryFilter> OrbitingDescendantIter<'w, 's, D, F>
where
    D::ReadOnly: QueryData<Item<'w> = &'w OrbitedBy>,
{
    /// Returns a new [`DescendantIter`].
    pub fn new(orbited_by_query: &'w Query<'w, 's, D, F>, entity: Entity) -> Self {
        OrbitingDescendantIter {
            orbited_by_query,
            vecdeque: orbited_by_query
                .get(entity)
                .into_iter()
                .flatten()
                .copied()
                .collect(),
        }
    }
}

impl<'w, D: QueryData, F: QueryFilter> Iterator for OrbitingDescendantIter<'w, '_, D, F>
where
    D::ReadOnly: QueryData<Item<'w> = &'w OrbitedBy>,
{
    type Item = Entity;

    fn next(&mut self) -> Option<Self::Item> {
        let entity = self.vecdeque.pop_front()?;

        if let Ok(orbited_by) = self.orbited_by_query.get(entity) {
            self.vecdeque.extend(orbited_by);
        }

        Some(entity)
    }
}

pub trait HierarchyQueryExt<'w, 's, D: QueryData, F: QueryFilter> {
    fn iter_orbiting_descendants(&'w self, entity: Entity) -> OrbitingDescendantIter<'w, 's, D, F>
    where
        D::ReadOnly: QueryData<Item<'w> = &'w OrbitedBy>;
}

impl<'w, 's, D: QueryData, F: QueryFilter> HierarchyQueryExt<'w, 's, D, F> for Query<'w, 's, D, F> {
    fn iter_orbiting_descendants(&'w self, entity: Entity) -> OrbitingDescendantIter<'w, 's, D, F>
    where
        D::ReadOnly: QueryData<Item<'w> = &'w OrbitedBy>,
    {
        OrbitingDescendantIter::new(self, entity)
    }
}
