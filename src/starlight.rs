use bevy::prelude::*;

#[derive(Component, Deref, DerefMut)]
pub struct StarLight(pub Entity);

#[derive(Component)]
pub struct Star {
    pub color: Color,
    pub illuminance: f32,
}

#[derive(Default)]
pub struct StarLightPlugin;

impl Plugin for StarLightPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(First, setup_lights).add_systems(
            PostUpdate,
            lighting.after(bevy::transform::TransformSystem::TransformPropagate),
        );
    }
}

fn setup_lights(mut commands: Commands, query: Query<(Entity, &Star), Added<Star>>) {
    for (entity, star) in &query {
        commands
            .spawn((
                Name::new(format!("{}'s starlight", entity)),
                StarLight(entity),
                DirectionalLight {
                    color: star.color,
                    illuminance: star.illuminance,
                    shadows_enabled: true,
                    ..default()
                },
                bevy::pbr::CascadeShadowConfigBuilder {
                    num_cascades: 4,
                    minimum_distance: 100.0,
                    maximum_distance: 1e9,
                    first_cascade_far_bound: 2000.0,
                    overlap_proportion: 0.2,
                }
                .build(),
            ))
            .set_parent(entity);
    }
}

fn lighting(
    mut light: Query<(&mut Transform, &mut GlobalTransform, &StarLight)>,
    star: Query<&GlobalTransform, (With<Star>, Without<StarLight>)>,
) {
    for (mut transform, mut global, light) in &mut light {
        if let Ok(star_global) = star.get(**light) {
            transform.look_at(-star_global.translation(), Vec3::Y);
            *global = GlobalTransform::from(*transform);
        }
    }
}
