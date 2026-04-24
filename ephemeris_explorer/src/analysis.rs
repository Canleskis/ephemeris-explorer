use crate::{
    MainState,
    camera::CameraProximityIgnore,
    dynamics::{
        Segment, SoiTransitions, SpacecraftTrajectory, SphereOfInfluence, Trajectory, find_soi,
    },
    load::SystemRoot,
    prediction::{PredictionPropagator, PredictionSystems},
    simulation::SimulationTime,
    ui::{PlotBound, PlotConfig, PlotSeparation, PlotSource, PlotSourceOf, remove_tooltip},
};

use bevy::{math::DVec3, prelude::*};
use ephemeris::{BoundedTrajectory, EvaluateTrajectory, RelativeTrajectory};
use ftime::Epoch;
use smallvec::SmallVec;

#[derive(Component, Debug, Default)]
#[relationship_target(relationship = Primary)]
pub struct Satellites(SmallVec<[Entity; 8]>);

impl<'a> IntoIterator for &'a Satellites {
    type Item = <Self::IntoIter as Iterator>::Item;

    type IntoIter = std::slice::Iter<'a, Entity>;

    #[inline(always)]
    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}

#[derive(Component, Debug, Eq, PartialEq, Deref)]
#[relationship(relationship_target = Satellites)]
pub struct Primary(pub Entity);

impl Primary {
    pub fn get_relative_trajectory<'w>(
        &self,
        entity: Entity,
        query: &'w Query<&Trajectory>,
    ) -> Option<RelativeTrajectory<&'w Trajectory>> {
        Some(RelativeTrajectory::new(
            query.get(entity).ok()?,
            query.get(self.0).ok(),
        ))
    }
}

#[derive(Default)]
pub struct OrbitalAnalysisPlugin;

impl Plugin for OrbitalAnalysisPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(DrawSoiSettings { enabled: true })
            .add_systems(
                PreUpdate,
                (
                    (
                        setup_initial_soi_transition.run_if(is_system_initialized),
                        setup_segment_plotting,
                        setup_target_plotting,
                    )
                        .chain()
                        .after(PredictionSystems),
                    sphere_of_influence_to_hierarchy.after(setup_initial_soi_transition),
                    (spawn_drawn_soi, despawn_drawn_soi).chain(),
                )
                    .run_if(in_state(MainState::Running)),
            );
    }
}

fn sphere_of_influence_to_hierarchy(
    mut commands: Commands,
    sim_time: Res<SimulationTime>,
    query: Query<(Entity, &SoiTransitions, Option<&Primary>)>,
) {
    for (entity, transitions, orbiting) in query.iter() {
        if let Some(new_orbiting) = transitions.soi_at(sim_time.current())
            && orbiting.is_none_or(|orbiting| **orbiting != new_orbiting)
        {
            commands.entity(entity).insert(Primary(new_orbiting));
        }
    }
}

#[derive(Clone, Copy, Debug, Component)]
#[require(SoiTransitions)]
pub enum SoiTransitionsAnalysis {
    Static,
    Dynamic,
}

fn is_system_initialized(query: Query<&Trajectory, With<SphereOfInfluence>>) -> bool {
    query
        .iter()
        .all(|trajectory| trajectory.start() < trajectory.end())
}

// Not the ideal way to do this, but handles all edge cases nicely and performance is fine.
fn setup_initial_soi_transition(
    mut query: Query<(
        Entity,
        Ref<Trajectory>,
        &SoiTransitionsAnalysis,
        &mut SoiTransitions,
    )>,
    query_bodies: Query<(Entity, &Trajectory, &SphereOfInfluence)>,
    root: Single<Entity, With<SystemRoot>>,
) {
    let Some(start) = query_bodies.iter().map(|(_, traj, _)| traj.start()).max() else {
        return;
    };
    for (entity, trajectory, analysis, mut transitions) in query.iter_mut() {
        if !trajectory.is_changed() {
            continue;
        }

        if matches!(analysis, SoiTransitionsAnalysis::Static) && !transitions.is_empty() {
            continue;
        }

        let search_time = match analysis {
            SoiTransitionsAnalysis::Static => start,
            SoiTransitionsAnalysis::Dynamic => trajectory.start(),
        };

        let Some(position) = trajectory.position(search_time) else {
            continue;
        };

        let soi = find_soi_query(query_bodies, entity, search_time, position);
        // If an SOI for an entity can't be found we use the root.
        transitions.insert(Epoch::MIN, soi.unwrap_or(*root));
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum OrbitPlotReference {
    Entity(Entity),
    Primary,
}

#[derive(Clone, Copy, Debug, PartialEq, Component)]
pub struct OrbitPlotConfig {
    pub enabled: bool,
    pub start: Epoch,
    pub end: Epoch,
    pub bound: PlotBound,
    pub color: Color,
    pub resolution: f32,
    pub max_points_per_segment: usize,
    pub reference: OrbitPlotReference,
}

#[derive(Clone, Copy, Debug, Component)]
pub enum PlotSegment {
    Capture,
    Escape,
    Flyby,
    Transit,
    Orbit,
}

#[derive(Component)]
pub struct BurnPlotSegment;

#[derive(Component)]
pub struct OverlappingPlotSegment;

// This function sets up plot segments for entities with `OrbitPlotConfig`. Plot segments are
// standalone entities.
// We plot a new segment after every significant trajectory event, currently consisting of SOI
// transitions and burns. This allows for events to always be visible and to have different styles
// for different segments (reference, color, etc.). There may be multiple plot segments for the same
// segment of a trajectory.
#[expect(clippy::type_complexity)]
fn setup_segment_plotting(
    query: Query<
        (
            Entity,
            &SoiTransitions,
            &OrbitPlotConfig,
            Option<&PredictionPropagator<SpacecraftTrajectory>>,
        ),
        Or<(Changed<SoiTransitions>, Changed<OrbitPlotConfig>)>,
    >,
    query_transitions: Query<&SoiTransitions, With<SphereOfInfluence>>,
    query_name: Query<&Name>,
    root: Single<Entity, With<SystemRoot>>,
    mut commands: Commands,
) {
    for (e, transitions, config, propagator) in query.iter() {
        commands.entity(e).despawn_related::<PlotSourceOf>();
        commands.entity(e).with_children(|cmds| {
            let plot = PlotConfig {
                enabled: config.enabled,
                start: config.start,
                end: config.end,
                bound: config.bound,
                color: config.color,
                threshold: config.resolution,
                max_points: config.max_points_per_segment,
                dashed: false,
            };
            let reference = match config.reference {
                OrbitPlotReference::Entity(reference) => Some(reference),
                OrbitPlotReference::Primary => None,
            };

            const DEFAULT: &[Segment] = &[Segment::Coast {
                start: Epoch::MIN,
                end: Epoch::MAX,
            }];

            for (i, &(start, b)) in transitions.iter().enumerate() {
                // Plot relative to the entered SOI until the next transition, if any.
                let next = transitions.get(i + 1);

                if start > config.end || next.is_some_and(|&(end, _)| end < config.start) {
                    continue;
                }

                let parent_name = query_name.get(b).map(Name::as_str).unwrap_or("Unknown");
                let b_parent = query_transitions
                    .get(b)
                    .ok()
                    .and_then(|t| t.soi_at(start))
                    .unwrap_or(*root);
                let previous = i.checked_sub(1).and_then(|j| transitions.get(j));

                let start = start.max(config.start);
                let end = next.map_or(config.end, |&(end, _)| end.min(config.end));

                let segments = propagator
                    .map(|p| p.timeline().segments_between(start, end))
                    .unwrap_or(DEFAULT);

                let is_from_parent = previous.is_some_and(|&(_, a)| a == b_parent);
                let is_to_parent = next.is_some_and(|&(_, c)| c == b_parent);
                for segment in segments {
                    let start = segment.start().max(start);
                    let end = segment.end().min(end);

                    let is_burn = matches!(segment, Segment::Burn { .. });
                    let plot = PlotConfig {
                        start,
                        end,
                        dashed: is_burn,
                        ..plot
                    };

                    let mut plot_segment =
                        cmds.spawn((PlotSource::new(e, Some(reference.unwrap_or(b))), plot));
                    plot_segment.insert_if(BurnPlotSegment, || is_burn);

                    let burn_name = if is_burn { " Burn" } else { "" };

                    if is_from_parent && is_to_parent {
                        // Flyby: Entered from parent, leaving to parent
                        plot_segment.insert((
                            Name::new(format!("{parent_name} Flyby{burn_name}")),
                            PlotSegment::Flyby,
                        ));
                        // In that case, we also plot the segment relative to the parent.
                        if reference.is_none() {
                            cmds.spawn((
                                PlotSource::new(e, Some(b_parent)),
                                PlotConfig {
                                    color: config.color.with_alpha(plot.color.alpha() * 0.2),
                                    ..plot
                                },
                                Name::new(format!("{parent_name} Flyby{burn_name}")),
                                PlotSegment::Flyby,
                                OverlappingPlotSegment,
                            ))
                            .insert_if(BurnPlotSegment, || is_burn);
                        }
                    } else if is_from_parent && !is_to_parent {
                        // Capture: Entered from parent, not leaving to parent
                        plot_segment.insert((
                            Name::new(format!("{parent_name} Capture{burn_name}")),
                            PlotSegment::Capture,
                        ));
                    } else if !is_from_parent && is_to_parent {
                        // Escape: Not entered from parent, leaving to parent
                        plot_segment.insert((
                            Name::new(format!("{parent_name} Escape{burn_name}")),
                            PlotSegment::Escape,
                        ));
                    } else if previous.is_some() || next.is_some() {
                        // Transit: Not entered from parent, not leaving to parent
                        plot_segment.insert((
                            Name::new(format!("{parent_name} Transit{burn_name}")),
                            PlotSegment::Transit,
                        ));
                    } else {
                        // Orbit: No transitions
                        plot_segment.insert((
                            Name::new(format!("{parent_name} Orbit{burn_name}")),
                            PlotSegment::Orbit,
                        ));
                    }
                }
            }
        });
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Component, Hash)]
pub struct OrbitTarget(pub Option<Entity>);

impl OrbitTarget {
    #[inline]
    pub fn entity(&self) -> Option<Entity> {
        self.0
    }
}

#[expect(clippy::type_complexity)]
fn setup_target_plotting(
    query: Query<(
        Ref<Trajectory>,
        Ref<OrbitPlotConfig>,
        &PlotSourceOf,
        Option<Ref<OrbitTarget>>,
    )>,
    query_plots: Query<(Entity, &PlotConfig)>,
    mut commands: Commands,
) {
    for (trajectory, plot, source_of, target) in query.iter() {
        let Some(target) = target else { continue };

        if !(target.is_changed()
            || trajectory.is_changed()
            || plot.is_changed()
            || target.entity().and_then(|e| query.get(e).ok()).is_some_and(
                |(target_trajectory, target_plot, ..)| {
                    target_trajectory.is_changed() || target_plot.is_changed()
                },
            ))
        {
            continue;
        }

        for plot_entity in source_of.iter() {
            commands.entity(plot_entity).queue(remove_tooltip);
        }

        let Some((target_trajectory, _, target_source_of, ..)) =
            target.entity().and_then(|entity| query.get(entity).ok())
        else {
            continue;
        };

        let relative = RelativeTrajectory::new(&*trajectory, Some(&*target_trajectory));
        if let Some(time) =
            relative.closest_separation_between(plot.start, plot.end, 0.001, 1000, |t1, t2, at| {
                t1.distance_squared_at(t2, at).unwrap()
            })
        {
            for (plot_entity, plot_config) in query_plots.iter_many(&**source_of) {
                if time < plot_config.start || time > plot_config.end {
                    continue;
                }

                for (target_plot_entity, target_plot_config) in
                    query_plots.iter_many(&**target_source_of)
                {
                    if time < target_plot_config.start || time > target_plot_config.end {
                        continue;
                    }

                    commands.entity(plot_entity).insert(PlotSeparation {
                        entity: target_plot_entity,
                        time,
                        distance: relative.position(time).unwrap().length(),
                    });
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Resource)]
pub struct DrawSoiSettings {
    pub enabled: bool,
}

#[derive(Clone, Copy, Debug, Component)]
pub struct IsSoiDrawn;

#[derive(Clone, Copy, Debug, Component)]
pub struct DrawnSoiOf(Entity);

fn spawn_drawn_soi(
    settings: Res<DrawSoiSettings>,
    query: Query<(Entity, &SphereOfInfluence), Without<IsSoiDrawn>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !settings.enabled {
        return;
    }

    for (entity, soi) in query.iter() {
        commands
            .entity(entity)
            .insert(IsSoiDrawn)
            .with_children(|cmds| {
                cmds.spawn((
                    Mesh3d(meshes.add(Sphere::new(soi.radius as f32).mesh().uv(144, 72))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: Color::linear_rgba(0.08, 0.13, 0.28, -0.135),
                        alpha_mode: AlphaMode::Add,
                        unlit: true,
                        ..default()
                    })),
                    bevy::light::NotShadowReceiver,
                    bevy::light::NotShadowCaster,
                    CameraProximityIgnore,
                    DrawnSoiOf(entity),
                ));

                cmds.spawn((
                    Mesh3d(meshes.add(Sphere::new(soi.radius as f32 * 1.002).mesh().uv(144, 72))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: Color::linear_rgba(0.08, 0.13, 0.28, 0.15),
                        alpha_mode: AlphaMode::Add,
                        unlit: true,
                        ..default()
                    })),
                    bevy::light::NotShadowReceiver,
                    bevy::light::NotShadowCaster,
                    CameraProximityIgnore,
                    DrawnSoiOf(entity),
                ));
            });
    }
}

fn despawn_drawn_soi(
    settings: Res<DrawSoiSettings>,
    query: Query<(Entity, &DrawnSoiOf)>,
    mut commands: Commands,
) {
    if settings.enabled {
        return;
    }

    for (entity, soi) in query.iter() {
        commands.entity(entity).despawn();
        commands.entity(soi.0).remove::<IsSoiDrawn>();
    }
}

fn find_soi_query(
    query_bodies: Query<(Entity, &Trajectory, &SphereOfInfluence)>,
    except: Entity,
    time: Epoch,
    position: DVec3,
) -> Option<Entity> {
    find_soi(
        query_bodies
            .iter()
            .filter(|(e, _, _)| *e != except)
            .filter_map(|(entity, other_trajectory, soi)| {
                Some((entity, other_trajectory.position(time)?, soi.radius))
            }),
        position,
    )
}
