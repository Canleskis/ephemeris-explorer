use crate::{
    floating_origin::{BigSpace, GridCell, Precision, ReferenceFrame},
    selection::Followed,
    GameState,
};

use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    math::{DQuat, DVec3},
    prelude::*,
};
use big_space::camera::{camera_controller, nearest_objects_in_frame, CameraInput};

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraState {
    #[default]
    Orbit,
    Follow,
}

#[derive(Component, Reflect)]
pub struct CanFollow {
    pub min_distance: f64,
    pub max_distance: f64,
}

#[derive(Component, Default)]
pub struct OrbitCamera {
    min_distance: f64,
    max_distance: f64,
    distance: f64,
}

impl OrbitCamera {
    pub fn with_distance(mut self, distance: f64) -> Self {
        self.distance = distance;
        self
    }
}

#[derive(Resource, Default)]
pub struct AdditionalCameraInput {
    pub scroll: f64,
    pub zoom: f64,
}

impl AdditionalCameraInput {
    pub fn reset(&mut self) {
        self.scroll = 0.0;
        self.zoom = 0.0;
    }
}

trait CameraInputExt {
    fn any_mouse_input(&self) -> bool;

    fn any_kb_input(&self) -> bool;
}

impl CameraInputExt for CameraInput {
    fn any_mouse_input(&self) -> bool {
        self.pitch != 0.0 || self.yaw != 0.0
    }

    fn any_kb_input(&self) -> bool {
        self.forward != 0.0 || self.right != 0.0 || self.up != 0.0 || self.roll != 0.0
    }
}

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(CameraInput {
            defaults_disabled: true,
            ..default()
        })
        .insert_resource(AdditionalCameraInput::default())
        .insert_resource(DisabledControls::default())
        .insert_state(CameraState::Orbit)
        .add_systems(
            PreUpdate,
            disable_controls_ui.run_if(in_state(GameState::Running)),
        )
        .add_systems(
            Update,
            (
                (
                    mouse_controls.run_if(|disabled: Res<DisabledControls>| !disabled.mouse),
                    keyboard_controls.run_if(|disabled: Res<DisabledControls>| !disabled.keyboard),
                ),
                (
                    change_camera_state,
                    hide_cursor.run_if(|input: Res<CameraInput>| input.any_mouse_input()),
                    show_cursor.run_if(|mouse: Res<ButtonInput<MouseButton>>| {
                        mouse.any_just_released([MouseButton::Left, MouseButton::Right])
                    }),
                ),
            )
                .chain()
                .run_if(in_state(GameState::Running)),
        )
        .add_systems(
            PostUpdate,
            (
                scale_mouse_to_zoom,
                zoom_camera,
                follow_changed,
                (
                    orbit_controls.chain().run_if(in_state(CameraState::Orbit)),
                    (
                        sync_camera_distance,
                        nearest_objects_in_frame::<Precision>,
                        camera_controller::<Precision>,
                    )
                        .chain()
                        .run_if(in_state(CameraState::Follow)),
                ),
                reset_controls,
            )
                .chain()
                .before(bevy::transform::TransformSystem::TransformPropagate)
                .run_if(in_state(GameState::Running)),
        );
    }
}

fn disable_controls_ui(
    ctx: bevy_egui::EguiContexts,
    mut disabled_controls: ResMut<DisabledControls>,
) {
    let Some(ctx) = ctx.try_ctx() else {
        return;
    };

    disabled_controls.mouse = ctx.is_pointer_over_area() || ctx.wants_pointer_input();
    disabled_controls.keyboard = ctx.wants_keyboard_input();
}

#[derive(Resource, Default)]
struct DisabledControls {
    pub mouse: bool,
    pub keyboard: bool,
}

fn follow_changed(
    mut commands: Commands,
    mut state: ResMut<NextState<CameraState>>,
    followed: Res<Followed>,
    query_can_follow: Query<&CanFollow>,
    mut query_camera: Query<(Entity, &mut OrbitCamera)>,
) {
    if !followed.is_changed() {
        return;
    }

    let Ok((camera_entity, mut orbit)) = query_camera.get_single_mut() else {
        return;
    };

    let Some(followed) = **followed else {
        return;
    };

    let Ok(can_follow) = query_can_follow.get(followed) else {
        return;
    };

    orbit.min_distance = can_follow.min_distance;
    orbit.max_distance = can_follow.max_distance;

    commands.entity(followed).add_child(camera_entity);

    state.set(CameraState::Orbit);
}

fn change_camera_state(world: &mut World) {
    if world.resource::<DisabledControls>().mouse {
        return;
    }

    let mouse_input = world.resource::<ButtonInput<MouseButton>>();
    let input = world.resource::<CameraInput>();

    if mouse_input.pressed(MouseButton::Right) {
        if *world.resource::<State<CameraState>>() != CameraState::Orbit {
            world
                .resource_mut::<NextState<CameraState>>()
                .set(CameraState::Orbit);
        }
    } else if ((mouse_input.pressed(MouseButton::Left) && input.any_mouse_input())
        || input.any_kb_input())
        && *world.resource::<State<CameraState>>() != CameraState::Follow
    {
        world
            .resource_mut::<NextState<CameraState>>()
            .set(CameraState::Follow);
    }
}

fn hide_cursor(mut query: Query<&mut Window, With<bevy::window::PrimaryWindow>>) {
    let mut primary_window = query.single_mut();

    if primary_window.cursor.visible {
        primary_window.cursor.visible = false;
        primary_window.cursor.grab_mode = bevy::window::CursorGrabMode::Confined;
    }
}

fn show_cursor(mut query: Query<&mut Window, With<bevy::window::PrimaryWindow>>) {
    let mut primary_window = query.single_mut();

    if !primary_window.cursor.visible {
        primary_window.cursor.visible = true;
        primary_window.cursor.grab_mode = bevy::window::CursorGrabMode::None;
    }
}

pub fn mouse_controls(
    mouse: Res<ButtonInput<MouseButton>>,
    mut mouse_move: EventReader<MouseMotion>,
    mut scroll_events: EventReader<MouseWheel>,
    mut input: ResMut<CameraInput>,
    mut input_2: ResMut<AdditionalCameraInput>,
) {
    input_2.scroll = scroll_events
        .read()
        .map(|ev| match ev.unit {
            bevy::input::mouse::MouseScrollUnit::Pixel => ev.y * 0.005,
            bevy::input::mouse::MouseScrollUnit::Line => ev.y * 1.0,
        } as f64)
        .sum::<f64>();

    if mouse.pressed(MouseButton::Left) || mouse.pressed(MouseButton::Right) {
        if let Some(motion) = mouse_move.read().map(|e| e.delta).reduce(|sum, i| sum + i) {
            input.pitch -= motion.y as f64 * 0.2;
            input.yaw -= motion.x as f64 * 0.2;
        }
    }
}

fn scale_mouse_to_zoom(
    projection: Query<&Projection, With<Camera>>,
    mut input: ResMut<CameraInput>,
) {
    let Projection::Perspective(perspective) = projection.single() else {
        return;
    };

    input.pitch *= perspective.fov as f64;
    input.yaw *= perspective.fov as f64;
}

pub fn keyboard_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut input: ResMut<CameraInput>,
    mut input_2: ResMut<AdditionalCameraInput>,
) {
    keyboard
        .pressed(KeyCode::ArrowUp)
        .then(|| input_2.zoom += 1.0);
    keyboard
        .pressed(KeyCode::ArrowDown)
        .then(|| input_2.zoom -= 1.0);
    keyboard
        .pressed(KeyCode::KeyW)
        .then(|| input.forward -= 1.0);
    keyboard
        .pressed(KeyCode::KeyS)
        .then(|| input.forward += 1.0);
    keyboard.pressed(KeyCode::KeyA).then(|| input.right -= 1.0);
    keyboard.pressed(KeyCode::KeyD).then(|| input.right += 1.0);
    keyboard.pressed(KeyCode::Space).then(|| input.up += 1.0);
    keyboard
        .pressed(KeyCode::ControlLeft)
        .then(|| input.up -= 1.0);
    keyboard.pressed(KeyCode::KeyQ).then(|| input.roll += 2.0);
    keyboard.pressed(KeyCode::KeyE).then(|| input.roll -= 2.0);
    keyboard
        .pressed(KeyCode::ShiftLeft)
        .then(|| input.boost = true);
}

fn zoom_camera(
    time: Res<Time>,
    input_2: ResMut<AdditionalCameraInput>,
    mut camera: Query<&mut Projection, With<Camera>>,
) {
    for mut projection in camera.iter_mut() {
        if let Projection::Perspective(perspective) = &mut *projection {
            perspective.fov *= 1.0 + input_2.zoom as f32 * time.delta_seconds();
            perspective.fov = perspective
                .fov
                .clamp(0.001, 2.0 * std::f32::consts::FRAC_PI_3);
        }
    }
}

fn sync_camera_distance(
    followed: Res<Followed>,
    mut query_camera: Query<(&GlobalTransform, &mut OrbitCamera)>,
    transform: Query<&GlobalTransform>,
) {
    let Ok((camera_transform, mut orbit)) = query_camera.get_single_mut() else {
        return;
    };

    if let Some(followed_pos) = followed.and_then(|e| transform.get(e).ok()) {
        orbit.distance = followed_pos
            .translation()
            .as_dvec3()
            .distance(camera_transform.translation().as_dvec3());
    }
}

pub fn orbit_controls(
    input: Res<CameraInput>,
    input_2: Res<AdditionalCameraInput>,
    mut query: Query<(&mut Transform, &mut GridCell)>,
    mut query_camera: Query<(Entity, &mut OrbitCamera)>,
    root: Query<&ReferenceFrame, With<BigSpace>>,
    time: Res<Time>,
) {
    let root = root.single();

    let Ok((camera_entity, mut orbit)) = query_camera.get_single_mut() else {
        return;
    };

    orbit.distance *= 1.0 - input_2.scroll * 0.2;
    orbit.distance = orbit.distance.clamp(orbit.min_distance, orbit.max_distance);

    let (mut camera_transform, mut camera_cell) = query.get_mut(camera_entity).unwrap();
    let mut rotation = camera_transform.rotation.as_dquat();

    let dt = time.delta_seconds_f64();
    let speed = 1.0;
    rotation *= DQuat::from_euler(
        EulerRot::XYZ,
        input.pitch * dt * speed,
        input.yaw * dt * speed,
        input.roll * dt * speed,
    );

    let (new_cell, translation) =
        root.translation_to_grid(rotation.mul_vec3(DVec3::new(0.0, 0.0, orbit.distance)));

    *camera_cell = new_cell;
    camera_transform.translation = translation;
    camera_transform.rotation = rotation.as_quat();
}

fn reset_controls(mut input: ResMut<CameraInput>, mut input_2: ResMut<AdditionalCameraInput>) {
    input.reset();
    input_2.reset();
}
