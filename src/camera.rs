use crate::{
    floating_origin::{BigSpace, Grid, GridCell, Precision},
    ui::WindowsUiSet,
    MainState,
};

use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    math::{DQuat, DVec3},
    prelude::*,
};
pub use big_space::camera::{
    camera_controller, nearest_objects_in_grid, CameraController, CameraInput,
};

pub fn using_pointer(query: Query<&Window, With<bevy::window::PrimaryWindow>>) -> bool {
    query.get_single().is_ok_and(|window| {
        window.cursor_options.grab_mode == bevy::window::CursorGrabMode::Confined
    })
}

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraState {
    #[default]
    Orbit,
    Follow,
}

#[derive(Component, Reflect, Clone, Copy)]
pub struct CanFollow {
    pub min_distance: f64,
    pub max_distance: f64,
}

#[derive(Resource, Deref, DerefMut, Default)]
pub struct Followed(pub Option<Entity>);

pub struct SetFollowed(pub Option<Entity>);

impl bevy::ecs::world::Command for SetFollowed {
    fn apply(self, world: &mut World) {
        let Some(followed) = self.0 else {
            return;
        };
        let can_follow = *world.entity(followed).get::<CanFollow>().unwrap();
        let (camera_entity, mut orbit) = world
            .query::<(Entity, &mut OrbitCamera)>()
            .single_mut(world);
        orbit.min_distance = can_follow.min_distance;
        orbit.max_distance = can_follow.max_distance;

        world.resource_mut::<Followed>().bypass_change_detection().0 = Some(followed);
        world.commands().entity(camera_entity).set_parent(followed);
        world
            .resource_mut::<NextState<CameraState>>()
            .set(CameraState::Orbit);
    }
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
}

impl AdditionalCameraInput {
    pub fn reset(&mut self) {
        self.scroll = 0.0;
    }
}

pub trait CameraInputExt {
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

#[derive(Default)]
pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Followed>()
            .insert_resource(CameraInput {
                defaults_disabled: true,
                ..default()
            })
            .insert_resource(AdditionalCameraInput::default())
            .insert_resource(DisabledControls::default())
            .insert_state(CameraState::Orbit)
            .add_systems(
                Update,
                (
                    (
                        crate::ui::using_pointer.pipe(disable_mouse_controls),
                        crate::ui::using_keyboard.pipe(disable_keyboard_controls),
                    )
                        .after(WindowsUiSet),
                    (
                        mouse_controls.run_if(|disabled: Res<DisabledControls>| !disabled.mouse),
                        keyboard_controls
                            .run_if(|disabled: Res<DisabledControls>| !disabled.keyboard),
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
                    .run_if(in_state(MainState::Running)),
            )
            .add_systems(
                PostUpdate,
                (
                    scale_mouse_to_fov,
                    follow_changed,
                    (
                        orbit_controls.run_if(in_state(CameraState::Orbit)),
                        (
                            sync_camera_distance,
                            nearest_objects_in_grid::<Precision>,
                            camera_controller::<Precision>,
                        )
                            .chain()
                            .run_if(in_state(CameraState::Follow)),
                    ),
                    reset_controls,
                )
                    .chain()
                    .before(bevy::transform::TransformSystem::TransformPropagate)
                    .run_if(in_state(MainState::Running)),
            );
    }
}

#[derive(Resource, Default)]
struct DisabledControls {
    pub mouse: bool,
    pub keyboard: bool,
}

fn disable_mouse_controls(
    In(disabled): In<bool>,
    mut disabled_controls: ResMut<DisabledControls>,
    query: Query<&Window, With<bevy::window::PrimaryWindow>>,
) {
    if !using_pointer(query) {
        disabled_controls.mouse = disabled;
    }
}

fn disable_keyboard_controls(
    In(disabled): In<bool>,
    mut disabled_controls: ResMut<DisabledControls>,
) {
    disabled_controls.keyboard = disabled;
}

fn follow_changed(mut commands: Commands, followed: Res<Followed>) {
    if !followed.is_changed() {
        return;
    }
    commands.queue(SetFollowed(**followed));
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

    if primary_window.cursor_options.visible {
        primary_window.cursor_options.visible = false;
        primary_window.cursor_options.grab_mode = bevy::window::CursorGrabMode::Confined;
    }
}

fn show_cursor(mut query: Query<&mut Window, With<bevy::window::PrimaryWindow>>) {
    let mut primary_window = query.single_mut();

    if !primary_window.cursor_options.visible {
        primary_window.cursor_options.visible = true;
        primary_window.cursor_options.grab_mode = bevy::window::CursorGrabMode::None;
    }
}

fn mouse_controls(
    mouse: Res<ButtonInput<MouseButton>>,
    mut mouse_move: EventReader<MouseMotion>,
    mut scroll_events: EventReader<MouseWheel>,
    mut input: ResMut<CameraInput>,
    mut input_2: ResMut<AdditionalCameraInput>,
    time: Res<Time>,
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
            input.pitch -= motion.y as f64 * 0.2 / time.delta_secs_f64();
            input.yaw -= motion.x as f64 * 0.2 / time.delta_secs_f64();
        }
    }
}

fn scale_mouse_to_fov(
    perspective: Single<&PerspectiveProjection, With<Camera>>,
    mut input: ResMut<CameraInput>,
) {
    input.pitch *= perspective.fov as f64;
    input.yaw *= perspective.fov as f64;
}

fn keyboard_controls(keyboard: Res<ButtonInput<KeyCode>>, mut input: ResMut<CameraInput>) {
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
    mut query_camera: Query<(Entity, &CameraController, &mut OrbitCamera)>,
    grid: Single<&Grid, With<BigSpace>>,
    time: Res<Time>,
) {
    let Ok((camera_entity, controller, mut orbit)) = query_camera.get_single_mut() else {
        return;
    };

    orbit.distance *= 1.0 - input_2.scroll * 0.2;
    orbit.distance = orbit.distance.clamp(orbit.min_distance, orbit.max_distance);

    let (mut camera_transform, mut camera_cell) = query.get_mut(camera_entity).unwrap();
    let mut rotation = camera_transform.rotation.as_dquat();

    let dt = time.delta_secs_f64();
    rotation *= DQuat::from_euler(
        EulerRot::XYZ,
        input.pitch * dt * controller.speed_pitch,
        input.yaw * dt * controller.speed_yaw,
        input.roll * dt * controller.speed_roll,
    );

    let (new_cell, translation) =
        grid.translation_to_grid(rotation.mul_vec3(DVec3::new(0.0, 0.0, orbit.distance)));

    *camera_cell = new_cell;
    camera_transform.translation = translation;
    camera_transform.rotation = rotation.as_quat();
}

fn reset_controls(mut input: ResMut<CameraInput>, mut input_2: ResMut<AdditionalCameraInput>) {
    input.reset();
    input_2.reset();
}
