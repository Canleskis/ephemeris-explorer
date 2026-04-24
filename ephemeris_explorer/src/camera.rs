use crate::{
    MainState,
    floating_origin::{BigSpace, CellCoord, Grid, Grids},
    ui::{WindowsUiSet, WorldUiSet},
};

use bevy::{
    camera::{primitives::Aabb, visibility::RenderLayers},
    input::mouse::{MouseMotion, MouseWheel},
    math::{DQuat, DVec3},
    prelude::*,
};

pub fn using_pointer(
    query: Query<&bevy::window::CursorOptions, With<bevy::window::PrimaryWindow>>,
) -> bool {
    query.single().is_ok_and(|cursor_options| {
        cursor_options.grab_mode == bevy::window::CursorGrabMode::Confined
    })
}

/// A simple fly-cam camera controller. Based on [`big_space`]'s camera controller.
#[derive(Clone, Debug, Reflect, Component)]
#[reflect(Component)]
pub struct CameraController {
    /// Smoothness of translation, from `0.0` to `1.0`.
    pub smoothness: f64,
    /// Rotational smoothness, from `0.0` to `1.0`.
    pub rotational_smoothness: f64,
    /// Base speed.
    pub speed: f64,
    /// Rotational yaw speed multiplier.
    pub speed_yaw: f64,
    /// Rotational pitch speed multiplier.
    pub speed_pitch: f64,
    /// Rotational roll speed multiplier.
    pub speed_roll: f64,
    /// Minimum and maximum speed.
    pub speed_bounds: [f64; 2],
    /// Whether the camera should slow down when approaching an entity's [`Aabb`].
    pub slow_near_objects: bool,
    nearest_object: Option<(Entity, f64)>,
    vel_translation: DVec3,
    vel_rotation: DQuat,
}

impl CameraController {
    /// Sets the `smoothness` parameter of the controller, and returns the modified result.
    pub fn with_smoothness(mut self, translation: f64, rotation: f64) -> Self {
        self.smoothness = translation;
        self.rotational_smoothness = rotation;
        self
    }

    /// Sets the `slow_near_objects` parameter of the controller, and returns the modified result.
    pub fn with_slowing(mut self, slow_near_objects: bool) -> Self {
        self.slow_near_objects = slow_near_objects;
        self
    }

    /// Sets the speed of the controller, and returns the modified result.
    pub fn with_speed(mut self, speed: f64) -> Self {
        self.speed = speed;
        self
    }

    /// Sets the yaw angular velocity of the controller, and returns the modified result.
    pub fn with_speed_yaw(mut self, speed: f64) -> Self {
        self.speed_yaw = speed;
        self
    }

    /// Sets the pitch angular velocity of the controller, and returns the modified result.
    pub fn with_speed_pitch(mut self, speed: f64) -> Self {
        self.speed_pitch = speed;
        self
    }

    /// Sets the pitch angular velocity of the controller, and returns the modified result.
    pub fn with_speed_roll(mut self, speed: f64) -> Self {
        self.speed_roll = speed;
        self
    }

    /// Sets the speed of the controller, and returns the modified result.
    pub fn with_speed_bounds(mut self, speed_limits: [f64; 2]) -> Self {
        self.speed_bounds = speed_limits;
        self
    }

    /// Returns the translational and rotational velocity of the camera.
    pub fn velocity(&self) -> (DVec3, DQuat) {
        (self.vel_translation, self.vel_rotation)
    }

    /// Returns the object nearest the camera, and its distance.
    pub fn nearest_object(&self) -> Option<(Entity, f64)> {
        self.nearest_object
    }
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            smoothness: 0.85,
            rotational_smoothness: 0.8,
            speed: 1.0,
            speed_pitch: 2.0,
            speed_yaw: 2.0,
            speed_roll: 1.0,
            speed_bounds: [1e-17, 1e30],
            slow_near_objects: true,
            nearest_object: None,
            vel_translation: DVec3::ZERO,
            vel_rotation: DQuat::IDENTITY,
        }
    }
}

/// `ButtonInput` state used to command [`CameraController`] motion. Reset every time the values
/// are read to update the camera. Allows you to map any input to camera motions. Uses aircraft
/// principle axes conventions.
#[derive(Clone, Debug, Default, Reflect, Resource)]
#[reflect(Resource)]
pub struct CameraInput {
    /// When disabled, the camera input system is not run.
    pub defaults_disabled: bool,
    /// Z-negative
    pub forward: f64,
    /// Y-positive
    pub up: f64,
    /// X-positive
    pub right: f64,
    /// Positive = right wing down
    pub roll: f64,
    /// Positive = nose up
    pub pitch: f64,
    /// Positive = nose right
    pub yaw: f64,
    /// Modifier to increase speed, e.g. "sprint"
    pub boost: bool,

    pub scroll: f64,
}

impl CameraInput {
    fn any_mouse_input(&self) -> bool {
        self.pitch != 0.0 || self.yaw != 0.0
    }

    fn any_kb_input(&self) -> bool {
        self.forward != 0.0 || self.right != 0.0 || self.up != 0.0 || self.roll != 0.0
    }

    /// Reset the controller back to zero to ready for the next grid.
    pub fn reset(&mut self) {
        *self = CameraInput {
            defaults_disabled: self.defaults_disabled,
            ..Default::default()
        };
    }

    /// Returns the desired velocity transform.
    pub fn target_velocity(
        &self,
        controller: &CameraController,
        speed: f64,
        dt: f64,
    ) -> (DVec3, DQuat) {
        let rotation = DQuat::from_euler(
            EulerRot::XYZ,
            self.pitch * dt * controller.speed_pitch,
            self.yaw * dt * controller.speed_yaw,
            self.roll * dt * controller.speed_roll,
        );

        let translation = DVec3::new(self.right, self.up, self.forward) * speed * dt;

        (translation, rotation)
    }
}

#[derive(Clone, Copy, Debug, Default, Component)]
pub struct CameraProximityIgnore;

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraState {
    #[default]
    Orbit,
    Follow,
}

#[derive(Component, Reflect, Clone, Copy, Default)]
pub struct CanFollow;

#[derive(Component, Reflect, Clone, Copy)]
#[require(CanFollow)]
pub struct FollowParameters {
    pub min_distance: f64,
    pub max_distance: f64,
}

#[derive(Resource, Deref, DerefMut, Default)]
pub struct Followed(pub Option<Entity>);

pub struct SetFollowed(pub Option<Entity>);

impl Command for SetFollowed {
    fn apply(self, world: &mut World) {
        let Some(followed) = self.0 else { return };

        let can_follow = *world.entity(followed).get::<FollowParameters>().unwrap();
        let (camera_entity, mut orbit) = world
            .query::<(Entity, &mut OrbitCamera)>()
            .single_mut(world)
            .unwrap();
        orbit.min_distance = can_follow.min_distance;
        orbit.max_distance = can_follow.max_distance;

        world.resource_mut::<Followed>().bypass_change_detection().0 = Some(followed);
        world
            .commands()
            .entity(camera_entity)
            .insert(ChildOf(followed));
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

#[derive(Default)]
pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Followed>()
            .insert_resource(CameraInput {
                defaults_disabled: true,
                ..default()
            })
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
                            nearest_objects_in_grid,
                            camera_controller,
                        )
                            .chain()
                            .run_if(in_state(CameraState::Follow)),
                    ),
                    reset_controls,
                )
                    .chain()
                    .before(WorldUiSet)
                    .before(TransformSystems::Propagate)
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
    query: Query<&bevy::window::CursorOptions, With<bevy::window::PrimaryWindow>>,
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

fn hide_cursor(
    mut cursor_options: Single<&mut bevy::window::CursorOptions, With<bevy::window::PrimaryWindow>>,
) {
    if cursor_options.visible {
        cursor_options.visible = false;
        cursor_options.grab_mode = bevy::window::CursorGrabMode::Confined;
    }
}

fn show_cursor(
    mut cursor_options: Single<&mut bevy::window::CursorOptions, With<bevy::window::PrimaryWindow>>,
) {
    if !cursor_options.visible {
        cursor_options.visible = true;
        cursor_options.grab_mode = bevy::window::CursorGrabMode::None;
    }
}

fn mouse_controls(
    mouse: Res<ButtonInput<MouseButton>>,
    mut mouse_move: MessageReader<MouseMotion>,
    mut scroll_events: MessageReader<MouseWheel>,
    mut input: ResMut<CameraInput>,
    time: Res<Time>,
) {
    input.scroll = scroll_events
        .read()
        .map(|ev| match ev.unit {
            bevy::input::mouse::MouseScrollUnit::Pixel => ev.y * 0.005,
            bevy::input::mouse::MouseScrollUnit::Line => ev.y * 1.0,
        } as f64)
        .sum::<f64>();

    if (mouse.pressed(MouseButton::Left) || mouse.pressed(MouseButton::Right))
        && let Some(motion) = mouse_move.read().map(|e| e.delta).reduce(|sum, i| sum + i)
    {
        input.pitch -= motion.y as f64 * 0.2 / time.delta_secs_f64();
        input.yaw -= motion.x as f64 * 0.2 / time.delta_secs_f64();
    }
}

fn scale_mouse_to_fov(
    perspective: Single<&Projection, With<Camera>>,
    mut input: ResMut<CameraInput>,
) {
    let Projection::Perspective(perspective) = *perspective else {
        unreachable!("Camera is not perspective")
    };
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
    let Ok((camera_transform, mut orbit)) = query_camera.single_mut() else {
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
    mut query: Query<(&mut Transform, &mut CellCoord)>,
    mut query_camera: Query<(Entity, &CameraController, &mut OrbitCamera)>,
    grid: Single<&Grid, With<BigSpace>>,
    time: Res<Time>,
) {
    let Ok((camera_entity, controller, mut orbit)) = query_camera.single_mut() else {
        return;
    };

    orbit.distance *= 1.0 - input.scroll * 0.2;
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

fn reset_controls(mut input: ResMut<CameraInput>) {
    input.reset();
}

/// Find the object nearest the camera, within the same grid as the camera.
#[expect(clippy::type_complexity)]
pub fn nearest_objects_in_grid(
    objects: Query<
        (
            Entity,
            &Transform,
            &GlobalTransform,
            &Aabb,
            Option<&RenderLayers>,
            &InheritedVisibility,
        ),
        Without<CameraProximityIgnore>,
    >,
    mut camera: Query<(
        Entity,
        &mut CameraController,
        &GlobalTransform,
        Option<&RenderLayers>,
    )>,
    children: Query<&Children>,
) {
    let Ok((cam_entity, mut camera, cam_pos, cam_layer)) = camera.single_mut() else {
        return;
    };
    if !camera.slow_near_objects {
        return;
    }
    let cam_layer = cam_layer.to_owned().unwrap_or_default();
    let cam_children: bevy::platform::collections::HashSet<Entity> =
        children.iter_descendants(cam_entity).collect();

    let nearest_object = objects
        .iter()
        .filter(|(entity, ..)| !cam_children.contains(entity))
        .filter(|(.., obj_layer, _)| {
            let obj_layer = obj_layer.unwrap_or_default();
            cam_layer.intersects(obj_layer)
        })
        .filter(|(.., visibility)| visibility.get())
        .map(|(entity, object_local, obj_pos, aabb, ..)| {
            let center_distance =
                obj_pos.translation().as_dvec3() - cam_pos.translation().as_dvec3();
            let nearest_distance = center_distance.length()
                - (aabb.half_extents.as_dvec3() * object_local.scale.as_dvec3())
                    .abs()
                    .min_element();
            (entity, nearest_distance)
        })
        .filter(|v| v.1.is_finite())
        .reduce(|nearest, this| if this.1 < nearest.1 { this } else { nearest });
    camera.nearest_object = nearest_object;
}

/// Uses [`BigSpaceCameraInput`] state to update the camera position.
pub fn camera_controller(
    time: Res<Time>,
    grids: Grids,
    mut input: ResMut<CameraInput>,
    mut camera: Query<(
        Entity,
        &mut CellCoord,
        &mut Transform,
        &mut CameraController,
    )>,
) {
    for (camera, mut cell, mut transform, mut controller) in camera.iter_mut() {
        let Some(grid) = grids.parent_grid(camera) else {
            continue;
        };
        let speed = match (controller.nearest_object, controller.slow_near_objects) {
            (Some(nearest), true) => nearest.1.abs(),
            _ => controller.speed,
        } * (controller.speed + input.boost as usize as f64);

        let [min, max] = controller.speed_bounds;
        let speed = speed.clamp(min, max);

        let lerp_translation = 1.0 - controller.smoothness.clamp(0.0, 0.999);
        let lerp_rotation = 1.0 - controller.rotational_smoothness.clamp(0.0, 0.999);

        let (vel_t_current, vel_r_current) = (controller.vel_translation, controller.vel_rotation);
        let (vel_t_target, vel_r_target) =
            input.target_velocity(&controller, speed, time.delta_secs_f64());

        let cam_rot = transform.rotation.as_dquat();
        let vel_t_next = cam_rot * vel_t_target; // Orients the translation to match the camera
        let vel_t_next = vel_t_current.lerp(vel_t_next, lerp_translation);
        // Convert the high precision translation to a grid cell and low precision translation
        let (cell_offset, new_translation) = grid.translation_to_grid(vel_t_next);
        let new = *cell.bypass_change_detection() + cell_offset;
        cell.set_if_neq(new);
        transform.translation += new_translation;

        let new_rotation = vel_r_current.slerp(vel_r_target, lerp_rotation);
        transform.rotation *= new_rotation.as_quat();

        // Store the new velocity to be used in the next grid
        controller.vel_translation = vel_t_next;
        controller.vel_rotation = new_rotation;

        input.reset();
    }
}
