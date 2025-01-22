use crate::MainState;

use bevy::prelude::*;

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LoadingScreenPlugin;

impl Plugin for LoadingScreenPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(MainState::Loading), add_loading_screen);
    }
}

#[derive(Component)]
pub struct LoadingScreen;

#[derive(Component)]
pub struct LoadingText;

fn add_loading_screen(mut commands: Commands) {
    let camera = commands
        .spawn((
            StateScoped(MainState::Loading),
            Camera3d::default(),
            Camera {
                order: isize::MAX,
                ..default()
            },
        ))
        .id();

    commands
        .spawn((
            StateScoped(MainState::Loading),
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,

                ..default()
            },
            BackgroundColor(Color::BLACK),
            LoadingScreen,
            TargetCamera(camera),
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new("Loading..."),
                TextFont::from_font_size(32.0),
                TextColor(Color::WHITE),
                LoadingText,
            ));
        });
}

pub fn text_on_load<A: Asset>(
    asset_server: Res<AssetServer>,
    mut events: EventReader<AssetEvent<A>>,
    mut query: Query<&mut Text, With<LoadingText>>,
) {
    let loaded_path = events.read().find_map(|event| match event {
        AssetEvent::LoadedWithDependencies { id } => asset_server
            .get_id_handle(*id)
            .and_then(|handle| handle.path().cloned()),
        _ => None,
    });

    if let Some(path) = loaded_path {
        for mut text in query.iter_mut() {
            **text = format!("Loaded {}", path);
        }
    }
}
