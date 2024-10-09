use crate::MainState;

use bevy::prelude::*;

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
    commands.spawn((
        StateScoped(MainState::Loading),
        Camera3dBundle {
            camera: Camera {
                order: isize::MAX,
                ..default()
            },
            ..default()
        },
    ));

    commands
        .spawn((
            StateScoped(MainState::Loading),
            NodeBundle {
                style: Style {
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::Center,
                    ..default()
                },
                ..default()
            },
            LoadingScreen,
        ))
        .with_children(|parent| {
            parent.spawn((
                TextBundle::from_section(
                    "Loading...",
                    TextStyle {
                        font_size: 40.0,
                        color: Color::WHITE,
                        ..default()
                    },
                ),
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
            text.sections[0].value = format!("Loaded {}", path);
        }
    }
}
