use std::{
    collections::{HashMap, VecDeque},
    net::UdpSocket,
    time::SystemTime,
};

use bevy::{
    input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel},
    prelude::*,
    window::WindowCloseRequested,
};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_rapier3d::prelude::*;
use bevy_renet::{
    client_connected,
    netcode::{
        ClientAuthentication, NetcodeClientPlugin, NetcodeClientTransport, NetcodeTransportError,
    },
    RenetClientPlugin,
};
use metin2::{
    add_colliders_to_gltf_scene, setup_level, ClientChannel, NetworkedEntities, Player,
    PlayerColliderDims, PlayerCommand, PlayerHitboxDebugPlugin, PlayerInput, RenetClientVisualizer,
    RenetVisualizerStyle, ServerChannel, ServerMessages, PROTOCOL_ID,
};
use renet::{ClientId, ConnectionConfig, RenetClient};

#[derive(Clone, Copy, Default, Eq, PartialEq, Debug, Hash, States)]
enum GameState {
    #[default]
    Splash,
    Menu,
    Game,
}

#[derive(Component)]
struct ControlledPlayer;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Connected;

#[derive(Debug, Resource)]
struct CurrentClientId(u64);

#[derive(Resource)]
struct OrbitSettings {
    distance: f32,
    min_distance: f32,
    max_distance: f32,
    zoom_speed: f32,
    yaw: f32,
    pitch: f32,
    min_pitch: f32,
    max_pitch: f32,
    mouse_sensitivity: f32,
    rotate_speed: f32,
}

#[derive(Debug)]
struct PlayerInfo {
    client_entity: Entity,
    server_entity: Entity,
}

#[derive(Debug, Default, Resource)]
struct ClientLobby {
    players: HashMap<ClientId, PlayerInfo>,
}

#[derive(Default, Resource)]
struct NetworkMapping(HashMap<Entity, Entity>);

#[derive(Resource)]
struct SnapshotBuffer {
    latest_tick: u32,
    render_tick: f32,
    server_hz: f32,
    initialized: bool,
    by_entity: HashMap<Entity, VecDeque<(u32, Vec3, Quat)>>,
}

impl Default for SnapshotBuffer {
    fn default() -> Self {
        Self {
            latest_tick: 0,
            render_tick: 0.0,
            server_hz: 30.0,
            initialized: false,
            by_entity: HashMap::default(),
        }
    }
}

const INTERP_DELAY_TICKS: u32 = 2;
const SERVER_HZ: f32 = 30.0;
const PLAYER_STEP_PER_TICK: f32 = 0.2;
const PLAYER_SPEED_PER_SEC: f32 = PLAYER_STEP_PER_TICK * SERVER_HZ;

fn main() {
    let mut app = App::new();

    let client = RenetClient::new(ConnectionConfig::default());
    let server_addr = "127.0.0.1:5000".parse().unwrap();
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    let current_time = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    let authentication = ClientAuthentication::Unsecure {
        client_id: 0,
        protocol_id: PROTOCOL_ID,
        server_addr,
        user_data: None,
    };
    let transport = NetcodeClientTransport::new(current_time, authentication, socket).unwrap();

    app.add_plugins(DefaultPlugins);
    app.add_plugins(RenetClientPlugin);
    app.add_plugins(NetcodeClientPlugin);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(EguiPlugin::default());
    // app.add_plugins(RapierDebugRenderPlugin::default());
    app.add_plugins(PlayerHitboxDebugPlugin);
    app.add_plugins(bevy_atmosphere::plugin::AtmospherePlugin);

    app.configure_sets(Update, Connected.run_if(client_connected));

    app.init_state::<GameState>();

    app.insert_resource(client);
    app.insert_resource(transport);
    app.insert_resource(CurrentClientId(0));
    app.insert_resource(SnapshotBuffer::default());
    app.insert_resource(OrbitSettings {
        distance: 10.0,
        min_distance: 3.0,
        max_distance: 25.0,
        yaw: 0.0,
        pitch: 0.0,
        min_pitch: 0.0,
        max_pitch: 1.4,
        mouse_sensitivity: 0.01,
        zoom_speed: 1.0,
        rotate_speed: 1.0,
    });

    app.add_event::<PlayerCommand>();
    app.insert_resource(RenetClientVisualizer::<200>::new(
        RenetVisualizerStyle::default(),
    ));
    app.insert_resource(ClientLobby::default());
    app.insert_resource(PlayerInput::default());
    app.insert_resource(NetworkMapping::default());

    app.add_systems(
        Update,
        predict_local_player
            .in_set(Connected)
            .before(interpolate_snapshots),
    );
    app.add_systems(
        Update,
        (
            interpolate_snapshots,
            zoom_camera_wheel,
            rotate_orbit_with_mouse,
            player_input,
            update_orbit_camera,
            panic_on_error_system,
            client_send_input,
            client_send_player_commands,
            client_sync_players,
        )
            .in_set(Connected),
    );
    app.add_systems(
        Update,
        (
            update_visulizer_system,
            add_colliders_to_gltf_scene,
            cmd_q_to_app_exit,
        ),
    );
    app.add_systems(Update, disconnect_on_close.after(cmd_q_to_app_exit));
    app.add_systems(Startup, (setup, setup_level));

    app.run();
}

mod splash {
    use bevy::prelude::*;

    pub struct SplashPlugin;

    impl Plugin for SplashPlugin {
        fn build(&self, app: &mut App) {}
    }
}

mod menu {
    use bevy::prelude::*;

    pub struct MenuPlugin;

    impl Plugin for MenuPlugin {
        fn build(&self, app: &mut App) {}
    }
}

mod game {
    use bevy::prelude::*;

    pub struct GamePlugin;

    impl Plugin for GamePlugin {
        fn build(&self, app: &mut App) {}
    }
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        bevy_atmosphere::plugin::AtmosphereCamera::default(),
    ));
}

fn update_visulizer_system(
    mut egui_contexts: EguiContexts,
    mut visualizer: ResMut<RenetClientVisualizer<200>>,
    client: Res<RenetClient>,
    mut show_visualizer: Local<bool>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    visualizer.add_network_info(client.network_info());
    if keyboard_input.just_pressed(KeyCode::F1) {
        *show_visualizer = !*show_visualizer;
    }
    if *show_visualizer {
        let ctx = egui_contexts.ctx_mut().unwrap();
        visualizer.show_window(ctx);
    }
}

fn player_input(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut player_input: ResMut<PlayerInput>,
    q_cam: Query<&Transform, With<Camera3d>>,
    q_player: Query<&Transform, With<ControlledPlayer>>,
    mut player_commands: EventWriter<PlayerCommand>,
) {
    let Ok(cam_tf) = q_cam.single() else {
        return;
    };
    let f = {
        let v = cam_tf.forward();
        Vec3::new(v.x, 0.0, v.z).normalize_or_zero()
    };
    let r = {
        let v = cam_tf.right();
        Vec3::new(v.x, 0.0, v.z).normalize_or_zero()
    };

    let mut wish = Vec3::ZERO;
    if keyboard_input.pressed(KeyCode::KeyW) || keyboard_input.pressed(KeyCode::ArrowUp) {
        wish += f;
    }
    if keyboard_input.pressed(KeyCode::KeyS) || keyboard_input.pressed(KeyCode::ArrowDown) {
        wish -= f;
    }
    if keyboard_input.pressed(KeyCode::KeyA) || keyboard_input.pressed(KeyCode::ArrowLeft) {
        wish -= r;
    }
    if keyboard_input.pressed(KeyCode::KeyD) || keyboard_input.pressed(KeyCode::ArrowRight) {
        wish += r;
    }

    let dir3 = wish.normalize_or_zero();
    player_input.dir = [dir3.x, dir3.z];

    if let Ok(player_tf) = q_player.single() {
        if keyboard_input.just_pressed(KeyCode::Space) {
            player_commands.write(PlayerCommand::BasicAttack {
                cast_at: player_tf.translation,
            });
        }
    }
}

fn client_send_input(player_input: Res<PlayerInput>, mut client: ResMut<RenetClient>) {
    let msg = postcard::to_allocvec(&*player_input).unwrap();
    client.send_message(ClientChannel::Input, msg);
}

fn client_send_player_commands(
    mut player_commands: EventReader<PlayerCommand>,
    mut client: ResMut<RenetClient>,
) {
    for command in player_commands.read() {
        let msg = postcard::to_allocvec(command).unwrap();
        client.send_message(ClientChannel::Command, msg);
    }
}

#[allow(clippy::too_many_arguments)]
fn client_sync_players(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut client: ResMut<RenetClient>,
    client_id: Res<CurrentClientId>,
    mut lobby: ResMut<ClientLobby>,
    mut network_mapping: ResMut<NetworkMapping>,
    mut buffer: ResMut<SnapshotBuffer>,
    mut q_tf: Query<&mut Transform>,
) {
    let client_id = client_id.0;

    while let Some(message) = client.receive_message(ServerChannel::ServerMessages) {
        let server_message = postcard::from_bytes(&message).unwrap();
        match server_message {
            ServerMessages::PlayerCreate {
                id,
                translation,
                entity: server_entity,
                rotation,
            } => {
                println!("Player {id} connected.");

                let transform = Transform {
                    translation: translation.into(),
                    rotation: Quat::from_xyzw(rotation[0], rotation[1], rotation[2], rotation[3]),
                    ..default()
                };

                let mut player = commands.spawn((
                    Player,
                    Mesh3d(meshes.add(Capsule3d::new(0.9, 0.4))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(255, 180, 80))),
                    transform,
                    PlayerColliderDims {
                        half_height: 0.4,
                        radius: 0.9,
                    },
                ));

                if client_id == id {
                    player.insert(ControlledPlayer);
                }

                let client_entity = player.id();

                network_mapping.0.insert(server_entity, client_entity);

                lobby.players.insert(
                    id,
                    PlayerInfo {
                        server_entity,
                        client_entity,
                    },
                );
            }
            ServerMessages::PlayerRemove { id } => {
                println!("Player {id} disconnected.");
                if let Some(info) = lobby.players.remove(&id) {
                    commands.entity(info.client_entity).despawn();
                    network_mapping.0.remove(&info.server_entity);
                    buffer.by_entity.remove(&info.client_entity);
                }
            }
        }
    }

    while let Some(message) = client.receive_message(ServerChannel::NetworkedEntities) {
        let networked_entities: NetworkedEntities = postcard::from_bytes(&message).unwrap();

        if !buffer.initialized {
            buffer.render_tick = networked_entities.tick as f32 - INTERP_DELAY_TICKS as f32;
            buffer.initialized = true;
        }
        buffer.latest_tick = buffer.latest_tick.max(networked_entities.tick);

        for i in 0..networked_entities.entities.len() {
            let server_entity = networked_entities.entities[i];
            let translation = networked_entities.translations[i].into();
            let rotation = Quat::from_xyzw(
                networked_entities.rotations[i][0],
                networked_entities.rotations[i][1],
                networked_entities.rotations[i][2],
                networked_entities.rotations[i][3],
            );

            if let Some(&client_entity) = network_mapping.0.get(&server_entity) {
                let is_local = lobby
                    .players
                    .get(&client_id)
                    .map(|p| p.client_entity == client_entity)
                    .unwrap_or(false);

                if is_local {
                    if let Ok(mut tf) = q_tf.get_mut(client_entity) {
                        let k = 0.15;
                        tf.translation = tf.translation.lerp(translation, k);
                        tf.rotation = tf.rotation.slerp(rotation, k);
                    }
                } else {
                    let dq = buffer.by_entity.entry(client_entity).or_default();
                    if let Some(&(last_tick, _, _)) = dq.back() {
                        if networked_entities.tick <= last_tick {
                            continue;
                        }
                    }

                    dq.push_back((networked_entities.tick, translation, rotation));

                    while dq.len() > 32 {
                        dq.pop_front();
                    }
                }
            }
        }
    }
}

fn update_orbit_camera(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut orbit: ResMut<OrbitSettings>,
    q_player: Query<&Transform, With<ControlledPlayer>>,
    mut q_cam: Query<&mut Transform, (With<Camera3d>, Without<ControlledPlayer>)>,
) {
    let Ok(player_tf) = q_player.single() else {
        return;
    };

    let Ok(mut cam_tf) = q_cam.single_mut() else {
        return;
    };

    if keys.pressed(KeyCode::KeyQ) {
        orbit.yaw -= orbit.rotate_speed * time.delta_secs();
    }

    if keys.pressed(KeyCode::KeyE) {
        orbit.yaw += orbit.rotate_speed * time.delta_secs();
    }

    let player_pos = player_tf.translation;
    let (cp, sp) = (orbit.pitch.cos(), orbit.pitch.sin());
    let (cy, sy) = (orbit.yaw.cos(), orbit.yaw.sin());

    let offset = Vec3::new(
        orbit.distance * cp * cy,
        orbit.distance * sp,
        orbit.distance * cp * sy,
    );

    cam_tf.translation = player_pos + offset;

    if cam_tf.translation.y < 0.001 {
        cam_tf.translation.y = 0.001;
    }

    cam_tf.look_at(player_pos, Vec3::Y);
}

#[allow(clippy::never_loop)]
fn panic_on_error_system(mut renet_error: EventReader<NetcodeTransportError>) {
    for e in renet_error.read() {
        panic!("Renet transport error: {e}");
    }
}

fn zoom_camera_wheel(mut mouse_wheels: EventReader<MouseWheel>, mut orbit: ResMut<OrbitSettings>) {
    let mut scroll = 0.0;

    for ev in mouse_wheels.read() {
        let delta = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y / 100.0,
        };

        scroll += delta;
    }

    if scroll != 0.0 {
        orbit.distance = (orbit.distance - scroll * orbit.zoom_speed)
            .clamp(orbit.min_distance, orbit.max_distance);
    }
}

fn rotate_orbit_with_mouse(
    mut mouse_motion: EventReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut orbit: ResMut<OrbitSettings>,
) {
    if !mouse_buttons.pressed(MouseButton::Right) {
        return;
    }

    let mut delta = Vec2::ZERO;
    for ev in mouse_motion.read() {
        delta += ev.delta;
    }
    if delta == Vec2::ZERO {
        return;
    }

    orbit.yaw += delta.x * orbit.mouse_sensitivity;
    orbit.pitch += delta.y * orbit.mouse_sensitivity;
    orbit.pitch = orbit.pitch.clamp(orbit.min_pitch, orbit.max_pitch);
}

fn interpolate_snapshots(
    mut q_tf: Query<&mut Transform>,
    mut buffer: ResMut<SnapshotBuffer>,
    time: Res<Time>,
    lobby: Res<ClientLobby>,
    client_id: Res<CurrentClientId>,
) {
    if !buffer.initialized {
        return;
    }

    let target = buffer.latest_tick as f32 - INTERP_DELAY_TICKS as f32;
    let step = buffer.server_hz * time.delta_secs();

    buffer.render_tick = if buffer.render_tick < target {
        (buffer.render_tick + step).min(target)
    } else {
        target
    };
    let rt = buffer.render_tick;

    let local_entity = lobby.players.get(&client_id.0).map(|p| p.client_entity);

    for (entity, dq) in buffer.by_entity.iter() {
        if Some(*entity) == local_entity {
            continue;
        }

        if let Ok(mut tf) = q_tf.get_mut(*entity) {
            let mut prev = None;
            let mut next = None;

            for (tick, pos, rot) in dq.iter() {
                if *tick as f32 <= rt {
                    prev = Some((*tick, *pos, *rot));
                }
                if *tick as f32 >= rt {
                    next = Some((*tick, *pos, *rot));
                    break;
                }
            }

            match (prev, next) {
                (Some((pt, ppos, prot)), Some((nt, npos, nrot))) if nt > pt => {
                    let dt = (nt - pt) as f32;
                    let t = ((rt - pt as f32) / dt).clamp(0.0, 1.0);
                    tf.translation = ppos.lerp(npos, t);
                    tf.rotation = prot.slerp(nrot, t);
                }
                (Some((_pt, ppos, prot)), None) => {
                    tf.translation = ppos;
                    tf.rotation = prot;
                }
                (None, Some((_nt, npos, nrot))) => {
                    tf.translation = npos;
                    tf.rotation = nrot;
                }
                _ => {}
            }
        }
    }
}

fn predict_local_player(
    time: Res<Time>,
    input: Res<PlayerInput>,
    mut q: Query<&mut Transform, With<ControlledPlayer>>,
) {
    let dt = time.delta_secs();
    let dir = Vec2::new(input.dir[0], input.dir[1]).normalize_or_zero();

    for mut tf in &mut q {
        tf.translation.x += dir.x * PLAYER_SPEED_PER_SEC * dt;
        tf.translation.z += dir.y * PLAYER_SPEED_PER_SEC * dt;

        if dir.length_squared() > 0.0 {
            let yaw = dir.x.atan2(-dir.y);
            tf.rotation = Quat::from_rotation_y(yaw);
        }
    }
}

fn disconnect_on_close(
    mut ev_exit: EventReader<AppExit>,
    mut ev_win_close: EventReader<WindowCloseRequested>,
    mut client: ResMut<RenetClient>,
    mut transport: ResMut<NetcodeClientTransport>,
) {
    let app_exiting = ev_exit.read().next().is_some();
    let win_closing = ev_win_close.read().next().is_some();
    if app_exiting || win_closing {
        // zuerst Transport, dann Client
        transport.disconnect();
        client.disconnect();
    }
}

fn cmd_q_to_app_exit(keys: Res<ButtonInput<KeyCode>>, mut exit_writer: EventWriter<AppExit>) {
    #[cfg(target_os = "macos")]
    {
        let super_down = keys.any_pressed([KeyCode::SuperLeft, KeyCode::SuperRight]);
        if super_down && keys.just_pressed(KeyCode::KeyQ) {
            exit_writer.write(AppExit::Success);
        }
    }
}
