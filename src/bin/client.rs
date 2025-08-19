use std::{
    collections::{HashMap, VecDeque},
    net::UdpSocket,
    time::SystemTime,
};

use bevy::{
    input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel},
    prelude::*,
};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_renet::{
    client_connected, client_just_connected, client_just_disconnected,
    netcode::{
        ClientAuthentication, NetcodeClientPlugin, NetcodeClientTransport, NetcodeTransportError,
    },
    RenetClientPlugin,
};
use metin2::{
    setup_level, ClientChannel, NetworkedEntities, PlayerCommand, PlayerInput,
    RenetClientVisualizer, RenetVisualizerStyle, ServerChannel, ServerMessages, PROTOCOL_ID,
};
use renet::{ClientId, ConnectionConfig, RenetClient};

#[derive(Component)]
struct Player;

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

#[derive(Default, Resource)]
struct LocalClientEntity(Option<Entity>);

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

    app.add_plugins(DefaultPlugins);
    app.add_plugins(RenetClientPlugin);
    app.add_plugins(EguiPlugin::default());
    app.add_plugins(NetcodeClientPlugin);
    app.configure_sets(Update, Connected.run_if(client_connected));

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

    app.insert_resource(client);
    app.insert_resource(transport);
    app.insert_resource(CurrentClientId(0));
    app.insert_resource(SnapshotBuffer::default());
    app.insert_resource(LocalClientEntity::default());
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
    app.insert_resource(ClientLobby::default());
    app.insert_resource(PlayerInput::default());
    app.insert_resource(NetworkMapping::default());

    app.add_systems(
        Update,
        (
            zoom_camera_wheel,
            rotate_orbit_with_mouse,
            player_input,
            update_orbit_camera,
            panic_on_error_system,
            graceful_disconnect_on_exit,
        ),
    );
    app.add_systems(Update, predict_local_player.before(interpolate_snapshots));

    app.add_systems(
        Update,
        (
            client_send_input,
            client_send_player_commands,
            client_sync_players,
        )
            .in_set(Connected),
    );
    app.add_systems(Update, on_connect.run_if(client_just_connected));
    app.add_systems(Update, on_disconnect.run_if(client_just_disconnected));

    app.insert_resource(RenetClientVisualizer::<200>::new(
        RenetVisualizerStyle::default(),
    ));

    app.add_systems(Update, update_visulizer_system);
    app.add_systems(Startup, (setup_level, setup));

    app.run();
}

fn on_disconnect() {}

fn on_connect() {}

fn graceful_disconnect_on_exit(
    mut exit_events: EventReader<AppExit>,
    mut client: ResMut<RenetClient>,
    mut transport: ResMut<NetcodeClientTransport>,
) {
    if exit_events.read().next().is_some() {
        transport.disconnect();
        client.disconnect();
    }
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
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
    mut q_player: Query<&Transform, With<Player>>,
    mut player_commands: EventWriter<PlayerCommand>,
) {
    player_input.left =
        keyboard_input.pressed(KeyCode::KeyA) || keyboard_input.pressed(KeyCode::ArrowLeft);
    player_input.right =
        keyboard_input.pressed(KeyCode::KeyD) || keyboard_input.pressed(KeyCode::ArrowRight);
    player_input.up =
        keyboard_input.pressed(KeyCode::KeyW) || keyboard_input.pressed(KeyCode::ArrowUp);
    player_input.down =
        keyboard_input.pressed(KeyCode::KeyS) || keyboard_input.pressed(KeyCode::ArrowDown);

    let Ok(player_tf) = q_player.single_mut() else {
        return;
    };

    if keyboard_input.just_pressed(KeyCode::Space) {
        player_commands.write(PlayerCommand::BasicAttack {
            cast_at: player_tf.translation,
        });
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

fn client_sync_players(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut client: ResMut<RenetClient>,
    client_id: Res<CurrentClientId>,
    mut lobby: ResMut<ClientLobby>,
    mut network_mapping: ResMut<NetworkMapping>,
    mut buffer: ResMut<SnapshotBuffer>,
    mut local: ResMut<LocalClientEntity>,
    mut q_tf: Query<&mut Transform>,
) {
    let client_id = client_id.0;
    while let Some(message) = client.receive_message(ServerChannel::ServerMessages) {
        let server_message = postcard::from_bytes(&message).unwrap();
        match server_message {
            ServerMessages::PlayerCreate {
                id,
                translation,
                entity,
            } => {
                println!("Player {} connected.", id);

                let mut player = commands.spawn((
                    Player,
                    Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(255, 180, 80))),
                    Transform::from_xyz(translation[0], translation[1], translation[2]),
                ));

                let player_id = player.id();

                if client_id == id {
                    player.insert(ControlledPlayer);
                    local.0 = Some(player_id);
                }

                let player_info = PlayerInfo {
                    server_entity: entity,
                    client_entity: player_id,
                };

                lobby.players.insert(id, player_info);

                network_mapping.0.insert(entity, player_id);
            }
            ServerMessages::PlayerRemove { id } => {
                println!("Player {} disconnected.", id);
                if let Some(PlayerInfo {
                    server_entity,
                    client_entity,
                }) = lobby.players.remove(&id)
                {
                    commands.entity(client_entity).despawn();
                    network_mapping.0.remove(&server_entity);
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
            let player_id = networked_entities.player_ids[i];
            let server_entity = networked_entities.entities[i];
            let translation = networked_entities.translations[i].into();
            let rotation = Quat::from_xyzw(
                networked_entities.rotations[i][0],
                networked_entities.rotations[i][1],
                networked_entities.rotations[i][2],
                networked_entities.rotations[i][3],
            );

            if let Some(&client_entity) = network_mapping.0.get(&server_entity) {
                if local.0 == Some(client_entity) {
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
            } else {
                let mut player = commands.spawn((
                    Player,
                    Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(255, 180, 80))),
                    Transform::from_xyz(translation[0], translation[1], translation[2]),
                ));

                if player_id == client_id {
                    player.insert(ControlledPlayer);
                }

                network_mapping.0.insert(server_entity, player.id());
            }
        }
    }
}

fn update_orbit_camera(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut orbit: ResMut<OrbitSettings>,
    q_player: Query<&Transform, With<Player>>,
    mut q_cam: Query<&mut Transform, (With<Camera3d>, Without<Player>)>,
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
        panic!("Renet transport error: {:?}", e);
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
    local: Res<LocalClientEntity>,
) {
    if !buffer.initialized {
        return;
    }

    let target = buffer.latest_tick as f32 - INTERP_DELAY_TICKS as f32;
    let step = buffer.server_hz * time.delta_secs();

    if buffer.render_tick < target {
        buffer.render_tick = (buffer.render_tick + step).min(target);
    } else {
        buffer.render_tick = target;
    }

    let rt = buffer.render_tick;

    for (entity, dq) in buffer.by_entity.iter() {
        if local.0 == Some(*entity) {
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
    let x = (input.right as i8 - input.left as i8) as f32;
    let y = (input.down as i8 - input.up as i8) as f32;
    let dir = Vec2::new(x, y).normalize_or_zero();

    for mut tf in &mut q {
        tf.translation.x += dir.x * PLAYER_SPEED_PER_SEC * dt;
        tf.translation.z += dir.y * PLAYER_SPEED_PER_SEC * dt;

        if dir.length_squared() > 0.0 {
            let yaw = dir.x.atan2(-dir.y);
            tf.rotation = Quat::from_rotation_y(yaw);
        }
    }
}
