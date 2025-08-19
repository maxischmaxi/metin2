use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_renet::{
    netcode::{NetcodeServerPlugin, NetcodeServerTransport, ServerAuthentication, ServerConfig},
    renet::{ConnectionConfig, RenetServer, ServerEvent},
    RenetServerPlugin,
};
use metin2::{
    serialize_server_messages, setup_level, ClientChannel, NetworkedEntities, PlayerCommand,
    PlayerInput, RenetServerVisualizer, ServerChannel, ServerMessages, PROTOCOL_ID,
};
use renet::ClientId;
use std::{collections::HashMap, net::UdpSocket, time::SystemTime};

#[derive(Component)]
pub struct Player {
    pub id: ClientId,
    pub speed: f32,
}

#[derive(Resource, Default)]
struct Tick(u32);

#[derive(Debug, Component)]
struct Bot {
    auto_cast: Timer,
}

#[derive(Debug, Default, Resource)]
struct ServerLobby {
    players: HashMap<ClientId, Entity>,
}

#[derive(Debug, Resource)]
struct BotId(u32);

fn main() {
    let mut app = App::new();
    let server_addr = "127.0.0.1:5000".parse().unwrap();
    let socket = UdpSocket::bind(server_addr).unwrap();
    let current_time = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    let server_config = ServerConfig {
        current_time: current_time,
        max_clients: 64,
        protocol_id: PROTOCOL_ID,
        public_addresses: vec![server_addr],
        authentication: ServerAuthentication::Unsecure,
    };
    let transport = NetcodeServerTransport::new(server_config, socket).unwrap();
    let server = RenetServer::new(ConnectionConfig::default());

    app.add_plugins(DefaultPlugins);
    app.add_plugins(RenetServerPlugin);
    app.add_plugins(EguiPlugin::default());
    app.add_plugins(NetcodeServerPlugin);

    app.insert_resource(ServerLobby::default());
    app.insert_resource(Tick(0));
    app.insert_resource(BotId(0));
    app.insert_resource(RenetServerVisualizer::<200>::default());
    app.insert_resource(server);
    app.insert_resource(transport);
    app.insert_resource(Time::<Fixed>::from_hz(30.0));

    app.add_systems(Update, (server_update_system, spawn_bot));
    app.add_systems(FixedUpdate, (move_players_system, server_network_sync));

    app.add_systems(Startup, (setup_level, setup_simple_camera));

    app.run();
}

fn server_update_system(
    mut server_events: EventReader<ServerEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut lobby: ResMut<ServerLobby>,
    mut server: ResMut<RenetServer>,
    mut visualizer: ResMut<RenetServerVisualizer<200>>,
    players: Query<(Entity, &Player, &Transform)>,
    tick: Res<Tick>,
) {
    for event in server_events.read() {
        match event {
            ServerEvent::ClientConnected { client_id } => {
                println!("Player {} connected.", client_id);
                visualizer.add_client(*client_id);

                for (entity, player, transform) in players.iter() {
                    let translation: [f32; 3] = transform.translation.into();
                    if let Ok(msg) = postcard::to_allocvec(&ServerMessages::PlayerCreate {
                        id: player.id,
                        entity,
                        translation,
                    }) {
                        server.send_message(*client_id, ServerChannel::ServerMessages, msg);
                    }
                }

                let transform = Transform::from_xyz(
                    (fastrand::f32() - 0.5) * 40.,
                    0.51,
                    (fastrand::f32() - 0.5) * 40.,
                );

                let player_entity = commands
                    .spawn((
                        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
                        MeshMaterial3d(materials.add(Color::srgb_u8(255, 180, 80))),
                        Player {
                            id: *client_id,
                            speed: 5.0,
                        },
                        PlayerInput::default(),
                        transform,
                    ))
                    .id();

                lobby.players.insert(*client_id, player_entity);

                let translation: [f32; 3] = transform.translation.into();
                let rotations: [f32; 4] = transform.rotation.into();

                if let Ok(msg) = postcard::to_allocvec(&NetworkedEntities {
                    tick: tick.0,
                    entities: vec![player_entity],
                    translations: vec![translation],
                    rotations: vec![rotations],
                    player_ids: vec![*client_id],
                }) {
                    server.send_message(*client_id, ServerChannel::NetworkedEntities, msg);
                }
            }
            ServerEvent::ClientDisconnected { client_id, reason } => {
                println!("Player {} disconnected: {}", client_id, reason);
                visualizer.remove_client(*client_id);
                if let Some(player_entity) = lobby.players.remove(client_id) {
                    commands.entity(player_entity).despawn();
                }

                let message =
                    serialize_server_messages(&ServerMessages::PlayerRemove { id: *client_id })
                        .unwrap();
                server.broadcast_message(ServerChannel::ServerMessages, message);
            }
        }
    }

    for client_id in server.clients_id() {
        while let Some(message) = server.receive_message(client_id, ClientChannel::Command) {
            let command: PlayerCommand = postcard::from_bytes(&message).unwrap();
            match command {
                PlayerCommand::BasicAttack { mut cast_at } => {}
            }
        }
        while let Some(message) = server.receive_message(client_id, ClientChannel::Input) {
            let input: PlayerInput = postcard::from_bytes(&message).unwrap();
            if let Some(player_entity) = lobby.players.get(&client_id) {
                commands.entity(*player_entity).insert(input);
            }
        }
    }
}

#[allow(clippy::type_complexity)]
fn server_network_sync(
    mut server: ResMut<RenetServer>,
    q_player: Query<(Entity, &Player, &Transform), With<Player>>,
    mut tick: ResMut<Tick>,
) {
    tick.0 = tick.0.wrapping_add(1);

    let mut ne = NetworkedEntities {
        tick: tick.0,
        ..default()
    };

    for (entity, player, transform) in q_player.iter() {
        ne.entities.push(entity);
        ne.translations.push(transform.translation.into());
        ne.rotations.push(transform.rotation.into());
        ne.player_ids.push(player.id);
    }

    let msg = postcard::to_allocvec(&ne).unwrap();
    server.broadcast_message(ServerChannel::NetworkedEntities, msg);
}

fn spawn_bot(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut lobby: ResMut<ServerLobby>,
    mut server: ResMut<RenetServer>,
    mut bot_id: ResMut<BotId>,
    mut commands: Commands,
) {
    if keyboard_input.just_pressed(KeyCode::Space) {
        let client_id: ClientId = bot_id.0 as ClientId;
        bot_id.0 += 1;

        let transform = Transform::from_xyz(
            (fastrand::f32() - 0.5) * 40.,
            0.51,
            (fastrand::f32() - 0.5) * 40.,
        );

        let player_entity = commands
            .spawn((
                Mesh3d(meshes.add(Mesh::from(Capsule3d::default()))),
                MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
                transform,
            ))
            .insert(Player {
                id: client_id,
                speed: 5.0,
            })
            .insert(Bot {
                auto_cast: Timer::from_seconds(3.0, TimerMode::Repeating),
            })
            .id();

        lobby.players.insert(client_id, player_entity);

        let translation: [f32; 3] = transform.translation.into();

        if let Ok(msg) = postcard::to_allocvec(&ServerMessages::PlayerCreate {
            entity: player_entity,
            id: client_id,
            translation: translation,
        }) {
            server.broadcast_message(ServerChannel::ServerMessages, msg);
        }
    }
}

fn move_players_system(
    mut query: Query<(&PlayerInput, &Player, &mut Transform)>,
    time: Res<Time<Fixed>>,
) {
    let dt = time.delta_secs();

    for (input, player, mut tf) in query.iter_mut() {
        let x = (input.right as i8 - input.left as i8) as f32;
        let y = (input.down as i8 - input.up as i8) as f32;
        let direction = Vec2::new(x, y).normalize_or_zero();

        tf.translation.x += direction.x * player.speed * dt;
        tf.translation.z += direction.y * player.speed * dt;

        let yaw = direction.x.atan2(-direction.y);
        tf.rotation = Quat::from_rotation_y(yaw);
    }
}

fn setup_simple_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-20.5, 30.0, 20.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
