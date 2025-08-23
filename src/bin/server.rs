use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_rapier3d::prelude::*;
use bevy_renet::{
    netcode::{NetcodeServerPlugin, NetcodeServerTransport, ServerAuthentication, ServerConfig},
    renet::{ConnectionConfig, RenetServer, ServerEvent},
    RenetServerPlugin,
};
use metin2::{
    add_colliders_to_gltf_scene, pump_scene_loads, serialize_server_messages, setup_level,
    ClientChannel, FreeFlyCameraPlugin, NetworkedEntities, Player, PlayerColliderDims,
    PlayerCommand, PlayerHitboxDebugPlugin, PlayerInput, RenetServerVisualizer, SceneLoadQueue,
    ServerChannel, ServerMessages, EXAMPLES, PROTOCOL_ID,
};
use renet::ClientId;
use std::{collections::HashMap, net::UdpSocket, time::SystemTime};

#[derive(Component)]
struct ConnectedPlayer {
    pub id: ClientId,
    pub speed: f32,
}

#[derive(Resource, Default)]
struct Tick(u32);

#[derive(Debug, Component)]
struct Bot {
    auto_cast: Timer,
}

#[derive(Default, Resource)]
struct SavedPositions(HashMap<ClientId, SavedPose>);

#[derive(Clone, Copy, Debug)]
struct SavedPose {
    translation: [f32; 3],
    rotation: [f32; 4],
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
        current_time,
        max_clients: 64,
        protocol_id: PROTOCOL_ID,
        public_addresses: vec![server_addr],
        authentication: ServerAuthentication::Unsecure,
    };
    let transport = NetcodeServerTransport::new(server_config, socket).unwrap();
    let server = RenetServer::new(ConnectionConfig::default());

    app.add_plugins(DefaultPlugins);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(RenetServerPlugin);
    app.add_plugins(EguiPlugin::default());
    app.add_plugins(NetcodeServerPlugin);
    app.add_plugins(FreeFlyCameraPlugin);
    // app.add_plugins(RapierDebugRenderPlugin::default());
    app.add_plugins(PlayerHitboxDebugPlugin);
    app.add_plugins(bevy_atmosphere::plugin::AtmospherePlugin);
    app.add_plugins(bevy_inspector_egui::quick::WorldInspectorPlugin::new());

    app.insert_resource(ServerLobby::default());
    app.insert_resource(Tick(0));
    app.insert_resource(BotId(0));
    app.insert_resource(RenetServerVisualizer::<200>::default());
    app.insert_resource(server);
    app.insert_resource(transport);
    app.insert_resource(Time::<Fixed>::from_hz(30.0));
    app.insert_resource(SavedPositions::default());
    app.insert_resource(SceneLoadQueue::new(EXAMPLES));

    app.add_systems(
        Update,
        (
            server_update_system,
            spawn_bot,
            add_colliders_to_gltf_scene,
            pump_scene_loads,
        ),
    );
    app.add_systems(FixedUpdate, (move_players_system, server_network_sync));
    app.add_systems(Startup, setup_level);

    app.run();
}

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
fn server_update_system(
    mut server_events: EventReader<ServerEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut lobby: ResMut<ServerLobby>,
    mut server: ResMut<RenetServer>,
    mut visualizer: ResMut<RenetServerVisualizer<200>>,
    players: Query<(Entity, &ConnectedPlayer, &Transform)>,
    mut saved: ResMut<SavedPositions>,
) {
    for event in server_events.read() {
        match event {
            ServerEvent::ClientConnected { client_id } => {
                println!("Player {client_id} connected.");
                visualizer.add_client(*client_id);

                for (entity, player, transform) in players.iter() {
                    let translation: [f32; 3] = transform.translation.into();
                    let rotation: [f32; 4] = transform.rotation.into();

                    if let Ok(msg) = postcard::to_allocvec(&ServerMessages::PlayerCreate {
                        id: player.id,
                        entity,
                        translation,
                        rotation,
                    }) {
                        server.send_message(*client_id, ServerChannel::ServerMessages, msg);
                    }
                }

                let transform = if let Some(pose) = saved.0.get(client_id) {
                    Transform {
                        translation: Vec3::from_array(pose.translation),
                        rotation: Quat::from_xyzw(
                            pose.rotation[0],
                            pose.rotation[1],
                            pose.rotation[2],
                            pose.rotation[3],
                        ),
                        ..default()
                    }
                } else {
                    Transform::from_xyz(
                        (fastrand::f32() - 0.5) * 40.,
                        0.51,
                        (fastrand::f32() - 0.5) * 40.,
                    )
                };

                let player_entity = commands
                    .spawn((
                        Mesh3d(meshes.add(Capsule3d::new(0.9, 0.4))),
                        MeshMaterial3d(materials.add(Color::srgb_u8(255, 180, 80))),
                        Player,
                        ConnectedPlayer {
                            id: *client_id,
                            speed: 10.0,
                        },
                        PlayerInput::default(),
                        transform,
                        RigidBody::KinematicPositionBased,
                        Collider::capsule_y(0.9, 0.4),
                        KinematicCharacterController {
                            up: Vec3::Y,
                            offset: CharacterLength::Relative(0.02),
                            autostep: Some(CharacterAutostep {
                                max_height: CharacterLength::Absolute(0.60),
                                min_width: CharacterLength::Absolute(0.20),
                                include_dynamic_bodies: false,
                            }),
                            snap_to_ground: Some(CharacterLength::Absolute(0.50)),
                            max_slope_climb_angle: 89.0_f32.to_radians(),
                            min_slope_slide_angle: 95.0_f32.to_radians(),
                            slide: true,
                            ..default()
                        },
                        Friction::coefficient(0.8),
                        PlayerColliderDims {
                            half_height: 0.4,
                            radius: 0.9,
                        },
                    ))
                    .id();

                lobby.players.insert(*client_id, player_entity);

                let translation: [f32; 3] = transform.translation.into();
                let rotation: [f32; 4] = transform.rotation.into();

                if let Ok(msg) = postcard::to_allocvec(&ServerMessages::PlayerCreate {
                    id: *client_id,
                    entity: player_entity,
                    translation,
                    rotation,
                }) {
                    server.send_message(*client_id, ServerChannel::ServerMessages, msg);
                }
            }
            ServerEvent::ClientDisconnected { client_id, reason } => {
                println!("Player {client_id} disconnected: {reason}");
                visualizer.remove_client(*client_id);

                if let Some(&player_entity) = lobby.players.get(client_id) {
                    if let Ok((_e, _p, tf)) = players.get(player_entity) {
                        saved.0.insert(
                            *client_id,
                            SavedPose {
                                translation: tf.translation.into(),
                                rotation: tf.rotation.into(),
                            },
                        );
                    }
                }

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
    q_player: Query<(Entity, &ConnectedPlayer, &Transform), With<Player>>,
    mut tick: ResMut<Tick>,
    mut saved: ResMut<SavedPositions>,
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

        saved.0.insert(
            player.id,
            SavedPose {
                translation: transform.translation.into(),
                rotation: transform.rotation.into(),
            },
        );
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
        let client_id: ClientId = (1_000_000_000_000u64 + bot_id.0 as u64) as ClientId;
        bot_id.0 += 1;

        let transform = Transform::from_xyz(10.0, 10.0, 10.0);

        let player_entity = commands
            .spawn((
                Mesh3d(meshes.add(Mesh::from(Capsule3d::new(0.9, 0.4)))),
                MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
                Player,
                transform,
                ConnectedPlayer {
                    id: client_id,
                    speed: 10.0,
                },
                Bot {
                    auto_cast: Timer::from_seconds(3.0, TimerMode::Repeating),
                },
                RigidBody::KinematicPositionBased,
                Collider::capsule_y(0.9, 0.4),
                KinematicCharacterController {
                    up: Vec3::Y,
                    offset: CharacterLength::Absolute(0.02),
                    autostep: Some(CharacterAutostep {
                        max_height: CharacterLength::Absolute(0.60),
                        min_width: CharacterLength::Absolute(0.20),
                        include_dynamic_bodies: true,
                    }),
                    snap_to_ground: Some(CharacterLength::Absolute(0.50)),
                    max_slope_climb_angle: 89.0_f32.to_radians(),
                    min_slope_slide_angle: 95.0_f32.to_radians(),
                    slide: true,
                    ..default()
                },
                Friction::coefficient(0.8),
                PlayerColliderDims {
                    half_height: 0.4,
                    radius: 0.9,
                },
            ))
            .id();

        lobby.players.insert(client_id, player_entity);

        let translation: [f32; 3] = transform.translation.into();
        let rotation: [f32; 4] = transform.rotation.into();

        if let Ok(msg) = postcard::to_allocvec(&ServerMessages::PlayerCreate {
            entity: player_entity,
            id: client_id,
            translation,
            rotation,
        }) {
            server.broadcast_message(ServerChannel::ServerMessages, msg);
        }
    }
}

fn move_players_system(
    mut query: Query<(
        &PlayerInput,
        &ConnectedPlayer,
        &mut KinematicCharacterController,
        &mut Transform,
    )>,
    time: Res<Time<Fixed>>,
) {
    let dt = time.delta_secs();

    for (input, player, mut kcc, mut tf) in query.iter_mut() {
        let v = Vec2::new(input.dir[0], input.dir[1]);
        let dir = if v.length_squared() > 0.0 {
            v.normalize()
        } else {
            Vec2::ZERO
        };

        let desired = Vec3::new(
            dir.x * player.speed * dt,
            -9.81 * dt,
            dir.y * player.speed * dt,
        );
        kcc.translation = Some(desired);

        let yaw = dir.x.atan2(-dir.y);
        tf.rotation = Quat::from_rotation_y(yaw);
    }
}
