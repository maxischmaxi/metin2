use bevy::{input::mouse::MouseMotion, pbr::CascadeShadowConfigBuilder, prelude::*};
use bevy_rapier3d::prelude::{
    Collider, ComputedColliderShape, Friction, Restitution, RigidBody, TriMeshFlags,
};
use egui::{
    epaint::{PathShape, RectShape},
    pos2, remap, vec2, Color32, CornerRadius, Rect, Rgba, RichText, Sense, Shape, Stroke,
    StrokeKind, TextStyle, Vec2, WidgetText,
};
use renet::{ChannelConfig, ClientId, NetworkInfo, RenetServer, SendType};
use serde::{Deserialize, Serialize};
use std::{
    collections::{HashMap, VecDeque},
    time::Duration,
};

#[derive(Component)]
pub struct Player;

pub const PROTOCOL_ID: u64 = 7;

#[derive(Debug, Serialize, Deserialize, Event)]
pub enum PlayerCommand {
    BasicAttack { cast_at: Vec3 },
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize, Component, Resource)]
pub struct PlayerInput {
    pub dir: [f32; 2],
}

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct NetworkedEntities {
    pub tick: u32,
    pub entities: Vec<Entity>,
    pub translations: Vec<[f32; 3]>,
    pub rotations: Vec<[f32; 4]>,
    pub player_ids: Vec<ClientId>,
}

pub enum ClientChannel {
    Input,
    Command,
}

impl From<ClientChannel> for u8 {
    fn from(channel_id: ClientChannel) -> Self {
        match channel_id {
            ClientChannel::Command => 0,
            ClientChannel::Input => 1,
        }
    }
}

impl ClientChannel {
    pub fn channels_config() -> Vec<ChannelConfig> {
        vec![
            ChannelConfig {
                channel_id: Self::Input.into(),
                max_memory_usage_bytes: 5 * 1024 * 1024,
                send_type: SendType::Unreliable,
            },
            ChannelConfig {
                channel_id: Self::Command.into(),
                max_memory_usage_bytes: 5 * 1024 * 1024,
                send_type: SendType::ReliableOrdered {
                    resend_time: Duration::ZERO,
                },
            },
        ]
    }
}

#[derive(Debug, Serialize, Deserialize, Component)]
pub enum ServerMessages {
    PlayerCreate {
        entity: Entity,
        id: ClientId,
        translation: [f32; 3],
        rotation: [f32; 4],
    },
    PlayerRemove {
        id: ClientId,
    },
}

pub enum ServerChannel {
    ServerMessages,
    NetworkedEntities,
}

impl From<ServerChannel> for u8 {
    fn from(channel_id: ServerChannel) -> Self {
        match channel_id {
            ServerChannel::NetworkedEntities => 0,
            ServerChannel::ServerMessages => 1,
        }
    }
}

impl ServerChannel {
    pub fn channels_config() -> Vec<ChannelConfig> {
        vec![
            ChannelConfig {
                channel_id: Self::NetworkedEntities.into(),
                max_memory_usage_bytes: 10 * 1024 * 1024,
                send_type: SendType::Unreliable,
            },
            ChannelConfig {
                channel_id: Self::ServerMessages.into(),
                max_memory_usage_bytes: 10 * 1024 * 1024,
                send_type: SendType::ReliableOrdered {
                    resend_time: Duration::from_millis(200),
                },
            },
        ]
    }
}

pub fn serialize_networked_entities(
    entities: &NetworkedEntities,
) -> postcard::Result<bytes::Bytes> {
    let buf = postcard::to_allocvec(entities).unwrap();
    Ok(bytes::Bytes::from(buf))
}

pub fn serialize_player_command(cmd: &PlayerCommand) -> postcard::Result<bytes::Bytes> {
    let buf = postcard::to_allocvec(cmd).unwrap();
    Ok(bytes::Bytes::from(buf))
}

pub fn serialize_server_messages(msg: &ServerMessages) -> postcard::Result<bytes::Bytes> {
    let buf = postcard::to_allocvec(msg).unwrap();
    Ok(bytes::Bytes::from(buf))
}

#[derive(Debug)]
pub struct CircularBuffer<const N: usize, T> {
    pub(crate) queue: [T; N],
    cursor: usize,
}

impl<const N: usize, T: Default + Copy> Default for CircularBuffer<N, T> {
    fn default() -> Self {
        Self {
            queue: [T::default(); N],
            cursor: 0,
        }
    }
}

impl<const N: usize, T: Default + Copy> CircularBuffer<N, T> {
    pub fn push(&mut self, value: T) {
        self.queue[self.cursor] = value;
        self.cursor = (self.cursor + 1) % N;
    }

    pub fn as_vec(&self) -> Vec<T> {
        let (end, start) = self.queue.split_at(self.cursor);
        let mut vec = Vec::with_capacity(N);
        vec.extend_from_slice(start);
        vec.extend_from_slice(end);

        vec
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usage() {
        let mut buffer: CircularBuffer<3, usize> = CircularBuffer::default();
        assert_eq!(buffer.as_vec(), vec![0, 0, 0]);

        buffer.push(1);
        buffer.push(2);
        buffer.push(3);
        assert_eq!(buffer.as_vec(), vec![1, 2, 3]);

        buffer.push(4);
        buffer.push(5);
        assert_eq!(buffer.as_vec(), vec![3, 4, 5]);
    }
}

#[derive(Resource)]
pub struct RenetClientVisualizer<const N: usize> {
    rtt: CircularBuffer<N, f32>,
    sent_bandwidth_kbps: CircularBuffer<N, f32>,
    received_bandwidth_kbps: CircularBuffer<N, f32>,
    packet_loss: CircularBuffer<N, f32>,
    style: RenetVisualizerStyle,
}

#[derive(Resource)]
pub struct RenetServerVisualizer<const N: usize> {
    show_all_clients: bool,
    selected_client: Option<ClientId>,
    clients: HashMap<ClientId, RenetClientVisualizer<N>>,
    style: RenetVisualizerStyle,
}

#[derive(Debug, Clone)]
pub struct RenetVisualizerStyle {
    pub width: f32,
    pub height: f32,
    pub text_color: Color32,
    pub rectangle_stroke: Stroke,
    pub line_stroke: Stroke,
}

enum TopValue {
    SuggestedValues([f32; 5]),
    MaxValue { multiplicated: f32 },
}

enum TextFormat {
    Percentage,
    Normal,
}

impl Default for RenetVisualizerStyle {
    fn default() -> Self {
        Self {
            width: 200.,
            height: 100.,
            text_color: Color32::WHITE,
            rectangle_stroke: Stroke::new(1., Color32::WHITE),
            line_stroke: Stroke::new(1., Color32::WHITE),
        }
    }
}

impl<const N: usize> Default for RenetClientVisualizer<N> {
    fn default() -> Self {
        RenetClientVisualizer::new(RenetVisualizerStyle::default())
    }
}

impl<const N: usize> Default for RenetServerVisualizer<N> {
    fn default() -> Self {
        RenetServerVisualizer::new(RenetVisualizerStyle::default())
    }
}

impl<const N: usize> RenetClientVisualizer<N> {
    pub fn new(style: RenetVisualizerStyle) -> Self {
        Self {
            rtt: CircularBuffer::default(),
            sent_bandwidth_kbps: CircularBuffer::default(),
            received_bandwidth_kbps: CircularBuffer::default(),
            packet_loss: CircularBuffer::default(),
            style,
        }
    }

    pub fn add_network_info(&mut self, network_info: NetworkInfo) {
        self.rtt.push((network_info.rtt * 1000.) as f32);
        self.sent_bandwidth_kbps
            .push((network_info.bytes_sent_per_second * 8. / 1000.) as f32);
        self.received_bandwidth_kbps
            .push((network_info.bytes_received_per_second * 8. / 1000.) as f32);
        self.packet_loss.push(network_info.packet_loss as f32);
    }

    pub fn show_window(&self, ctx: &egui::Context) {
        egui::Window::new("Client Network Info")
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    self.draw_all(ui);
                });
            });
    }

    pub fn draw_received_kbps(&self, ui: &mut egui::Ui) {
        show_graph(
            ui,
            &self.style,
            "Received Kbitps",
            TextFormat::Normal,
            TopValue::MaxValue { multiplicated: 1.5 },
            self.received_bandwidth_kbps.as_vec(),
        );
    }

    pub fn draw_sent_kbps(&self, ui: &mut egui::Ui) {
        show_graph(
            ui,
            &self.style,
            "Sent Kbitps",
            TextFormat::Normal,
            TopValue::MaxValue { multiplicated: 1.5 },
            self.sent_bandwidth_kbps.as_vec(),
        );
    }

    pub fn draw_packet_loss(&self, ui: &mut egui::Ui) {
        show_graph(
            ui,
            &self.style,
            "Packet Loss",
            TextFormat::Percentage,
            TopValue::SuggestedValues([0.05, 0.1, 0.25, 0.5, 1.]),
            self.packet_loss.as_vec(),
        );
    }

    pub fn draw_rtt(&self, ui: &mut egui::Ui) {
        show_graph(
            ui,
            &self.style,
            "Round Time Trip (ms)",
            TextFormat::Normal,
            TopValue::SuggestedValues([32., 64., 128., 256., 512.]),
            self.rtt.as_vec(),
        );
    }

    pub fn draw_all(&self, ui: &mut egui::Ui) {
        self.draw_received_kbps(ui);
        self.draw_sent_kbps(ui);
        self.draw_rtt(ui);
        self.draw_packet_loss(ui);
    }
}

impl<const N: usize> RenetServerVisualizer<N> {
    pub fn new(style: RenetVisualizerStyle) -> Self {
        Self {
            show_all_clients: false,
            selected_client: None,
            clients: HashMap::new(),
            style,
        }
    }

    /// Add a new client to keep track off. Should be called whenever a new client
    /// connected event is received.
    ///
    /// # Usage
    /// ```
    /// # use renet::{RenetServer, ServerEvent, ConnectionConfig};
    /// # use renet_visualizer::RenetServerVisualizer;
    /// # let mut renet_server = RenetServer::new(ConnectionConfig::default());
    /// # let mut visualizer = RenetServerVisualizer::<5>::new(Default::default());
    /// while let Some(event) = renet_server.get_event() {
    ///     match event {
    ///         ServerEvent::ClientConnected { client_id } => {
    ///             visualizer.add_client(client_id);
    ///             // ...
    ///         }
    ///         _ => {}
    ///     }
    /// }
    /// ```
    pub fn add_client(&mut self, client_id: ClientId) {
        self.clients
            .insert(client_id, RenetClientVisualizer::new(self.style.clone()));
    }

    /// Remove a client from the visualizer. Should be called whenever a client
    /// disconnected event is received.
    ///
    /// # Usage
    /// ```
    /// # use renet::{RenetServer, ServerEvent, ConnectionConfig};
    /// # use renet_visualizer::RenetServerVisualizer;
    /// # let mut renet_server = RenetServer::new(ConnectionConfig::default());
    /// # let mut visualizer = RenetServerVisualizer::<5>::new(Default::default());
    /// while let Some(event) = renet_server.get_event() {
    ///     match event {
    ///         ServerEvent::ClientDisconnected { client_id , reason } => {
    ///             visualizer.remove_client(client_id);
    ///             // ...
    ///         }
    ///         _ => {}
    ///     }
    /// }
    /// ```
    pub fn remove_client(&mut self, client_id: ClientId) {
        self.clients.remove(&client_id);
    }

    fn add_network_info(&mut self, client_id: ClientId, network_info: NetworkInfo) {
        if let Some(client) = self.clients.get_mut(&client_id) {
            client.add_network_info(network_info);
        }
    }

    /// Update the metrics for all connected clients. Should be called every time the server
    /// updates.
    ///
    /// # Usage
    /// ```
    /// # use renet::{RenetServer, ConnectionConfig};
    /// # use renet_visualizer::RenetServerVisualizer;
    /// # let mut renet_server = RenetServer::new(ConnectionConfig::default());
    /// # let mut visualizer = RenetServerVisualizer::<5>::new(Default::default());
    /// # let delta = std::time::Duration::ZERO;
    /// renet_server.update(delta);
    /// visualizer.update(&renet_server);
    /// ```
    pub fn update(&mut self, server: &RenetServer) {
        for client_id in server.clients_id_iter() {
            if let Ok(network_info) = server.network_info(client_id) {
                self.add_network_info(client_id, network_info);
            }
        }
    }

    pub fn draw_client_metrics(&self, client_id: ClientId, ui: &mut egui::Ui) {
        if let Some(client) = self.clients.get(&client_id) {
            client.draw_all(ui);
        }
    }

    pub fn show_window(&mut self, ctx: &egui::Context) {
        egui::Window::new("Server Network Info")
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_all_clients, "Show all clients");
                    ui.add_enabled_ui(!self.show_all_clients, |ui| {
                        let selected_text = match self.selected_client {
                            Some(client_id) => format!("{client_id}"),
                            None => "------".to_string(),
                        };
                        egui::ComboBox::from_label("Select client")
                            .selected_text(selected_text)
                            .show_ui(ui, |ui| {
                                for client_id in self.clients.keys() {
                                    ui.selectable_value(
                                        &mut self.selected_client,
                                        Some(*client_id),
                                        format!("{client_id}"),
                                    );
                                }
                            })
                    });
                });
                ui.vertical(|ui| {
                    if self.show_all_clients {
                        for (client_id, client) in self.clients.iter() {
                            ui.vertical(|ui| {
                                ui.heading(format!("Client {client_id}"));
                                ui.horizontal(|ui| {
                                    client.draw_all(ui);
                                });
                            });
                        }
                    } else if let Some(selected_client) = self.selected_client {
                        if let Some(client) = self.clients.get(&selected_client) {
                            ui.horizontal(|ui| {
                                client.draw_all(ui);
                            });
                        }
                    }
                });
            });
    }
}

fn show_graph(
    ui: &mut egui::Ui,
    style: &RenetVisualizerStyle,
    label: &str,
    text_format: TextFormat,
    top_value: TopValue,
    values: Vec<f32>,
) {
    if values.is_empty() {
        return;
    }

    ui.vertical(|ui| {
        ui.label(RichText::new(label).heading().color(style.text_color));

        let last_value = values.last().unwrap();

        let min = 0.0;
        let mut max = values.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        match top_value {
            TopValue::MaxValue { multiplicated } => {
                max *= multiplicated;
            }
            TopValue::SuggestedValues(suggested_values) => {
                for value in suggested_values.into_iter() {
                    if max < value {
                        max = value;
                        break;
                    }
                }
            }
        }

        let spacing_x = ui.spacing().item_spacing.x;

        let last_text: WidgetText = match text_format {
            TextFormat::Normal => format!("{last_value}").into(),
            TextFormat::Percentage => format!("{:.1}%", last_value * 100.).into(),
        };
        let galley = last_text.into_galley(
            ui,
            Some(egui::TextWrapMode::Wrap),
            f32::INFINITY,
            TextStyle::Button,
        );
        let (outer_rect, _) = ui.allocate_exact_size(
            Vec2::new(style.width + galley.size().x + spacing_x, style.height),
            Sense::hover(),
        );
        let rect = Rect::from_min_size(outer_rect.left_top(), vec2(style.width, style.height));
        let text_pos = rect.right_center() + vec2(spacing_x / 2.0, -galley.size().y / 2.);
        ui.painter()
            .with_clip_rect(outer_rect)
            .galley(text_pos, galley, style.text_color);

        let body = Shape::Rect(RectShape {
            rect,
            fill: Rgba::TRANSPARENT.into(),
            stroke: style.rectangle_stroke,
            stroke_kind: StrokeKind::Inside,
            blur_width: 0.0,
            corner_radius: CornerRadius::ZERO,
            round_to_pixels: None,
            brush: None,
        });
        ui.painter().add(body);
        let init_point = rect.left_bottom();

        let size = values.len();
        let points = values
            .iter()
            .enumerate()
            .map(|(i, value)| {
                let x = remap(i as f32, 0.0..=size as f32, 0.0..=style.width);
                let y = if max == 0.0 {
                    0.0
                } else {
                    remap(*value, min..=max, 0.0..=style.height)
                };

                pos2(x + init_point.x, init_point.y - y)
            })
            .collect();

        let path = PathShape::line(points, style.line_stroke);
        ui.painter().add(path);

        {
            let text: WidgetText = match text_format {
                TextFormat::Normal => format!("{min}").into(),
                TextFormat::Percentage => format!("{:.0}%", max * 100.).into(),
            };
            let galley = text.into_galley(
                ui,
                Some(egui::TextWrapMode::Wrap),
                f32::INFINITY,
                TextStyle::Button,
            );
            let text_pos =
                rect.left_top() + Vec2::new(0.0, galley.size().y / 2.) + vec2(spacing_x, 0.0);
            ui.painter()
                .with_clip_rect(outer_rect)
                .galley(text_pos, galley, style.text_color);
        }
        {
            let text: WidgetText = match text_format {
                TextFormat::Normal => format!("{min}").into(),
                TextFormat::Percentage => format!("{:.0}%", min * 100.).into(),
            };
            let galley = text.into_galley(
                ui,
                Some(egui::TextWrapMode::Wrap),
                f32::INFINITY,
                TextStyle::Button,
            );
            let text_pos =
                rect.left_bottom() - Vec2::new(0.0, galley.size().y * 1.5) + vec2(spacing_x, 0.0);
            ui.painter()
                .with_clip_rect(outer_rect)
                .galley(text_pos, galley, style.text_color);
        }
    });
}

#[derive(Resource)]
pub struct SceneLoadQueue {
    queue: VecDeque<(String, usize)>,
    in_flight: Vec<Handle<Scene>>,
}

pub struct GameAsset {
    pub name: &'static str,
    pub transform: Transform,
}

impl SceneLoadQueue {
    pub fn new(names: &[&GameAsset]) -> Self {
        let mut q = VecDeque::new();
        for (i, asset) in names.iter().enumerate() {
            let name = asset.name;
            q.push_back((format!("{name}.glb"), i));
        }
        Self {
            queue: q,
            in_flight: Vec::new(),
        }
    }
}

pub fn setup_level(mut commands: Commands, asset_server: Res<AssetServer>) {
    let cascade_shadow_config = CascadeShadowConfigBuilder {
        first_cascade_far_bound: 0.3,
        maximum_distance: 0.3,
        ..default()
    }
    .build();

    // Sun
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.98, 0.95, 0.82),
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.0, 0.0, 0.0).looking_at(Vec3::new(-0.15, -0.05, 0.25), Vec3::Y),
        cascade_shadow_config,
    ));

    // Terrain
    commands.spawn((
        SceneRoot(
            asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/terrain/Mountains.gltf")),
        ),
        Transform::from_xyz(0.0, -0.5, 0.0).with_scale(Vec3::splat(100.0)),
    ));

    commands.spawn((
        SceneRoot(
            asset_server.load(GltfAssetLabel::Scene(0).from_asset("Dungeon/gltf/box_stacked.gltf")),
        ),
        Transform::from_xyz(-2.0, 0.0, 0.0).with_scale(Vec3::splat(1.0)),
    ));
}

pub fn pump_scene_loads(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut q: ResMut<SceneLoadQueue>,
) {
    q.in_flight.retain(|handle| {
        !matches!(
            asset_server.load_state(handle.id()),
            bevy::asset::LoadState::Loaded
        )
    });

    while q.in_flight.len() < 10 {
        let Some((path, idx)) = q.queue.pop_front() else {
            break;
        };
        let handle: Handle<Scene> =
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(path.clone()));

        let transform = if let Some(asset) = EXAMPLES.get(idx) {
            asset.transform
        } else {
            Transform::from_xyz(0.0, 0.0, 0.0).with_scale(Vec3::splat(1.0))
        };

        commands.spawn((SceneRoot(handle.clone()), transform));
        q.in_flight.push(handle);
    }
}

#[allow(clippy::type_complexity)]
pub fn add_colliders_to_gltf_scene(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    asset_server: Res<AssetServer>,
    q: Query<(Entity, &Mesh3d), (Added<Mesh3d>, Without<Collider>)>,
) {
    for (e, mesh3d) in &q {
        // nur Instanzen aus Mountains.gltf anfassen
        let Some(asset_path) = asset_server.get_path(mesh3d.0.id()) else {
            continue;
        };
        if !asset_path.to_string().contains("Mountains.gltf") {
            continue;
        }

        if let Some(mesh) = meshes.get(&mesh3d.0) {
            // bevy_rapier3d 0.26: TriMesh ist ein Konstruktor, Flags nötig
            if let Some(collider) = Collider::from_bevy_mesh(
                mesh,
                &ComputedColliderShape::TriMesh(TriMeshFlags::default()),
            ) {
                commands
                    .entity(e)
                    .insert(RigidBody::Fixed)
                    .insert(collider)
                    .insert(Friction::coefficient(0.4))
                    .insert(Restitution::coefficient(0.0));
            }
        }
    }
}

#[derive(Component)]
pub struct FreeFlyCamera;

#[derive(Component)]
pub struct FreeFlyState {
    pub yaw: f32,
    pub pitch: f32,
}

#[derive(Resource)]
pub struct FreeFlyConfig {
    pub base_speed: f32,
    pub boost_multiplier: f32,
    pub mouse_sensitivity: f32,
}

impl Default for FreeFlyConfig {
    fn default() -> Self {
        Self {
            base_speed: 12.0,
            boost_multiplier: 6.0,
            mouse_sensitivity: 0.0025,
        }
    }
}

pub struct FreeFlyCameraPlugin;

impl Plugin for FreeFlyCameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FreeFlyConfig>()
            .add_systems(Startup, free_fly_setup)
            .add_systems(Update, (free_fly_look, free_fly_move));
    }
}

fn free_fly_setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        FreeFlyCamera,
        FreeFlyState {
            yaw: 0.0,
            pitch: -0.35,
        },
        Transform::from_xyz(0.0, 40.0, 80.0).looking_at(Vec3::ZERO, Vec3::Y),
        bevy_atmosphere::plugin::AtmosphereCamera::default(),
    ));
}

fn free_fly_look(
    mut motion_evr: EventReader<MouseMotion>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    cfg: Res<FreeFlyConfig>,
    mut q: Query<(&mut Transform, &mut FreeFlyState), With<FreeFlyCamera>>,
) {
    if !mouse_buttons.pressed(MouseButton::Right) {
        motion_evr.clear();
        return;
    }

    let mut delta = bevy::prelude::Vec2::ZERO;
    for ev in motion_evr.read() {
        delta += ev.delta;
    }
    if delta == bevy::prelude::Vec2::ZERO {
        return;
    }

    if let Some((mut tf, mut st)) = q.iter_mut().next() {
        st.yaw -= delta.x * cfg.mouse_sensitivity;
        st.pitch -= delta.y * cfg.mouse_sensitivity;
        st.pitch = st.pitch.clamp(-1.553343, 1.553343); // ~±89°

        tf.rotation = Quat::from_euler(EulerRot::YXZ, st.yaw, st.pitch, 0.0);
    }
}

fn free_fly_move(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    cfg: Res<FreeFlyConfig>,
    mut q: Query<&mut Transform, With<FreeFlyCamera>>,
) {
    let mut tf = q.iter_mut().next().expect("FreeFlyCamera not found");

    let mut wish = Vec3::ZERO;

    // vor/zurück (relativ zur Kamera)
    let forward = tf.rotation * -Vec3::Z;
    if keys.pressed(KeyCode::KeyW) {
        wish += forward;
    }
    if keys.pressed(KeyCode::KeyS) {
        wish -= forward;
    }

    // links/rechts (relativ zur Kamera)
    let right = tf.rotation * Vec3::X;
    if keys.pressed(KeyCode::KeyD) {
        wish += right;
    }
    if keys.pressed(KeyCode::KeyA) {
        wish -= right;
    }

    // hoch/runter (welt-achsen)
    if keys.pressed(KeyCode::Space) {
        wish += Vec3::Y;
    }
    if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
        wish -= Vec3::Y;
    }

    let mut speed = cfg.base_speed;
    if keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
        speed *= cfg.boost_multiplier;
    }

    let dir = wish.normalize_or_zero();
    tf.translation += dir * speed * time.delta_secs();
}

#[derive(Component, Clone, Copy)]
pub struct PlayerColliderDims {
    pub half_height: f32, // Kapsel-Halblänge entlang Y
    pub radius: f32,      // Kapsel-Radius
}

pub struct PlayerHitboxDebugPlugin;

#[derive(Resource)]
struct PlayerHitboxDebugConfig {
    enabled: bool,
}

impl Default for PlayerHitboxDebugConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

impl Plugin for PlayerHitboxDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PlayerHitboxDebugConfig>()
            .add_systems(Update, (toggle_player_hitbox_debug, draw_player_hitboxes));
    }
}

fn toggle_player_hitbox_debug(
    keys: Res<ButtonInput<KeyCode>>,
    mut cfg: ResMut<PlayerHitboxDebugConfig>,
) {
    if keys.just_pressed(KeyCode::F2) {
        cfg.enabled = !cfg.enabled;
    }
}

fn draw_player_hitboxes(
    mut gizmos: Gizmos,
    cfg: Res<PlayerHitboxDebugConfig>,
    q: Query<(&GlobalTransform, &PlayerColliderDims), With<Player>>,
) {
    if !cfg.enabled {
        return;
    }

    for (gt, dims) in &q {
        let mut t = Transform::from_translation(gt.translation());
        t.rotation = gt.compute_transform().rotation;

        let size = Vec3::new(
            dims.radius * 2.0,
            dims.half_height * 2.0 + dims.radius * 2.0,
            dims.radius * 2.0,
        );
        t.scale = size;
        gizmos.cuboid(t, Color::srgb(0.1, 0.9, 0.3));
    }
}

pub const EXAMPLES: &[&GameAsset] = &[
    &GameAsset {
        name: "MedievalVillage/glb/Balcony_Cross_Corner",
        transform: Transform::from_xyz(0.0, 0.0, 0.0).with_scale(Vec3::splat(1.0)),
    },
    &GameAsset {
        name: "custom/house_1",
        transform: Transform::from_xyz(10.0, 10.0, 0.0).with_scale(Vec3::splat(1.0)),
    },
];

/*
    "MedievalVillage/glb/Balcony_Cross_Straight",
    "MedievalVillage/glb/Balcony_Simple_Corner",
    "MedievalVillage/glb/Balcony_Simple_Straight",
    "MedievalVillage/glb/Corner_ExteriorWide_Brick",
    "MedievalVillage/glb/Corner_ExteriorWide_Wood",
    "MedievalVillage/glb/Corner_Exterior_Brick",
    "MedievalVillage/glb/Corner_Exterior_TopDown",
    "MedievalVillage/glb/Corner_Exterior_TopOnly",
    "MedievalVillage/glb/Corner_Exterior_Wood",
    "MedievalVillage/glb/Corner_Interior_Big",
    "MedievalVillage/glb/Corner_Interior_Small",
    "MedievalVillage/glb/DoorFrame_Flat_Brick",
    "MedievalVillage/glb/DoorFrame_Flat_WoodDark",
    "MedievalVillage/glb/DoorFrame_Round_Brick",
    "MedievalVillage/glb/DoorFrame_Round_WoodDark",
    "MedievalVillage/glb/Door_1_Flat",
    "MedievalVillage/glb/Door_1_Round",
    "MedievalVillage/glb/Door_2_Flat",
    "MedievalVillage/glb/Door_2_Round",
    "MedievalVillage/glb/Door_4_Flat",
    "MedievalVillage/glb/Door_4_Round",
    "MedievalVillage/glb/Door_8_Flat",
    "MedievalVillage/glb/Door_8_Round",
    "MedievalVillage/glb/Floor_Brick",
    "MedievalVillage/glb/Floor_RedBrick",
    "MedievalVillage/glb/Floor_UnevenBrick",
    "MedievalVillage/glb/Floor_WoodDark",
    "MedievalVillage/glb/Floor_WoodDark_Half1",
    "MedievalVillage/glb/Floor_WoodDark_Half2",
    "MedievalVillage/glb/Floor_WoodDark_Half3",
    "MedievalVillage/glb/Floor_WoodDark_OverhangCorner",
    "MedievalVillage/glb/Floor_WoodDark_OverhangCorner2",
    "MedievalVillage/glb/Floor_WoodLight",
    "MedievalVillage/glb/Floor_WoodLight_OverhangCorner",
    "MedievalVillage/glb/Floor_WoodLight_OverhangCorner2",
    "MedievalVillage/glb/HoleCover_90Angle",
    "MedievalVillage/glb/HoleCover_90Half",
    "MedievalVillage/glb/HoleCover_90Stairs",
    "MedievalVillage/glb/HoleCover_Straight",
    "MedievalVillage/glb/HoleCover_StraightHalf",
    "MedievalVillage/glb/Overhang_Plaster_Corner",
    "MedievalVillage/glb/Overhang_Plaster_Corner_Front",
    "MedievalVillage/glb/Overhang_Plaster_Long",
    "MedievalVillage/glb/Overhang_Plaster_Short",
    "MedievalVillage/glb/Overhang_RoofIncline_Plaster",
    "MedievalVillage/glb/Overhang_RoofIncline_UnevenBricks",
    "MedievalVillage/glb/Overhang_Roof_Plaster",
    "MedievalVillage/glb/Overhang_Roof_UnevenBricks",
    "MedievalVillage/glb/Overhang_Side_Plaster_Long_L",
    "MedievalVillage/glb/Overhang_Side_Plaster_Long_R",
    "MedievalVillage/glb/Overhang_Side_Plaster_Short_L",
    "MedievalVillage/glb/Overhang_Side_Plaster_Short_R",
    "MedievalVillage/glb/Overhang_Side_UnevenBrick_Long_L",
    "MedievalVillage/glb/Overhang_Side_UnevenBrick_Long_R",
    "MedievalVillage/glb/Overhang_Side_UnevenBrick_Short_L",
    "MedievalVillage/glb/Overhang_Side_UnevenBrick_Short_R",
    "MedievalVillage/glb/Overhang_UnevenBrick_Corner",
    "MedievalVillage/glb/Overhang_UnevenBrick_Corner_Front",
    "MedievalVillage/glb/Overhang_UnevenBrick_Long",
    "MedievalVillage/glb/Overhang_UnevenBrick_Short",
    "MedievalVillage/glb/Prop_Brick1",
    "MedievalVillage/glb/Prop_Brick2",
    "MedievalVillage/glb/Prop_Brick3",
    "MedievalVillage/glb/Prop_Brick4",
    "MedievalVillage/glb/Prop_Chimney",
    "MedievalVillage/glb/Prop_Chimney2",
    "MedievalVillage/glb/Prop_Crate",
    "MedievalVillage/glb/Prop_ExteriorBorder_Corner",
    "MedievalVillage/glb/Prop_ExteriorBorder_Straight1",
    "MedievalVillage/glb/Prop_ExteriorBorder_Straight2",
    "MedievalVillage/glb/Prop_MetalFence_Ornament",
    "MedievalVillage/glb/Prop_MetalFence_Simple",
    "MedievalVillage/glb/Prop_Support",
    "MedievalVillage/glb/Prop_Vine1",
    "MedievalVillage/glb/Prop_Vine2",
    "MedievalVillage/glb/Prop_Vine4",
    "MedievalVillage/glb/Prop_Vine5",
    "MedievalVillage/glb/Prop_Vine6",
    "MedievalVillage/glb/Prop_Vine9",
    "MedievalVillage/glb/Prop_Wagon",
    "MedievalVillage/glb/Prop_WoodenFence_Extension1",
    "MedievalVillage/glb/Prop_WoodenFence_Extension2",
    "MedievalVillage/glb/Prop_WoodenFence_Single",
    "MedievalVillage/glb/Roof_2x4_RoundTile",
    "MedievalVillage/glb/Roof_Dormer_RoundTile",
    "MedievalVillage/glb/Roof_FrontSupports",
    "MedievalVillage/glb/Roof_Front_Brick2",
    "MedievalVillage/glb/Roof_Front_Brick4",
    "MedievalVillage/glb/Roof_Front_Brick4_Half_L",
    "MedievalVillage/glb/Roof_Front_Brick4_Half_R",
    "MedievalVillage/glb/Roof_Front_Brick6",
    "MedievalVillage/glb/Roof_Front_Brick6_Half_L",
    "MedievalVillage/glb/Roof_Front_Brick6_Half_R",
    "MedievalVillage/glb/Roof_Front_Brick8",
    "MedievalVillage/glb/Roof_Front_Brick8_Half_L",
    "MedievalVillage/glb/Roof_Front_Brick8_Half_R",
    "MedievalVillage/glb/Roof_Log",
    "MedievalVillage/glb/Roof_Modular_RoundTiles",
    "MedievalVillage/glb/Roof_RoundTile_2x1",
    "MedievalVillage/glb/Roof_RoundTile_2x1_Long",
    "MedievalVillage/glb/Roof_RoundTiles_4x4",
    "MedievalVillage/glb/Roof_RoundTiles_4x6",
    "MedievalVillage/glb/Roof_RoundTiles_4x8",
    "MedievalVillage/glb/Roof_RoundTiles_6x4",
    "MedievalVillage/glb/Roof_RoundTiles_6x6",
    "MedievalVillage/glb/Roof_RoundTiles_6x8",
    "MedievalVillage/glb/Roof_RoundTiles_6x10",
    "MedievalVillage/glb/Roof_RoundTiles_6x12",
    "MedievalVillage/glb/Roof_RoundTiles_6x14",
    "MedievalVillage/glb/Roof_RoundTiles_8x8",
    "MedievalVillage/glb/Roof_RoundTiles_8x10",
    "MedievalVillage/glb/Roof_RoundTiles_8x12",
    "MedievalVillage/glb/Roof_RoundTiles_8x14",
    "MedievalVillage/glb/Roof_Support2",
    "MedievalVillage/glb/Roof_Tower_RoundTiles",
    "MedievalVillage/glb/Roof_Wooden_2x1",
    "MedievalVillage/glb/Roof_Wooden_2x1_Center",
    "MedievalVillage/glb/Roof_Wooden_2x1_Center_Mirror",
    "MedievalVillage/glb/Roof_Wooden_2x1_Corner",
    "MedievalVillage/glb/Roof_Wooden_2x1_L",
    "MedievalVillage/glb/Roof_Wooden_2x1_Middle",
    "MedievalVillage/glb/Roof_Wooden_2x1_R",
    "MedievalVillage/glb/Stair_Interior_Rails",
    "MedievalVillage/glb/Stair_Interior_Simple",
    "MedievalVillage/glb/Stair_Interior_Solid",
    "MedievalVillage/glb/Stair_Interior_SolidExtended",
    "MedievalVillage/glb/Stairs_Exterior_NoFirstStep",
    "MedievalVillage/glb/Stairs_Exterior_Platform",
    "MedievalVillage/glb/Stairs_Exterior_Platform45",
    "MedievalVillage/glb/Stairs_Exterior_Platform45Clean",
    "MedievalVillage/glb/Stairs_Exterior_PlatformU",
    "MedievalVillage/glb/Stairs_Exterior_SidePlatform",
    "MedievalVillage/glb/Stairs_Exterior_Sides",
    "MedievalVillage/glb/Stairs_Exterior_Sides45",
    "MedievalVillage/glb/Stairs_Exterior_SidesU",
    "MedievalVillage/glb/Stairs_Exterior_SingleSide",
    "MedievalVillage/glb/Stairs_Exterior_SingleSideThick",
    "MedievalVillage/glb/Stairs_Exterior_Straight",
    "MedievalVillage/glb/Stairs_Exterior_Straight_Center",
    "MedievalVillage/glb/Stairs_Exterior_Straight_L",
    "MedievalVillage/glb/Stairs_Exterior_Straight_R",
    "MedievalVillage/glb/Wall_Arch",
    "MedievalVillage/glb/Wall_BottomCover",
    "MedievalVillage/glb/Wall_Plaster_Door_Flat",
    "MedievalVillage/glb/Wall_Plaster_Door_Round",
    "MedievalVillage/glb/Wall_Plaster_Door_RoundInset",
    "MedievalVillage/glb/Wall_Plaster_Straight",
    "MedievalVillage/glb/Wall_Plaster_Straight_Base",
    "MedievalVillage/glb/Wall_Plaster_Straight_L",
    "MedievalVillage/glb/Wall_Plaster_Straight_R",
    "MedievalVillage/glb/Wall_Plaster_Window_Thin_Round",
    "MedievalVillage/glb/Wall_Plaster_Window_Wide_Flat",
    "MedievalVillage/glb/Wall_Plaster_Window_Wide_Flat2",
    "MedievalVillage/glb/Wall_Plaster_Window_Wide_Round",
    "MedievalVillage/glb/Wall_Plaster_WoodGrid",
    "MedievalVillage/glb/Wall_UnevenBrick_Door_Flat",
    "MedievalVillage/glb/Wall_UnevenBrick_Door_Round",
    "MedievalVillage/glb/Wall_UnevenBrick_Straight",
    "MedievalVillage/glb/Wall_UnevenBrick_Window_Thin_Round",
    "MedievalVillage/glb/Wall_UnevenBrick_Window_Wide_Flat",
    "MedievalVillage/glb/Wall_UnevenBrick_Window_Wide_Round",
    "MedievalVillage/glb/WindowShutters_Thin_Flat_Closed",
    "MedievalVillage/glb/WindowShutters_Thin_Flat_Open",
    "MedievalVillage/glb/WindowShutters_Thin_Round_Closed",
    "MedievalVillage/glb/WindowShutters_Thin_Round_Open",
    "MedievalVillage/glb/WindowShutters_Wide_Flat_Closed",
    "MedievalVillage/glb/WindowShutters_Wide_Flat_Open",
    "MedievalVillage/glb/WindowShutters_Wide_Round_Closed",
    "MedievalVillage/glb/WindowShutters_Wide_Round_Open",
    "MedievalVillage/glb/Window_Roof_Thin",
    "MedievalVillage/glb/Window_Roof_Wide",
    "MedievalVillage/glb/Window_Thin_Flat1",
    "MedievalVillage/glb/Window_Thin_Round1",
    "MedievalVillage/glb/Window_Wide_Flat1",
    "MedievalVillage/glb/Window_Wide_Round1",
*/
