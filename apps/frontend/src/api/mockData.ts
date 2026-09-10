/**
 * src/api/mockData.ts
 *
 * Self-contained mock telemetry generators for Demo Mode.
 *
 * Produces data in the exact shape the Go backend emits (protojson snake_case,
 * ISO-8601 timestamps) so that mock data flows through the same Zod schemas and
 * components as live data — Demo Mode behaves identically to a real connection,
 * just without a backend.
 *
 * The simulated robot drives a loop around the WRO track (0–3 m world, centre
 * at 1.5, 1.5). LiDAR points trace the outer and inner track walls so the 3D
 * scene and radar chart look realistic.
 */

import type {
  Position3D,
  ReplaySessionInfo,
  RobotSnapshot,
  TopicsSnapshot,
  TopicUpdate,
} from '../types';
import { NodeHealth, RobotState, RosMessageType } from '../types';

// World geometry (metres) — mirrors SIMULATION_CONFIG.TRACK.
const TRACK_MIN = 0;
const TRACK_MAX = 3;
const TRACK_CENTER = 1.5;
const INNER_MIN = 1.0;
const INNER_MAX = 2.0;

const DRIVE_RADIUS = 1.15; // radius of the robot's circular path around centre
const ANGULAR_SPEED = 0.12; // rad per tick
const LINEAR_SPEED = 0.42; // m/s reported on gauges
const PATH_HISTORY_MAX = 80;
const LIDAR_RAYS = 120;

/** Deterministic pseudo-noise so a given tick always renders the same. */
const noise = (seed: number): number => (Math.sin(seed * 12.9898) * 43758.5453) % 1;

const nowIso = (): string => new Date().toISOString();

/** Robot pose on the circular demo path at a given tick. */
function poseAt(tick: number): { position: Position3D; orientation: number } {
  const angle = tick * ANGULAR_SPEED;
  return {
    position: {
      x: TRACK_CENTER + DRIVE_RADIUS * Math.cos(angle),
      y: TRACK_CENTER + DRIVE_RADIUS * Math.sin(angle),
      z: 0.1,
    },
    // Heading is tangent to the circle.
    orientation: angle + Math.PI / 2,
  };
}

/**
 * LiDAR points tracing the outer track perimeter and the inner square, in world
 * coordinates (the same frame the scene plots the robot in).
 */
function lidarWalls(tick: number): Position3D[] {
  const points: Position3D[] = [];
  const jitter = () => (noise(tick + points.length) - 0.5) * 0.02;

  const edge = (count: number, fn: (t: number) => [number, number]) => {
    for (let i = 0; i < count; i++) {
      const [x, y] = fn(i / (count - 1));
      points.push({ x: x + jitter(), y: y + jitter(), z: 0.05 });
    }
  };

  const perRay = Math.floor(LIDAR_RAYS / 8);
  // Outer walls.
  edge(perRay, (t) => [TRACK_MIN + t * (TRACK_MAX - TRACK_MIN), TRACK_MIN]);
  edge(perRay, (t) => [TRACK_MAX, TRACK_MIN + t * (TRACK_MAX - TRACK_MIN)]);
  edge(perRay, (t) => [TRACK_MIN + t * (TRACK_MAX - TRACK_MIN), TRACK_MAX]);
  edge(perRay, (t) => [TRACK_MIN, TRACK_MIN + t * (TRACK_MAX - TRACK_MIN)]);
  // Inner square walls.
  edge(perRay, (t) => [INNER_MIN + t * (INNER_MAX - INNER_MIN), INNER_MIN]);
  edge(perRay, (t) => [INNER_MAX, INNER_MIN + t * (INNER_MAX - INNER_MIN)]);
  edge(perRay, (t) => [INNER_MIN + t * (INNER_MAX - INNER_MIN), INNER_MAX]);
  edge(perRay, (t) => [INNER_MIN, INNER_MIN + t * (INNER_MAX - INNER_MIN)]);

  return points;
}

/** Accumulated breadcrumb trail along the circular path. */
function pathHistory(tick: number): Position3D[] {
  const trail: Position3D[] = [];
  const start = Math.max(0, tick - PATH_HISTORY_MAX);
  for (let t = start; t <= tick; t++) {
    trail.push(poseAt(t).position);
  }
  return trail;
}

const LOG_LINES = [
  'nav: waypoint reached, replanning corridor',
  'lidar: 0 obstacles within safety margin',
  'vision: tracking 1 traffic sign',
  'control: steering trim nominal',
  'imu: orientation drift within tolerance',
  'mission: lap segment complete',
];

/** Build a single mock robot snapshot for the given tick. */
export function generateMockSnapshot(tick: number): RobotSnapshot {
  const { position, orientation } = poseAt(tick);
  const lidar = lidarWalls(tick);
  const wobble = Math.sin(tick * 0.3);

  return {
    timestamp: nowIso(),
    mission_name: 'Demo Mission — Obstacle Run',

    robot_position: position,
    robot_orientation: orientation,

    lidar_points: lidar,
    path_history: pathHistory(tick),
    logs: Array.from({ length: 4 }, (_, i) => LOG_LINES[(tick + i) % LOG_LINES.length]),

    metrics: {
      timestamp: nowIso(),
      node_health: NodeHealth.NOMINAL,
      points_captured: lidar.length,
      range_min: 0.18 + Math.abs(wobble) * 0.05,
      range_max: 1.95,
      range_mean: 0.92,
      forward: 0.85 + wobble * 0.1,
      left: 0.55 + wobble * 0.2,
      right: 0.6 - wobble * 0.2,
      back: 1.2,
      speed: LINEAR_SPEED + wobble * 0.05,
      stage: RobotState.RACING,
      lidar_available: true,
      imu_available: true,
      camera_available: true,
      odometry_available: true,
    },

    imu_data: {
      linear_acceleration: { x: wobble * 0.4, y: Math.cos(tick * 0.3) * 0.3, z: 9.81 },
      angular_velocity: { x: 0.01, y: -0.02, z: ANGULAR_SPEED + wobble * 0.05 },
      orientation_x: 0,
      orientation_y: 0,
      orientation_z: Math.sin(orientation / 2),
      orientation_w: Math.cos(orientation / 2),
    },

    vision_detections:
      tick % 5 < 3
        ? [
            {
              class_name: tick % 2 === 0 ? 'red_sign' : 'green_sign',
              confidence: 0.78 + Math.abs(wobble) * 0.15,
              bbox_x: 0.45,
              bbox_y: 0.4,
              bbox_w: 0.12,
              bbox_h: 0.2,
            },
          ]
        : [],

    motor_state: {
      steering_angle: wobble * 0.35,
      drive_speed: 60 + wobble * 15,
      encoder_position: Math.round(tick * 128),
    },
  };
}

/** Raw ROS-topic payloads derived from a snapshot, for the Topic Inspector. */
export function generateMockTopics(tick: number): TopicsSnapshot {
  const snap = generateMockSnapshot(tick);
  const { imu_data: imu, motor_state: motor } = snap;
  if (!imu || !motor) {
    throw new Error('generateMockSnapshot must always populate imu_data and motor_state');
  }
  const det = snap.vision_detections ?? [];

  const ranges = Array.from({ length: 180 }, (_, i) => {
    const base = 0.5 + 0.4 * Math.abs(Math.sin(i * 0.05 + tick * 0.1));
    return Math.min(2.0, base + (noise(tick + i) - 0.5) * 0.05);
  });

  const topic = (
    topic_name: string,
    message_type: string,
    update_rate_hz: number,
    data: Record<string, unknown>
  ): TopicUpdate => ({ topic_name, message_type, timestamp: nowIso(), update_rate_hz, data });

  return {
    timestamp: nowIso(),
    topics: [
      topic('/scan', RosMessageType.LASER_SCAN, 10, {
        ranges,
        angle_min: -Math.PI,
        angle_max: Math.PI,
        angle_increment: (2 * Math.PI) / ranges.length,
        range_min: 0.05,
        range_max: 2.0,
      }),
      topic('/odom', RosMessageType.ODOMETRY, 30, {
        pose: {
          position: snap.robot_position,
          orientation: { z: imu.orientation_z, w: imu.orientation_w },
        },
        twist: { linear: { x: snap.metrics.speed }, angular: { z: imu.angular_velocity.z } },
      }),
      topic('/imu', RosMessageType.IMU, 50, {
        orientation: {
          x: imu.orientation_x,
          y: imu.orientation_y,
          z: imu.orientation_z,
          w: imu.orientation_w,
        },
        linear_acceleration: imu.linear_acceleration,
        angular_velocity: imu.angular_velocity,
      }),
      topic('/cmd_vel', RosMessageType.TWIST, 20, {
        linear: { x: snap.metrics.speed, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: imu.angular_velocity.z },
      }),
      topic('/joint_states', RosMessageType.JOINT_STATE, 30, {
        name: ['steering_joint', 'drive_joint'],
        position: [motor.steering_angle, (tick * 0.5) % (Math.PI * 2)],
        velocity: [0, motor.drive_speed / 60],
        effort: [0.4, 0.9],
      }),
      topic('/robot/state', RosMessageType.STRING, 1, { data: snap.metrics.stage }),
      topic('/detections', RosMessageType.DETECTION_2D_ARRAY, 15, {
        detections: det.map((d) => ({
          results: [{ hypothesis: { class_id: d.class_name, score: d.confidence } }],
          bbox: {
            center: { position: { x: d.bbox_x * 640, y: d.bbox_y * 480 } },
            size_x: d.bbox_w * 640,
            size_y: d.bbox_h * 480,
          },
        })),
      }),
    ],
  };
}

/** A pre-recorded "replay" — one full lap of snapshots. */
export function generateMockSession(sessionId: string): RobotSnapshot[] {
  const length = sessionId === 'demo-lap-2' ? 60 : 48;
  return Array.from({ length }, (_, i) => generateMockSnapshot(i));
}

/** Available demo replay sessions. */
export function generateMockSessions(): ReplaySessionInfo[] {
  return [
    { session_id: 'demo-lap-1', created_at: nowIso(), entry_count: 48 },
    { session_id: 'demo-lap-2', created_at: nowIso(), entry_count: 60 },
  ];
}
