// API request/response types — generated from OpenAPI spec via `pnpm api:generate`.
// Import from here (not directly from ./api/generated) so aliases and overrides stay in one place.

// Backward-compat aliases for renamed generated types.
export type {
  Detection,
  HealthResponse,
  ImuData,
  MotorState,
  Position3d,
  Position3d as Position3D,
  SessionResponse,
  SessionResponse as ReplaySessionInfo,
  TelemetryMetrics,
  TopicsSnapshot,
  TopicUpdate,
} from './api/generated';

// RobotSnapshot: generated type has optional arrays; Zod `.default([])` in schemas.ts
// guarantees they're always present after validation, so we override them as required here.
import type {
  RobotSnapshot as ApiRobotSnapshot,
  Position3d,
  TelemetryMetrics,
} from './api/generated';
export interface RobotSnapshot
  extends Omit<ApiRobotSnapshot, 'lidar_points' | 'path_history' | 'logs'> {
  lidar_points: Array<Position3d>;
  path_history: Array<Position3d>;
  logs: Array<string>;
}

// NodeHealthValue is the discriminated union from the generated TelemetryMetrics.
export type NodeHealthValue = TelemetryMetrics['node_health'];

// Frontend-specific constants (not derived from the OpenAPI spec)

// Robot state-machine stages. Values are lowercase to match what
// state_machine_node publishes on /robot_state (shared/domain/enums.py
// defines the enum with lowercase values); consumers must compare against
// the same casing. See StateDiagram's boundary normalisation.
export const RobotState = {
  BOOT_CHECK: 'boot_check',
  READY: 'ready',
  RACING: 'racing',
  FINISHED: 'finished',
} as const;

export const RosMessageType = {
  LASER_SCAN: 'sensor_msgs/LaserScan',
  ODOMETRY: 'nav_msgs/Odometry',
  IMU: 'sensor_msgs/Imu',
  STRING: 'std_msgs/String',
  TWIST: 'geometry_msgs/Twist',
  JOINT_STATE: 'sensor_msgs/JointState',
  DETECTION_2D_ARRAY: 'vision_msgs/Detection2DArray',
} as const;

// Values match proto enum names emitted by protojson.
export const NodeHealth = {
  UNSPECIFIED: 'NODE_HEALTH_UNSPECIFIED',
  NOMINAL: 'NODE_HEALTH_NOMINAL',
  WATCHDOG: 'NODE_HEALTH_WATCHDOG',
  REPLANNING: 'NODE_HEALTH_REPLANNING',
} as const;

// Detection class names emitted by the vision pipeline.
export const DetectionClass = {
  RED_SIGN: 'red_sign',
  GREEN_SIGN: 'green_sign',
} as const;

export type DetectionClassValue = (typeof DetectionClass)[keyof typeof DetectionClass];

// A point/vector in Three.js space (X right, Y up, Z toward camera).
export type Vec3 = [number, number, number];

// ROS message payloads (raw TopicUpdate.data shapes)
//
// These model the untyped `data` carried by each TopicUpdate so the topic
// visualizers — and the demo data generators — share one definition instead of
// reaching into `Record<string, any>`.

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/** sensor_msgs/LaserScan */
export interface LaserScanMsg {
  ranges: number[];
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
}

/** geometry_msgs/Twist */
export interface TwistMsg {
  linear: Position3d;
  angular: Position3d;
}

/** sensor_msgs/Imu */
export interface ImuMsg {
  orientation: Quaternion;
  linear_acceleration: Position3d;
  angular_velocity: Position3d;
}

/** sensor_msgs/JointState */
export interface JointStateMsg {
  name: string[];
  position: number[];
  velocity?: number[];
  effort?: number[];
}

/** nav_msgs/Odometry (subset used by the dashboard) */
export interface OdometryMsg {
  pose: { position: Position3d; orientation: Partial<Quaternion> };
  twist: { linear: Partial<Position3d>; angular: Partial<Position3d> };
}

/** std_msgs/String */
export interface StringMsg {
  data: string;
}

/** vision_msgs/Detection2DArray (subset used by the dashboard) */
export interface Detection2DMsg {
  results: Array<{ hypothesis: { class_id: string; score: number } }>;
  bbox: {
    center: { position: { x: number; y: number } };
    size_x: number;
    size_y: number;
  };
}

export interface Detection2DArrayMsg {
  detections: Detection2DMsg[];
}
