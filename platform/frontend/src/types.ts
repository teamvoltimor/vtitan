export const RobotState = {
  BOOT_CHECK: 'BOOT_CHECK',
  READY: 'READY',
  RACING: 'RACING',
  FINISHED: 'FINISHED',
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
  NOMINAL:     'NODE_HEALTH_NOMINAL',
  WATCHDOG:    'NODE_HEALTH_WATCHDOG',
  REPLANNING:  'NODE_HEALTH_REPLANNING',
} as const;

export type NodeHealthValue = typeof NodeHealth[keyof typeof NodeHealth];

// Detection class names emitted by the vision pipeline.
export const DetectionClass = {
  RED_SIGN: 'red_sign',
  GREEN_SIGN: 'green_sign',
} as const;

export type DetectionClassValue = typeof DetectionClass[keyof typeof DetectionClass];

// A point/vector in Three.js space (X right, Y up, Z toward camera).
export type Vec3 = [number, number, number]

// Position3D is a proto message {x, y, z} — not a tuple.
export interface Position3D {
  x: number
  y: number
  z: number
}

export interface TopicUpdate {
  topic_name: string
  message_type: typeof RosMessageType[keyof typeof RosMessageType] | string
  timestamp: string  // ISO 8601 (google.protobuf.Timestamp via protojson)
  update_rate_hz: number
  data: Record<string, unknown>
}

export interface TopicsSnapshot {
  timestamp: string  // ISO 8601
  topics: Array<TopicUpdate>
}

export interface ImuData {
  linear_acceleration: Position3D  // m/s² (x, y, z)
  angular_velocity: Position3D     // rad/s (roll, pitch, yaw)
  orientation_x: number
  orientation_y: number
  orientation_z: number
  orientation_w: number
}

export interface Detection {
  class_name: string   // "red_sign" | "green_sign"
  confidence: number   // [0, 1]
  bbox_x: number       // normalized bounding box
  bbox_y: number
  bbox_w: number
  bbox_h: number
}

export interface MotorState {
  steering_angle: number    // radians
  drive_speed: number       // motor speed 0-100
  encoder_position: number  // encoder ticks
}

export interface TelemetryMetrics {
  timestamp: string      // ISO 8601
  node_health: NodeHealthValue

  points_captured?: number
  range_min?: number | null
  range_max?: number | null
  range_mean?: number | null
  forward?: number | null
  left?: number | null
  right?: number | null
  back?: number | null
  speed?: number | null
  stage?: string

  lidar_available: boolean
  imu_available: boolean
  camera_available: boolean
  odometry_available: boolean
}

export interface RobotSnapshot {
  timestamp: string      // ISO 8601
  mission_name: string

  robot_position?: Position3D | null
  robot_orientation?: number | null

  lidar_points: Array<Position3D>
  path_history: Array<Position3D>
  logs: Array<string>

  metrics: TelemetryMetrics

  imu_data?: ImuData | null
  vision_detections?: Array<Detection> | null
  motor_state?: MotorState | null
}

export interface ReplaySessionInfo {
  session_id: string
  created_at: string  // ISO 8601
  entry_count: number
}

// ============================================================================
// ROS MESSAGE PAYLOADS (raw TopicUpdate.data shapes)
//
// These model the untyped `data` carried by each TopicUpdate so the topic
// visualizers — and the demo data generators — share one definition instead of
// reaching into `Record<string, any>`.
// ============================================================================

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

/** sensor_msgs/LaserScan */
export interface LaserScanMsg {
  ranges: number[]
  angle_min: number
  angle_max: number
  angle_increment: number
  range_min: number
  range_max: number
}

/** geometry_msgs/Twist */
export interface TwistMsg {
  linear: Position3D
  angular: Position3D
}

/** sensor_msgs/Imu */
export interface ImuMsg {
  orientation: Quaternion
  linear_acceleration: Position3D
  angular_velocity: Position3D
}

/** sensor_msgs/JointState */
export interface JointStateMsg {
  name: string[]
  position: number[]
  velocity?: number[]
  effort?: number[]
}

/** nav_msgs/Odometry (subset used by the dashboard) */
export interface OdometryMsg {
  pose: { position: Position3D; orientation: Partial<Quaternion> }
  twist: { linear: Partial<Position3D>; angular: Partial<Position3D> }
}

/** std_msgs/String */
export interface StringMsg {
  data: string
}

/** vision_msgs/Detection2DArray (subset used by the dashboard) */
export interface Detection2DMsg {
  results: Array<{ hypothesis: { class_id: string; score: number } }>
  bbox: {
    center: { position: { x: number; y: number } }
    size_x: number
    size_y: number
  }
}

export interface Detection2DArrayMsg {
  detections: Detection2DMsg[]
}
