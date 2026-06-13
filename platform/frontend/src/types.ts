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
  node_health: NodeHealthValue | string

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
