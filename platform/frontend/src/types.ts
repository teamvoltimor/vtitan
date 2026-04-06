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

export const NodeHealth = {
  NOMINAL: 'nominal',
  WATCHDOG: 'watchdog',
  REPLANNING: 'replanning',
} as const;

export interface TopicUpdate {
  topicName: string
  messageType: typeof RosMessageType[keyof typeof RosMessageType] | string
  timestamp: number
  updateRateHz: number
  data: Record<string, any>
}

export interface TopicsSnapshot {
  timestamp: number
  topics: Array<TopicUpdate>
}

export interface ImuData {
  linearAcceleration: [number, number, number]
  angularVelocity: [number, number, number]
  orientationQuaternion: [number, number, number, number]
}

export interface Detection {
  className: string
  confidence: number
  bbox: [number, number, number, number]
}

export interface MotorState {
  steeringAngle: number
  driveSpeed: number
  encoderPosition: number
}

export interface TelemetryMetrics {
  timestamp: number
  nodeHealth: typeof NodeHealth[keyof typeof NodeHealth] | string
  
  // All sensor data is optional
  pointsCaptured?: number
  rangeMin?: number | null
  rangeMax?: number | null
  rangeMean?: number | null
  forward?: number | null
  left?: number | null
  right?: number | null
  back?: number | null
  speed?: number | null
  stage?: string
  
  // Hardware health flags
  lidarAvailable: boolean
  imuAvailable: boolean
  cameraAvailable: boolean
  odometryAvailable: boolean
}

export type Position3D = [number, number, number]

export interface RobotSnapshot {
  timestamp: number
  missionName: string
  
  // Optional position
  robotPosition?: Position3D | null
  robotOrientation?: number | null
  
  // Sensor data (always arrays, never undefined)
  lidarPoints: Array<Position3D>
  pathHistory: Array<Position3D>
  logs: Array<string>
  
  metrics: TelemetryMetrics
  
  // Extended telemetry
  imuData?: ImuData | null
  visionDetections?: Array<Detection> | null
  motorState?: MotorState | null
}

export interface ReplaySessionInfo {
  sessionId: string
  createdAt: number
  entryCount: number
}
