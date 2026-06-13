/**
 * src/config/app.config.ts
 * 
 * Centralized application configuration.
 * All magic numbers, strings, and constants are defined here.
 * This eliminates hardcoded values scattered throughout the codebase.
 */

// ============================================================================
// SIMULATION & 3D RENDERING
// ============================================================================

export const SIMULATION_CONFIG = {
  // Track geometry
  TRACK: {
    CENTER: { x: 1.5, y: 1.5, z: 0 },  // Origin of simulation coordinate system
    SIZE: { width: 3.2, height: 3.2 },
  },

  // Robot model & defaults
  ROBOT: {
    DEFAULT_POSITION: [1.5, 1.5, 0.1] as const,
    DEFAULT_ORIENTATION_RADIANS: 0,
    DIMENSIONS: [0.2, 0.08, 0.14] as const,  // [width, height, depth]
  },

  // 3D camera configuration
  CAMERA: {
    POSITION: [0, 5, 3] as const,
    FOV: 45,
    MAX_POLAR_ANGLE: Math.PI / 2.1,  // Prevent camera from going below ground
  },
} as const

// ============================================================================
// LIDAR VISUALIZATION
// ============================================================================

export const LIDAR_CONFIG = {
  MAX_RANGE_METERS: 2.0,

  POINT_CLOUD: {
    POINT_SIZE: 0.03,
    OPACITY: {
      AVAILABLE: 0.9,
      UNAVAILABLE: 0.3,
    },
    NO_DATA_RADIUS: 0.05,  // Fallback sphere when no LIDAR data
  },

  RADAR_CHART: {
    CANVAS_WIDTH: 200,
    CANVAS_HEIGHT: 200,
    GRID_STEP_METERS: 0.5,
    DOT_SIZE_PIXELS: 2,
    DOT_OFFSET_PIXELS: 1,
    COLORS: {
      GRID: 'rgba(255, 255, 255, 0.1)',
      FORWARD_INDICATOR: 'rgba(255, 255, 255, 0.3)',
      DANGER: '#ff4444',      // < 0.3 meters
      WARNING: '#ffaa00',     // 0.3-0.8 meters
      SAFE: '#44ff44',        // > 0.8 meters
    },
    DISTANCE_THRESHOLDS: {
      DANGER: 0.3,
      WARNING: 0.8,
    },
  },
} as const

// ============================================================================
// ROBOT PATH VISUALIZATION
// ============================================================================

export const ROBOT_PATH_CONFIG = {
  Y_OFFSET: 0.03,           // Vertical offset from ground
  TUBE_SEGMENTS: 80,        // Number of segments in tube geometry
  TUBE_RADIUS: 0.01,        // Radius of the path tube
  TUBE_RADIAL_SEGMENTS: 5,  // Radial subdivisions
} as const

// ============================================================================
// SPEED GAUGE VISUALIZATION
// ============================================================================

export const SPEED_GAUGE_CONFIG = {
  CANVAS_WIDTH: 200,
  CANVAS_HEIGHT: 120,
  VIEWBOX: '0 0 200 120',
  ARC: {
    RADIUS: 80,
    START_X: 20,
    START_Y: 100,
  },
  NEEDLE_LENGTH: 70,
  MAX_LINEAR_SPEED: 0.60,    // Max speed shown on gauge
  STEERING_SENSITIVITY: 2.0,  // Divider for angular speed (higher = less sensitive)
} as const

// ============================================================================
// ROBOT SPEED CONTROL
// ============================================================================

export const SPEED_CONTROL_CONFIG = {
  MIN: 0,
  MAX: 2,
  STEP: 0.1,
  DEFAULT: 1.0,
} as const

// ============================================================================
// API CONFIGURATION
// ============================================================================

export const API_CONFIG = {
  BASE_URL: (import.meta.env.VITE_TELEMETRY_BASE ?? '').replace(/\/$/, ''),

  ENDPOINTS: {
    LATEST:      '/v1/telemetry/latest',
    TOPICS:      '/v1/telemetry/topics',
    HISTORY:     '/v1/telemetry/history',
    SESSIONS:    '/v1/telemetry/sessions',
    SESSION:     (id: string) => `/v1/telemetry/sessions/${id}`,
    ROBOT_SPEED: '/v1/telemetry/robot/config/speed',
    STREAM:      '/v1/telemetry/ws',
  },

  FETCH_CACHE: 'no-store' as const,
  TIMEOUT_MS: 30000,

  WEBSOCKET: {
    RECONNECT_DELAY_MS: 2000,
    RECONNECT_MAX_ATTEMPTS: 5,
    RECONNECT_BACKOFF_MULTIPLIER: 1.5,
  },
} as const

// ============================================================================
// PROTOCOL CONVERSION
// ============================================================================

export const URL_PROTOCOL_MAP = {
  'http://': 'ws://',
  'https://': 'wss://',
} as const

// ============================================================================
// UI STRINGS
// ============================================================================

export const UI_STRINGS = {
  SENSOR_STATUS: 'SENSOR STATUS',
  LIVE_TRACKING: 'LIVE TRACKING',
  SPEED_CONTROL: 'MAX LINEAR SPEED',
  EVENT_FEED: 'Event Feed',
  NODE_BRIDGE: 'Node Bridge',
  REPLAYS: 'Replays',
  GO_LIVE: 'Go live',
  LIVE: 'Live',
  FILTER_TOPICS: 'Filter topics...',
} as const

// ============================================================================
// TELEMETRY SETTINGS
// ============================================================================

export const TELEMETRY_CONFIG = {
  HISTORY_MAX_SIZE: 60,                    // Max snapshots kept in memory
  TOPIC_STALENESS_THRESHOLD_SECONDS: 2.0, // Mark topic stale after this duration
  POLL_INTERVAL_MS: parseInt(
    import.meta.env.VITE_POLL_INTERVAL_MS ?? '2500'
  ),
} as const

// ============================================================================
// THEME COLORS
// ============================================================================

export const THEME = {
  COLORS: {
    BACKGROUND: '#050b12',
    PANEL: '#0f1c2b',
    ACCENT: '#ff8a65',
    HIGHLIGHT: '#5fdde5',
    ERROR: '#ff0000',
    SUCCESS: '#4caf50',
    WARNING: '#ffaa00',
  },
} as const

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

export const SENSOR_CONFIG = [
  {
    id: 'lidar',
    name: 'LiDAR',
    icon: '📡',
    key: 'lidar_available' as const,
  },
  {
    id: 'imu',
    name: 'IMU',
    icon: '🧭',
    key: 'imu_available' as const,
  },
  {
    id: 'camera',
    name: 'Camera',
    icon: '📷',
    key: 'camera_available' as const,
  },
  {
    id: 'odometry',
    name: 'Odometry',
    icon: '⚙️',
    key: 'odometry_available' as const,
  },
] as const

// ============================================================================
// IMU VISUALIZATION
// ============================================================================

export const IMU_METRICS_CONFIG = [
  { label: 'Accel X', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.x' },
  { label: 'Accel Y', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.y' },
  { label: 'Accel Z', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.z' },
  { label: 'Gyro X', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.x' },
  { label: 'Gyro Y', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.y' },
  { label: 'Gyro Z', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.z' },
] as const

// ============================================================================
// MOTOR DIALS VISUALIZATION
// ============================================================================

export const MOTOR_DIALS_CONFIG = {
  CANVAS_WIDTH: 120,
  CANVAS_HEIGHT: 120,
  VIEWBOX: '0 0 120 120',
  CIRCLE_RADIUS: 50,
  CIRCLE_CENTER: { x: 60, y: 60 },
} as const

// ============================================================================
// VISION VISUALIZATION
// ============================================================================

export const VISION_CONFIG = {
  CANVAS_WIDTH: 320,
  CANVAS_HEIGHT: 240,
  VIEWBOX: '0 0 640 480',
  LABEL_WIDTH_PER_CHAR: 10,
  COLORS: {
    RED: '#ff4444',
    GREEN: '#4caf50',
    BLUE: '#2196f3',
  },
} as const

// ============================================================================
// JSON VIEW CONFIGURATION
// ============================================================================

export const JSON_VIEW_CONFIG = {
  ARRAY_PREVIEW_LIMIT: 10,
  DECIMAL_PRECISION: 4,
  INDENT_PER_LEVEL: 16,
} as const
