/**
 * src/config/app.config.ts
 *
 * Centralized application configuration.
 * All magic numbers, strings, and constants are defined here.
 * This eliminates hardcoded values scattered throughout the codebase.
 */

// ============================================================================
// COLOR PALETTE (single source of truth)
//
// Every colour used in JS/TSX derives from here. The CSS custom properties in
// index.css mirror these values for styling that lives in stylesheets.
// ============================================================================

export const COLORS = {
  BACKGROUND: '#050b12',
  PANEL: '#0f1c2b',
  SURFACE: '#111b27',
  ACCENT: '#ff8a65',
  HIGHLIGHT: '#5fdde5',
  WHITE: '#ffffff',
  BLUE: '#2196f3',
  DANGER: '#ff4444',
  WARNING: '#ffaa00',
  SUCCESS: '#4caf50',
  ERROR: '#ff0000',
  GRID_LINE: 'rgba(255, 255, 255, 0.1)',
  FORWARD_INDICATOR: 'rgba(255, 255, 255, 0.3)',
} as const;

// ============================================================================
// SIMULATION & 3D RENDERING
// ============================================================================

export const SIMULATION_CONFIG = {
  // Track geometry
  TRACK: {
    CENTER: { x: 1.5, y: 1.5, z: 0 }, // Origin of simulation coordinate system
    SIZE: { width: 3.2, height: 3.2 },
  },

  // Robot model & defaults
  ROBOT: {
    DEFAULT_POSITION: { x: 1.5, y: 1.5, z: 0.1 }, // Position3D (sim coords)
    DEFAULT_ORIENTATION_RADIANS: 0,
    DIMENSIONS: [0.2, 0.08, 0.14] as const, // [width, height, depth]
  },

  // 3D camera configuration
  CAMERA: {
    POSITION: [0, 5, 3] as const,
    FOV: 45,
    MAX_POLAR_ANGLE: Math.PI / 2.1, // Prevent camera from going below ground
  },
} as const;

// ============================================================================
// 3D SCENE (lighting, materials, track geometry)
// ============================================================================

export const SCENE_CONFIG = {
  AMBIENT_INTENSITY: 0.6,
  DIRECTIONAL: {
    INTENSITY: 1.1,
    POSITION: [3, -3, 4] as const,
  },
  FLOOR: {
    COLOR: COLORS.SURFACE,
    METALNESS: 0.4,
    ROUGHNESS: 0.7,
  },
  ROBOT: {
    COLOR_ACTIVE: '#f3c677',
    EMISSIVE_ACTIVE: '#e57f2e',
    COLOR_INACTIVE: '#666666',
    EMISSIVE_INACTIVE: '#333333',
    INACTIVE_OPACITY: 0.5,
    HEADING_LENGTH: 0.22, // length of the forward-heading arrow (metres)
  },
  // WRO track: 3×3 m outer, 1×2 m inner square — rendered as low walls.
  TRACK: {
    OUTER_MIN: 0,
    OUTER_MAX: 3,
    INNER_MIN: 1,
    INNER_MAX: 2,
    WALL_HEIGHT: 0.1,
    WALL_THICKNESS: 0.02,
    WALL_COLOR: '#1b2d44',
    WALL_OPACITY: 0.55,
    GRID_DIVISIONS: 6,
  },
  PATH: {
    EMISSIVE_INTENSITY: 0.6,
  },
  // Fallback marker shown when no LiDAR data is available.
  NO_DATA_SPHERE: {
    POSITION: [0, 0.5, 0] as const,
    SEGMENTS: 8,
  },
} as const;

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
    NO_DATA_RADIUS: 0.05, // Fallback sphere when no LIDAR data
  },

  RADAR_CHART: {
    CANVAS_WIDTH: 200,
    CANVAS_HEIGHT: 200,
    GRID_STEP_METERS: 0.5,
    DOT_SIZE_PIXELS: 2,
    DOT_OFFSET_PIXELS: 1,
    COLORS: {
      GRID: COLORS.GRID_LINE,
      FORWARD_INDICATOR: COLORS.FORWARD_INDICATOR,
      DANGER: COLORS.DANGER, // < 0.3 meters
      WARNING: COLORS.WARNING, // 0.3-0.8 meters
      SAFE: COLORS.SUCCESS, // > 0.8 meters
    },
    DISTANCE_THRESHOLDS: {
      DANGER: 0.3,
      WARNING: 0.8,
    },
  },
} as const;

// ============================================================================
// ROBOT PATH VISUALIZATION
// ============================================================================

export const ROBOT_PATH_CONFIG = {
  Y_OFFSET: 0.03, // Vertical offset from ground
  TUBE_SEGMENTS: 80, // Number of segments in tube geometry
  TUBE_RADIUS: 0.01, // Radius of the path tube
  TUBE_RADIAL_SEGMENTS: 5, // Radial subdivisions
} as const;

// ============================================================================
// SPEED GAUGE VISUALIZATION
// ============================================================================

export const SPEED_GAUGE_CONFIG = {
  CANVAS_WIDTH: 200,
  CANVAS_HEIGHT: 120,
  VIEWBOX: '0 0 200 120',
  CENTER: { x: 100, y: 100 },
  ARC: {
    RADIUS: 80,
    START_X: 20,
    START_Y: 100,
    END_X: 180,
    END_Y: 100,
    STROKE_WIDTH: 12,
  },
  // Static background arc (full sweep) — start/end match ARC above.
  BG_ARC_PATH: 'M 20 100 A 80 80 0 0 1 180 100',
  NEEDLE_LENGTH: 70,
  NEEDLE_STROKE: 2,
  HUB_RADIUS: 6,
  MAX_LINEAR_SPEED: 0.6, // Max speed shown on gauge
  STEERING_SENSITIVITY: 2.0, // Divider for angular speed (higher = less sensitive)
} as const;

// ============================================================================
// ROBOT SPEED CONTROL
// ============================================================================

export const SPEED_CONTROL_CONFIG = {
  MIN: 0,
  MAX: 2,
  STEP: 0.1,
  DEFAULT: 1.0,
} as const;

// ============================================================================
// API CONFIGURATION
// ============================================================================

export const API_CONFIG = {
  BASE_URL: (import.meta.env.VITE_TELEMETRY_BASE ?? '').replace(/\/$/, ''),

  ENDPOINTS: {
    LATEST: '/v1/telemetry/latest',
    TOPICS: '/v1/telemetry/topics',
    HISTORY: '/v1/telemetry/history',
    SESSIONS: '/v1/telemetry/sessions',
    SESSION: (id: string) => `/v1/telemetry/sessions/${id}`,
    ROBOT_SPEED: '/v1/telemetry/robot/config/speed',
    STREAM: '/v1/telemetry/ws',
  },

  FETCH_CACHE: 'no-store' as const,
  TIMEOUT_MS: 30000,

  WEBSOCKET: {
    RECONNECT_DELAY_MS: 2000,
    RECONNECT_MAX_ATTEMPTS: 5,
    RECONNECT_BACKOFF_MULTIPLIER: 1.5,
  },
} as const;

// ============================================================================
// PROTOCOL CONVERSION
// ============================================================================

export const URL_PROTOCOL_MAP = {
  'http://': 'ws://',
  'https://': 'wss://',
} as const;

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
} as const;

// ============================================================================
// TELEMETRY SETTINGS
// ============================================================================

export const TELEMETRY_CONFIG = {
  HISTORY_MAX_SIZE: 60, // Max snapshots kept in memory
  TOPIC_STALENESS_THRESHOLD_SECONDS: 2.0, // Mark topic stale after this duration
  POLL_INTERVAL_MS: parseInt(import.meta.env.VITE_POLL_INTERVAL_MS ?? '2500', 10),
} as const;

// ============================================================================
// THEME COLORS
// ============================================================================

export const THEME = {
  COLORS: {
    BACKGROUND: COLORS.BACKGROUND,
    PANEL: COLORS.PANEL,
    ACCENT: COLORS.ACCENT,
    HIGHLIGHT: COLORS.HIGHLIGHT,
    ERROR: COLORS.ERROR,
    SUCCESS: COLORS.SUCCESS,
    WARNING: COLORS.WARNING,
  },
} as const;

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

export const SENSOR_CONFIG = [
  { id: 'lidar', name: 'LiDAR', key: 'lidar_available' as const },
  { id: 'imu', name: 'IMU', key: 'imu_available' as const },
  { id: 'camera', name: 'Camera', key: 'camera_available' as const },
  { id: 'odometry', name: 'Odometry', key: 'odometry_available' as const },
] as const;

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
] as const;

// ============================================================================
// MOTOR DIALS VISUALIZATION
// ============================================================================

export const MOTOR_DIALS_CONFIG = {
  CANVAS_WIDTH: 120,
  CANVAS_HEIGHT: 120,
  VIEWBOX: '0 0 120 120',
  CIRCLE_RADIUS: 50,
  CIRCLE_CENTER: { x: 60, y: 60 },
  NEEDLE_LENGTH: 40,
  ARC_STROKE_WIDTH: 8,
  NEEDLE_STROKE: 2,
  HUB_RADIUS: 5,
  LABEL_Y: 75,
} as const;

// ============================================================================
// VISION VISUALIZATION
// ============================================================================

export const VISION_CONFIG = {
  CANVAS_WIDTH: 320,
  CANVAS_HEIGHT: 240,
  VIEWBOX: '0 0 640 480',
  LABEL_WIDTH_PER_CHAR: 10,
  MIN_LABEL_WIDTH: 100,
  BBOX_STROKE_WIDTH: 4,
  IMG_WIDTH: 640,
  IMG_HEIGHT: 480,
  COLORS: {
    RED: COLORS.DANGER,
    GREEN: COLORS.SUCCESS,
    BLUE: COLORS.BLUE,
  },
} as const;

// ============================================================================
// JSON VIEW CONFIGURATION
// ============================================================================

export const JSON_VIEW_CONFIG = {
  ARRAY_PREVIEW_LIMIT: 10,
  DECIMAL_PRECISION: 4,
  INDENT_PER_LEVEL: 16,
} as const;
