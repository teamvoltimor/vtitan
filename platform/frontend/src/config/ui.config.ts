/**
 * src/config/ui.config.ts
 *
 * UI-facing strings, panels, and controls. Nothing here touches the
 * transport or the 3D scene; see api.config.ts and scene.config.ts.
 */

// UI STRINGS

export const UI_STRINGS = {
  SENSOR_STATUS: 'SENSOR STATUS',
  LIVE_TRACKING: 'LIVE TRACKING',
  SPEED_CONTROL: 'MAX LINEAR SPEED',
  EVENT_FEED: 'Event Feed',
  NODE_BRIDGE: 'Node Bridge',
  REPLAYS: 'Replays',
  FILTER_TOPICS: 'Filter topics...',
  VISION_DEBUG: 'VISION DEBUG STREAM',
  TELEMETRY_CHANNEL: 'TELEMETRY CHANNEL',
  COMMAND_CHANNEL: 'COMMAND CHANNEL',
} as const;

// SENSOR CONFIGURATION

export const SENSOR_CONFIG = [
  { id: 'lidar', name: 'LiDAR', key: 'lidar_available' as const },
  { id: 'imu', name: 'IMU', key: 'imu_available' as const },
  { id: 'camera', name: 'Camera', key: 'camera_available' as const },
  { id: 'odometry', name: 'Odometry', key: 'odometry_available' as const },
] as const;

// ROBOT SPEED CONTROL

export const SPEED_CONTROL_CONFIG = {
  MIN: 0,
  MAX: 2,
  STEP: 0.1,
  DEFAULT: 1.0,
} as const;

// SPEED CONTROL VALIDATION

export const SPEED_CONTROL_DEBOUNCE_MS = 250;

// JSON VIEW CONFIGURATION

export const JSON_VIEW_CONFIG = {
  ARRAY_PREVIEW_LIMIT: 10,
  ARRAY_EXPAND_STEP: 20,
  DECIMAL_PRECISION: 4,
  INDENT_PER_LEVEL: 16,
  MAX_DEPTH: 20, // recursion guard — ROS messages are shallow, but arbitrary payloads need not be
} as const;
