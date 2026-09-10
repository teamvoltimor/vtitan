/**
 * src/config/scene.config.ts
 *
 * 3D scene, simulation, and visualizer geometry. Palette references come from
 * colors.ts; transport settings live in api.config.ts.
 */

import { COLORS } from './colors';

// SIMULATION & 3D RENDERING

export const SIMULATION_CONFIG = {
  // Track geometry
  TRACK: {
    CENTER: { x: 1.5, y: 1.5, z: 0 }, // Origin of simulation coordinate system
    SIZE: { width: 3.2, height: 3.2 },
  },

  // Robot model & defaults. Matches the measured chassis in
  // src/shared/config/robot.toml (chassis.length/width/height) — the
  // single source of truth for the real robot's physical dimensions.
  ROBOT: {
    DEFAULT_POSITION: { x: 1.5, y: 1.5, z: 0.1 }, // Position3D (sim coords)
    DEFAULT_ORIENTATION_RADIANS: 0,
    // [length, height, width] — length is the forward (local +X) axis.
    DIMENSIONS: [0.3, 0.1, 0.2] as const,
  },

  // 3D camera configuration
  CAMERA: {
    POSITION: [0, 5, 3] as const,
    FOV: 45,
    MAX_POLAR_ANGLE: Math.PI / 2.1, // Prevent camera from going below ground
  },
} as const;

// 3D SCENE (lighting, materials, track geometry)

export const SCENE_CONFIG = {
  AMBIENT_INTENSITY: 0.6,
  DIRECTIONAL: {
    INTENSITY: 1.1,
    // Three.js is Y-up; this must sit above the floor plane (y > 0) or the
    // scene has no key light and reads as flat, ambient-only.
    POSITION: [3, 4, 3] as const,
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
    HEADING_LENGTH: 0.08, // length of the forward-heading arrow (metres) — shorter than the chassis
    HEADING_RADIUS: 0.05, // base radius of the forward-heading cone (metres)
  },
  // WRO track: 3×3 m outer, 1×1 m inner square — rendered as low walls.
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

// LIDAR VISUALIZATION

export const LIDAR_CONFIG = {
  MAX_RANGE_METERS: 2.0,

  POINT_CLOUD: {
    POINT_SIZE: 0.03,
    // Slamtec C1 at ~0.25° angular resolution over 360° — fixed buffer capacity
    // so the GPU attribute is allocated once instead of reallocated per snapshot.
    MAX_POINTS: 1440,
    // The point cloud is only ever rendered when data is present (the no-data
    // fallback sphere handles the lidar_available=false case), so a single
    // opacity value suffices.
    OPACITY: 0.9,
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

// ROBOT PATH VISUALIZATION

export const ROBOT_PATH_CONFIG = {
  Y_OFFSET: 0.03, // Vertical offset from ground
} as const;

// SPEED GAUGE VISUALIZATION

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

// IMU VISUALIZATION

export const IMU_METRICS_CONFIG = [
  { label: 'Accel X', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.x' },
  { label: 'Accel Y', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.y' },
  { label: 'Accel Z', range: [-10, 10], unit: 'm/s²', key: 'linear_acceleration.z' },
  { label: 'Gyro X', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.x' },
  { label: 'Gyro Y', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.y' },
  { label: 'Gyro Z', range: [-5, 5], unit: 'rad/s', key: 'angular_velocity.z' },
] as const;

// MOTOR DIALS VISUALIZATION

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

// VISION VISUALIZATION

// Raspberry Pi Camera Module 3 Wide native resolution (16:9). The
// vision_msgs/Detection2DArray topic carries pixel-space bboxes with no
// per-message image dimensions, so this can't be derived from the wire data —
// it must track the camera's actual configured resolution. If the vision
// pipeline ever changes capture resolution, update this to match.
export const VISION_CONFIG = {
  CANVAS_WIDTH: 320,
  CANVAS_HEIGHT: 180,
  VIEWBOX: '0 0 1536 864',
  LABEL_WIDTH_PER_CHAR: 10,
  MIN_LABEL_WIDTH: 100,
  BBOX_STROKE_WIDTH: 4,
  IMG_WIDTH: 1536,
  IMG_HEIGHT: 864,
  COLORS: {
    RED: COLORS.DANGER,
    GREEN: COLORS.SUCCESS,
    BLUE: COLORS.BLUE,
  },
} as const;
