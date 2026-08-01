/**
 * src/api/schemas.ts
 *
 * Zod validation schemas for API responses from the Go backend (protojson, snake_case).
 * Field names and enum values match proto definitions in telemetry/v1/types.proto.
 */

import { z } from 'zod';
import type {
  Detection,
  HealthResponse,
  ImuData,
  MotorState,
  Position3d,
  SessionResponse,
  TelemetryMetrics,
  TopicsSnapshot,
  TopicUpdate,
} from '../api/generated';
import type { RobotSnapshot } from '../types';

// Timestamp emitted by google.protobuf.Timestamp via protojson — always ISO 8601 string.
const TimestampSchema = z.string().datetime({ offset: true });

// Position3D is a proto message {x, y, z}, not a tuple.
const Position3DSchema = z.object({
  x: z.number(),
  y: z.number(),
  z: z.number(),
}) as z.ZodType<Position3d>;

const ImuDataSchema = z.object({
  linear_acceleration: Position3DSchema,
  angular_velocity: Position3DSchema,
  orientation_x: z.number(),
  orientation_y: z.number(),
  orientation_z: z.number(),
  orientation_w: z.number(),
}) as z.ZodType<ImuData>;

const DetectionSchema = z.object({
  class_name: z.string(),
  confidence: z.number().min(0).max(1),
  bbox_x: z.number(),
  bbox_y: z.number(),
  bbox_w: z.number(),
  bbox_h: z.number(),
}) as z.ZodType<Detection>;

const MotorStateSchema = z.object({
  steering_angle: z.number(),
  drive_speed: z.number(),
  encoder_position: z.number().int(),
}) as z.ZodType<MotorState>;

// Proto enum names emitted by protojson (e.g. "NODE_HEALTH_NOMINAL").
const NodeHealthSchema = z
  .enum([
    'NODE_HEALTH_UNSPECIFIED',
    'NODE_HEALTH_NOMINAL',
    'NODE_HEALTH_WATCHDOG',
    'NODE_HEALTH_REPLANNING',
  ])
  .default('NODE_HEALTH_UNSPECIFIED');

const TelemetryMetricsSchema = z.object({
  timestamp: TimestampSchema,
  node_health: NodeHealthSchema,

  points_captured: z.number().int().nonnegative().optional(),
  range_min: z.number().nullable().optional(),
  range_max: z.number().nullable().optional(),
  range_mean: z.number().nullable().optional(),
  forward: z.number().nullable().optional(),
  left: z.number().nullable().optional(),
  right: z.number().nullable().optional(),
  back: z.number().nullable().optional(),
  speed: z.number().nullable().optional(),
  stage: z.string().optional(),

  lidar_available: z.boolean().default(false),
  imu_available: z.boolean().default(false),
  camera_available: z.boolean().default(false),
  odometry_available: z.boolean().default(false),
}) as z.ZodType<TelemetryMetrics>;

const TopicUpdateSchema = z.object({
  topic_name: z.string().nonempty(),
  message_type: z.string(),
  timestamp: TimestampSchema,
  // Nonnegative, not positive: a topic that has stopped publishing legitimately
  // reads 0 Hz and must survive validation (staleness rendering handles it).
  update_rate_hz: z.number().nonnegative(),
  data: z.record(z.string(), z.unknown()),
}) as z.ZodType<TopicUpdate>;

const TopicsSnapshotSchema = z.object({
  timestamp: TimestampSchema,
  topics: z.array(TopicUpdateSchema),
}) as z.ZodType<TopicsSnapshot>;

const RobotSnapshotSchema = z.object({
  timestamp: TimestampSchema,
  mission_name: z.string().default('unknown'),

  robot_position: Position3DSchema.nullable().optional(),
  robot_orientation: z.number().nullable().optional(),

  lidar_points: z.array(Position3DSchema).default([]),
  path_history: z.array(Position3DSchema).default([]),
  logs: z.array(z.string()).default([]),

  metrics: TelemetryMetricsSchema,

  imu_data: ImuDataSchema.nullable().optional(),
  vision_detections: z.array(DetectionSchema).nullable().optional(),
  motor_state: MotorStateSchema.nullable().optional(),
}) as z.ZodType<RobotSnapshot>;

const ReplaySessionInfoSchema = z.object({
  session_id: z.string().nonempty(),
  created_at: TimestampSchema,
  entry_count: z.number().int().nonnegative(),
}) satisfies z.ZodType<SessionResponse>;

const SessionsResponseSchema = z.array(ReplaySessionInfoSchema);

const ErrorResponseSchema = z.object({
  error: z.string(),
  message: z.string().optional(),
  statusCode: z.number().int().optional(),
});

const HealthResponseSchema = z.object({
  status: z.string(),
  version: z.string(),
}) as z.ZodType<HealthResponse>;

export const schemas = {
  Position3D: Position3DSchema,
  ImuData: ImuDataSchema,
  Detection: DetectionSchema,
  MotorState: MotorStateSchema,
  TelemetryMetrics: TelemetryMetricsSchema,
  TopicUpdate: TopicUpdateSchema,
  TopicsSnapshot: TopicsSnapshotSchema,
  RobotSnapshot: RobotSnapshotSchema,
  ReplaySessionInfo: ReplaySessionInfoSchema,
  SessionsResponse: SessionsResponseSchema,
  ErrorResponse: ErrorResponseSchema,
  HealthResponse: HealthResponseSchema,
};

export function validateRobotSnapshot(data: unknown): RobotSnapshot {
  return RobotSnapshotSchema.parse(data);
}

export function validateReplaySessionInfo(data: unknown): SessionResponse {
  return ReplaySessionInfoSchema.parse(data);
}

export function validateSessions(data: unknown): SessionResponse[] {
  return SessionsResponseSchema.parse(data);
}

/**
 * Parse a TopicsSnapshot tolerantly: each topic is validated individually and
 * structurally invalid entries are dropped with a console warning, so one bad
 * topic (e.g. a zero or malformed update rate) can't reject the whole snapshot
 * and blank every topic — audit §5.5. Returns null only when the envelope
 * itself is unrecognisable.
 */
export function safeParseTopicsSnapshot(data: unknown): TopicsSnapshot | null {
  if (typeof data !== 'object' || data === null) return null;
  const raw = data as Record<string, unknown>;

  const timestamp = TimestampSchema.safeParse(raw.timestamp);
  if (!timestamp.success) return null;
  if (!Array.isArray(raw.topics)) return null;

  const topics: TopicUpdate[] = [];
  for (const item of raw.topics) {
    const parsed = TopicUpdateSchema.safeParse(item);
    if (parsed.success) {
      topics.push(parsed.data);
    } else {
      console.warn('Dropping invalid topic entry:', parsed.error.issues);
    }
  }

  return { timestamp: timestamp.data, topics };
}

export function safeValidateRobotSnapshot(data: unknown):
  | {
      success: true;
      data: RobotSnapshot;
    }
  | {
      success: false;
      errors: z.ZodError['issues'];
    } {
  const result = RobotSnapshotSchema.safeParse(data);
  if (result.success) {
    return { success: true, data: result.data };
  }
  return { success: false, errors: result.error.issues };
}
