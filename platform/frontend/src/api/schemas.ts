/**
 * src/api/schemas.ts
 * 
 * Zod validation schemas for API responses.
 * Provides runtime type checking for all telemetry data received from the backend.
 * Enables early error detection and safe type narrowing without `any` casts.
 * 
 * Phase 3: Type Safety - Runtime validation
 */

import { z } from 'zod'
import {
  type TopicUpdate,
  type TopicsSnapshot,
  type ImuData,
  type Detection,
  type MotorState,
  type TelemetryMetrics,
  type RobotSnapshot,
  type ReplaySessionInfo,
  type Position3D,
} from '../types'

/**
 * Position in 3D space [x, y, z]
 */
const Position3DSchema = z.tuple([z.number(), z.number(), z.number()]) as z.ZodType<Position3D>

/**
 * IMU sensor data
 */
const ImuDataSchema = z.object({
  linearAcceleration: z.tuple([z.number(), z.number(), z.number()]),
  angularVelocity: z.tuple([z.number(), z.number(), z.number()]),
  orientationQuaternion: z.tuple([z.number(), z.number(), z.number(), z.number()]),
}) as z.ZodType<ImuData>

/**
 * Vision detection bounding box with confidence
 */
const DetectionSchema = z.object({
  className: z.string(),
  confidence: z.number().min(0).max(1),
  bbox: z.tuple([z.number(), z.number(), z.number(), z.number()]),
}) as z.ZodType<Detection>

/**
 * Motor state including steering and drive
 */
const MotorStateSchema = z.object({
  steeringAngle: z.number(),
  driveSpeed: z.number(),
  encoderPosition: z.number(),
}) as z.ZodType<MotorState>

/**
 * Telemetry metrics collected from robot
 */
const TelemetryMetricsSchema = z.object({
  timestamp: z.number().int().positive(),
  nodeHealth: z.enum(['nominal', 'watchdog', 'replanning', 'unknown']).default('unknown'),

  // Optional sensor metrics
  pointsCaptured: z.number().int().nonnegative().optional(),
  rangeMin: z.number().nullable().optional(),
  rangeMax: z.number().nullable().optional(),
  rangeMean: z.number().nullable().optional(),
  forward: z.number().nullable().optional(),
  left: z.number().nullable().optional(),
  right: z.number().nullable().optional(),
  back: z.number().nullable().optional(),
  speed: z.number().nullable().optional(),
  stage: z.string().optional(),

  // Hardware health flags
  lidarAvailable: z.boolean().default(false),
  imuAvailable: z.boolean().default(false),
  cameraAvailable: z.boolean().default(false),
  odometryAvailable: z.boolean().default(false),
}) as z.ZodType<TelemetryMetrics>

/**
 * ROS topic update from telemetry stream
 */
const TopicUpdateSchema = z.object({
  topicName: z.string().nonempty(),
  messageType: z.string(),
  timestamp: z.number().int().positive(),
  updateRateHz: z.number().positive(),
  data: z.record(z.string(), z.unknown()),
}) as z.ZodType<TopicUpdate>

/**
 * Snapshot of all topics at a given timestamp
 */
const TopicsSnapshotSchema = z.object({
  timestamp: z.number().int().positive(),
  topics: z.array(TopicUpdateSchema),
}) as z.ZodType<TopicsSnapshot>

/**
 * Complete robot state snapshot with sensor data
 */
const RobotSnapshotSchema = z.object({
  timestamp: z.number().int().positive(),
  missionName: z.string().default('unknown'),

  // Optional position data
  robotPosition: Position3DSchema.nullable().optional(),
  robotOrientation: z.number().nullable().optional(),

  // Sensor data arrays
  lidarPoints: z.array(Position3DSchema).default([]),
  pathHistory: z.array(Position3DSchema).default([]),
  logs: z.array(z.string()).default([]),

  // Core metrics
  metrics: TelemetryMetricsSchema,

  // Extended optional telemetry
  imuData: ImuDataSchema.nullable().optional(),
  visionDetections: z.array(DetectionSchema).nullable().optional(),
  motorState: MotorStateSchema.nullable().optional(),
}) as z.ZodType<RobotSnapshot>

/**
 * Session information for replay
 */
const ReplaySessionInfoSchema = z.object({
  sessionId: z.string().uuid().or(z.string().nonempty()),
  createdAt: z.number().int().positive(),
  entryCount: z.number().int().nonnegative(),
}) satisfies z.ZodType<ReplaySessionInfo>

/**
 * Array of sessions
 */
const SessionsResponseSchema = z.array(ReplaySessionInfoSchema)

/**
 * Generic API error response
 */
const ErrorResponseSchema = z.object({
  error: z.string(),
  message: z.string().optional(),
  statusCode: z.number().int().optional(),
})

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
}

/**
 * Validates and narrows a value to RobotSnapshot type with full type safety
 */
export function validateRobotSnapshot(data: unknown): RobotSnapshot {
  return RobotSnapshotSchema.parse(data)
}

/**
 * Validates and narrows a value to ReplaySessionInfo type
 */
export function validateReplaySessionInfo(data: unknown): ReplaySessionInfo {
  return ReplaySessionInfoSchema.parse(data)
}

/**
 * Validates and narrows a value to array of sessions
 */
export function validateSessions(data: unknown): ReplaySessionInfo[] {
  return SessionsResponseSchema.parse(data)
}

/**
 * Safely validates data with error handling instead of throwing
 */
export function safeValidateRobotSnapshot(data: unknown): {
  success: true
  data: RobotSnapshot
} | {
  success: false
  errors: z.ZodError['issues']
} {
  const result = RobotSnapshotSchema.safeParse(data)
  if (result.success) {
    return { success: true, data: result.data }
  }
  return { success: false, errors: result.error.issues }
}
