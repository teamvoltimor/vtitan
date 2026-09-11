/**
 * src/api/guards.ts
 *
 * Runtime type guards for the live stream, which multiplexes RobotSnapshot and
 * TopicsSnapshot over one channel. The wire format has no discriminator field,
 * so we discriminate structurally: only RobotSnapshot carries `metrics`, only
 * TopicsSnapshot carries `topics`.
 */

import type { RobotSnapshot, TopicsSnapshot } from '../types';

export type TelemetryMessage = RobotSnapshot | TopicsSnapshot;

export function isRobotSnapshot(msg: TelemetryMessage): msg is RobotSnapshot {
  return 'metrics' in msg;
}

export function isTopicsSnapshot(msg: TelemetryMessage): msg is TopicsSnapshot {
  return 'topics' in msg && !('metrics' in msg);
}

/**
 * Same structural discriminator as `isRobotSnapshot`, but usable on raw,
 * not-yet-validated JSON (e.g. straight from `JSON.parse`) so callers can
 * pick the right Zod schema before parsing instead of parsing against one
 * schema, catching the failure, and retrying against the other.
 */
export function isRobotSnapshotShape(data: unknown): boolean {
  return typeof data === 'object' && data !== null && 'metrics' in data;
}
