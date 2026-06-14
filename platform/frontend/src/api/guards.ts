/**
 * src/api/guards.ts
 *
 * Runtime type guards for the live stream, which multiplexes RobotSnapshot and
 * TopicsSnapshot over one channel. The wire format has no discriminator field,
 * so we discriminate structurally: only RobotSnapshot carries `metrics`, only
 * TopicsSnapshot carries `topics`.
 */

import type { RobotSnapshot, TopicsSnapshot } from '../types'

export type TelemetryMessage = RobotSnapshot | TopicsSnapshot

export function isRobotSnapshot(msg: TelemetryMessage): msg is RobotSnapshot {
  return 'metrics' in msg
}

export function isTopicsSnapshot(msg: TelemetryMessage): msg is TopicsSnapshot {
  return 'topics' in msg && !('metrics' in msg)
}
