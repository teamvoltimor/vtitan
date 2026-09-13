/**
 * src/config/api.config.ts
 *
 * Transport-level settings for the Go backend. Endpoint URLs are typed
 * against the OpenAPI-generated request models (`*Data['url']` literals) via
 * `satisfies`, so a path change in the spec that isn't regenerated — or a
 * hand-typed path that drifts from it — fails to compile (audit §7.3 / §10.2).
 */

import type {
  GetLatestTelemetryData,
  GetLatestTopicsData,
  GetTelemetryHistoryData,
  HealthCheckData,
  ListSessionsData,
  StreamTelemetryData,
  UpdateRobotSpeedData,
} from '../api/generated';
import type { ListRobotsData } from '../api/generated/robot';

export const API_CONFIG = {
  BASE_URL: (import.meta.env.VITE_TELEMETRY_BASE ?? '').replace(/\/$/, ''),

  ENDPOINTS: {
    HEALTH: '/v1/telemetry/health' satisfies HealthCheckData['url'],
    LATEST: '/v1/telemetry/latest' satisfies GetLatestTelemetryData['url'],
    TOPICS: '/v1/telemetry/topics' satisfies GetLatestTopicsData['url'],
    HISTORY: '/v1/telemetry/history' satisfies GetTelemetryHistoryData['url'],
    SESSIONS: '/v1/telemetry/sessions' satisfies ListSessionsData['url'],
    ROBOT_SPEED: '/v1/telemetry/robot/config/speed' satisfies UpdateRobotSpeedData['url'],
    STREAM: '/v1/telemetry/ws' satisfies StreamTelemetryData['url'],
    ROBOTS: '/v1/robots' satisfies ListRobotsData['url'],
    SESSION: (sessionId: string) => `/v1/telemetry/sessions/${sessionId}`,
    ROBOT_COMMAND: (robotId: string) => `/v1/robots/${robotId}/command`,
  },

  FETCH_CACHE: 'no-store' as const,
  TIMEOUT_MS: 30000,

  // Independent backstop for connection status — a stalled/wedged backend
  // process can hold a WebSocket connection open (no onerror/onclose event)
  // while failing to actually serve requests; polling health catches that.
  HEALTH_CHECK_INTERVAL_MS: 5000,

  WEBSOCKET: {
    RECONNECT_DELAY_MS: 2000,
    RECONNECT_MAX_ATTEMPTS: 5,
    RECONNECT_BACKOFF_MULTIPLIER: 1.5,
  },
} as const;

// PROTOCOL CONVERSION

export const URL_PROTOCOL_MAP = {
  'http://': 'ws://',
  'https://': 'wss://',
} as const;

// TELEMETRY SETTINGS

export const TELEMETRY_CONFIG = {
  HISTORY_MAX_SIZE: 60, // Max snapshots kept in memory
  LOG_BUFFER_MAX_SIZE: 200, // Max accumulated log lines kept for the Event Feed
  TOPIC_STALENESS_THRESHOLD_SECONDS: 2.0, // Mark topic stale after this duration
  POLL_INTERVAL_MS: parseInt(import.meta.env.VITE_POLL_INTERVAL_MS ?? '2500', 10),
} as const;
