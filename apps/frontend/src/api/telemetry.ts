/**
 * src/api/telemetry.ts
 *
 * Telemetry API client. Request/response models come from the
 * OpenAPI-generated types (./generated); the shared HTTP client in ./http
 * handles transport, timeouts, error classification, and RFC 7807 surfacing.
 * Zod adds runtime validation of the wire data (audit §5.5 tolerant parsing).
 */

import type {
  HealthCheckResponses,
  ListSessionsResponses,
  UpdateRobotSpeedData,
  UpdateRobotSpeedResponses,
} from '../api/generated';
import { API_CONFIG } from '../config';
import type { RobotSnapshot, TopicsSnapshot } from '../types';
import { getErrorMessage } from '../utils/formatting';
import { ExponentialBackoff, TelemetryError } from './errors';
import { isRobotSnapshotShape } from './guards';
import { fetchJson, httpToWsUrl, postJson } from './http';
import { safeParseTopicsSnapshot, schemas } from './schemas';

// API endpoints

/**
 * Health check — an independent backstop for connection status. A wedged
 * backend process can hold a WebSocket connection open (no onerror/onclose
 * event fires) while failing to actually serve requests; polling this
 * catches that in a way the WS callbacks alone can't.
 */
export const fetchHealth = (): Promise<HealthCheckResponses['200']> =>
  fetchJson<HealthCheckResponses['200']>(API_CONFIG.ENDPOINTS.HEALTH, {
    signal: AbortSignal.timeout(API_CONFIG.HEALTH_CHECK_INTERVAL_MS),
    schema: schemas.HealthResponse,
  });

/**
 * Fetch the latest robot telemetry snapshot.
 *
 * Returns the frontend RobotSnapshot type: the wire's optional arrays are
 * made required by the Zod schema's `.default([])` guarantees.
 */
export const fetchLatestTelemetry = (): Promise<RobotSnapshot> =>
  fetchJson<RobotSnapshot>(API_CONFIG.ENDPOINTS.LATEST, {
    schema: schemas.RobotSnapshot,
  });

/**
 * Fetch current topic information.
 */
export const fetchRawTopics = async (): Promise<TopicsSnapshot> => {
  const data = await fetchJson<unknown>(API_CONFIG.ENDPOINTS.TOPICS);
  const parsed = safeParseTopicsSnapshot(data);
  if (parsed === null) {
    throw new TelemetryError('PARSE', 'Invalid topics snapshot response');
  }
  return parsed;
};

/**
 * Fetch telemetry history.
 */
/** Parse a list of snapshots tolerantly, dropping malformed entries. */
const parseSnapshotList = (data: unknown): RobotSnapshot[] => {
  if (!Array.isArray(data)) return [];
  const out: RobotSnapshot[] = [];
  for (const item of data) {
    const result = schemas.RobotSnapshot.safeParse(item);
    if (result.success) {
      out.push(result.data);
    } else {
      console.warn('Dropping invalid history snapshot:', result.error.issues);
    }
  }
  return out;
};

export const fetchHistory = (): Promise<RobotSnapshot[]> =>
  fetchJson<unknown>(API_CONFIG.ENDPOINTS.HISTORY).then(parseSnapshotList);

/**
 * Fetch available replay sessions.
 */
export const fetchSessions = (): Promise<ListSessionsResponses['200']> =>
  fetchJson<ListSessionsResponses['200']>(API_CONFIG.ENDPOINTS.SESSIONS, {
    schema: schemas.SessionsResponse,
  });

/**
 * Fetch snapshots for a specific replay session.
 */
export const fetchSession = (sessionId: string): Promise<RobotSnapshot[]> =>
  fetchJson<unknown>(API_CONFIG.ENDPOINTS.SESSION(sessionId)).then(parseSnapshotList);

/**
 * Update robot maximum linear speed.
 */
export const updateRobotSpeed = async (speed: number): Promise<void> => {
  const body: UpdateRobotSpeedData['body'] = { maxLinearSpeed: speed };
  await postJson<UpdateRobotSpeedResponses['200']>(API_CONFIG.ENDPOINTS.ROBOT_SPEED, body);
};

// WebSocket

/**
 * WebSocket message callback type.
 */
export type TelemetryMessageHandler = (data: RobotSnapshot | TopicsSnapshot) => void;

/**
 * WebSocket error callback type.
 */
export type TelemetryErrorHandler = (error: TelemetryError) => void;

/**
 * Connection success callback type.
 */
export type TelemetryConnectedHandler = () => void;

/**
 * Connect to live telemetry WebSocket stream.
 * Automatically reconnects with exponential backoff on connection loss.
 *
 * @param onMessage - Called when a valid message is received
 * @param onError - Called when an error occurs (optional)
 * @param onConnected - Called when connection is established (optional)
 * @returns Disconnect function to close the connection
 *
 * @example
 * const disconnect = connectTelemetryWS(
 *   (data) => console.log('Got data:', data),
 *   (error) => console.error('Error:', error.message),
 *   () => console.log('Connected')
 * )
 *
 * // Later, disconnect:
 * disconnect()
 */
export function connectTelemetryWS(
  onMessage: TelemetryMessageHandler,
  onError?: TelemetryErrorHandler,
  onConnected?: TelemetryConnectedHandler
): () => void {
  // Build WebSocket URL from the OpenAPI request model (GET /v1/telemetry/ws).
  const baseUrl = API_CONFIG.BASE_URL || window.location.origin;
  const wsUrl = httpToWsUrl(baseUrl) + API_CONFIG.ENDPOINTS.STREAM;

  let ws: WebSocket | null = null;
  let isClosed = false;

  // Exponential backoff for reconnection attempts. A live telemetry dashboard
  // should keep trying indefinitely (e.g. across a robot reboot) rather than
  // give up after a fixed attempt count and require a manual page refresh —
  // maxAttempts is Infinity so `canRetry` never goes false; the delay still
  // grows exponentially up to maxDelayMs.
  const backoff = new ExponentialBackoff(
    API_CONFIG.WEBSOCKET.RECONNECT_DELAY_MS,
    30000, // max delay of 30 seconds
    Number.POSITIVE_INFINITY,
    API_CONFIG.WEBSOCKET.RECONNECT_BACKOFF_MULTIPLIER
  );

  /**
   * Attempt to establish WebSocket connection.
   */
  const connect = () => {
    // Guard clause: connection explicitly closed
    if (isClosed) return;

    try {
      ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        // Reset backoff on successful connection
        backoff.reset();
        onConnected?.();
      };

      ws.onmessage = (event) => {
        try {
          const parsed = JSON.parse(event.data);
          // Discriminate structurally (only RobotSnapshot carries `metrics`)
          // before parsing, rather than trying RobotSnapshot, catching the
          // failure, and retrying against TopicsSnapshot on every message.
          try {
            // Topics go through safeParseTopicsSnapshot: per-topic tolerance so
            // one malformed topic can't reject the whole snapshot (§5.5).
            const validated = isRobotSnapshotShape(parsed)
              ? schemas.RobotSnapshot.parse(parsed)
              : safeParseTopicsSnapshot(parsed);
            if (validated !== null) {
              onMessage(validated);
              return;
            }
            throw new Error('Invalid TopicsSnapshot shape');
          } catch (validationErr) {
            const error = new TelemetryError(
              'PARSE',
              `Invalid WebSocket message structure: ${getErrorMessage(validationErr)}`,
              undefined,
              validationErr
            );
            onError?.(error);
          }
        } catch (err) {
          // Handle parse errors
          const error = new TelemetryError(
            'PARSE',
            `Failed to parse WebSocket message: ${getErrorMessage(err)}`,
            undefined,
            err
          );
          onError?.(error);
        }
      };

      ws.onerror = () => {
        // Same guard as onclose below: closing a socket that's still
        // CONNECTING (the disconnect function runs `ws.close()` regardless of
        // readyState) makes the browser fire `error` before `close`. Without
        // this, a deliberate teardown is reported to the caller as a genuine
        // network failure — which under React StrictMode's dev-mode
        // mount/unmount/remount cycle happens on every single page load.
        if (isClosed) return;
        const error = new TelemetryError('NETWORK', 'WebSocket connection error');
        onError?.(error);
      };

      ws.onclose = () => {
        // Guard clause: connection was explicitly closed
        if (isClosed) return;

        // Always retry — connection loss is reported to the caller but is
        // never treated as fatal, so a robot reboot mid-run doesn't strand
        // the dashboard on a dead connection requiring a manual refresh.
        const { delay } = backoff.getNextDelay();
        const error = new TelemetryError(
          'NETWORK',
          `Live connection lost — reconnecting (attempt ${backoff.getAttemptCount()})`
        );
        onError?.(error);
        setTimeout(connect, delay);
      };
    } catch (err) {
      // Handle WebSocket creation errors
      const error = new TelemetryError(
        'NETWORK',
        `Failed to create WebSocket: ${getErrorMessage(err)}`,
        undefined,
        err
      );
      onError?.(error);

      // Attempt to reconnect
      if (!isClosed) {
        const { delay } = backoff.getNextDelay();
        setTimeout(connect, delay);
      }
    }
  };

  // Initiate connection
  connect();

  // Return disconnect function
  return () => {
    isClosed = true;
    ws?.close();
  };
}
