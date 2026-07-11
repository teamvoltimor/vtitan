/**
 * src/api/telemetry.ts
 *
 * Telemetry API client with improved error handling and configuration.
 * Includes Zod runtime validation for type safety.
 * Follows the architectural refactoring patterns recommended in the audit report.
 */

import type { ReplaySessionInfo, RobotSnapshot, TopicsSnapshot } from '../types';
import { TelemetryError, ExponentialBackoff, classifyHttpError } from './errors';
import { API_CONFIG, URL_PROTOCOL_MAP } from '../config';
import { getErrorMessage } from '../utils/formatting';
import { schemas } from './schemas';

/**
 * Resolve URL with base path configuration.
 */
const resolveUrl = (path: string): string => {
  const baseUrl = API_CONFIG.BASE_URL;
  return baseUrl ? `${baseUrl}${path}` : path;
};

/**
 * Convert HTTP protocol to WebSocket protocol.
 */
const httpToWsUrl = (url: string): string => {
  return Object.entries(URL_PROTOCOL_MAP).reduce(
    (acc, [httpProtocol, wsProtocol]) => acc.replace(httpProtocol, wsProtocol),
    url
  );
};

/**
 * Fetch JSON with error handling, classification, and optional validation.
 *
 * @throws TelemetryError with appropriate error code
 */
async function fetchJson<T>(
  path: string,
  signal?: AbortSignal,
  schema?: { parse: (data: unknown) => T }
): Promise<T> {
  try {
    const response = await fetch(resolveUrl(path), {
      cache: API_CONFIG.FETCH_CACHE,
      signal,
    });

    // Handle HTTP errors
    if (!response.ok) {
      const code = classifyHttpError(response.status);
      throw new TelemetryError(
        code,
        `HTTP ${response.status}: ${response.statusText}`,
        response.status
      );
    }

    // Parse JSON
    let data: unknown;
    try {
      data = await response.json();
    } catch (err) {
      throw new TelemetryError(
        'PARSE',
        `Invalid JSON response: ${getErrorMessage(err)}`,
        response.status,
        err
      );
    }

    // Validate with schema if provided
    if (schema) {
      try {
        return schema.parse(data);
      } catch (err) {
        throw new TelemetryError(
          'PARSE',
          `Invalid response structure: ${getErrorMessage(err)}`,
          response.status,
          err
        );
      }
    }

    return data as T;
  } catch (err) {
    // Re-throw TelemetryErrors as-is
    if (err instanceof TelemetryError) {
      throw err;
    }

    // Handle abort signals (timeouts)
    if (err instanceof Error && err.name === 'AbortError') {
      throw new TelemetryError('TIMEOUT', 'Request timeout', undefined, err);
    }

    // Classify all other errors as network errors
    throw new TelemetryError('NETWORK', `Network error: ${getErrorMessage(err)}`, undefined, err);
  }
}

// ============================================================================
// API ENDPOINTS
// ============================================================================

/**
 * Fetch the latest robot telemetry snapshot.
 */
export const fetchLatestTelemetry = (): Promise<RobotSnapshot> =>
  fetchJson(API_CONFIG.ENDPOINTS.LATEST, undefined, schemas.RobotSnapshot);

/**
 * Fetch current topic information.
 */
export const fetchRawTopics = (): Promise<TopicsSnapshot> =>
  fetchJson(API_CONFIG.ENDPOINTS.TOPICS, undefined, schemas.TopicsSnapshot);

/**
 * Fetch telemetry history.
 */
export const fetchHistory = (): Promise<RobotSnapshot[]> =>
  fetchJson(API_CONFIG.ENDPOINTS.HISTORY, undefined, {
    parse: (data: unknown) =>
      Array.isArray(data) ? data.map((item) => schemas.RobotSnapshot.parse(item)) : [],
  });

/**
 * Fetch available replay sessions.
 */
export const fetchSessions = (): Promise<ReplaySessionInfo[]> =>
  fetchJson(API_CONFIG.ENDPOINTS.SESSIONS, undefined, schemas.SessionsResponse);

/**
 * Fetch snapshots for a specific replay session.
 */
export const fetchSession = (sessionId: string): Promise<RobotSnapshot[]> =>
  fetchJson(API_CONFIG.ENDPOINTS.SESSION(sessionId), undefined, {
    parse: (data: unknown) =>
      Array.isArray(data) ? data.map((item) => schemas.RobotSnapshot.parse(item)) : [],
  });

/**
 * Update robot maximum linear speed.
 */
export const updateRobotSpeed = async (speed: number): Promise<void> => {
  try {
    const response = await fetch(resolveUrl(API_CONFIG.ENDPOINTS.ROBOT_SPEED), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ maxLinearSpeed: speed }),
    });

    if (!response.ok) {
      const code = classifyHttpError(response.status);
      throw new TelemetryError(
        code,
        `Failed to update robot speed: HTTP ${response.status}`,
        response.status
      );
    }

    // Parse response but don't require specific format
    await response.json();
  } catch (err) {
    if (err instanceof TelemetryError) {
      throw err;
    }

    throw new TelemetryError(
      'NETWORK',
      `Failed to update robot speed: ${getErrorMessage(err)}`,
      undefined,
      err
    );
  }
};

// ============================================================================
// WEBSOCKET
// ============================================================================

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
  // Build WebSocket URL
  const baseUrl = API_CONFIG.BASE_URL || window.location.origin;
  const wsUrl = httpToWsUrl(baseUrl) + API_CONFIG.ENDPOINTS.STREAM;

  let ws: WebSocket | null = null;
  let isClosed = false;

  // Exponential backoff for reconnection attempts
  const backoff = new ExponentialBackoff(
    API_CONFIG.WEBSOCKET.RECONNECT_DELAY_MS,
    30000, // max delay of 30 seconds
    API_CONFIG.WEBSOCKET.RECONNECT_MAX_ATTEMPTS,
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
          // Validate WebSocket message structure (could be RobotSnapshot or TopicsSnapshot)
          // For now, we accept either type by validating against RobotSnapshot first
          try {
            const validated = schemas.RobotSnapshot.parse(parsed);
            onMessage(validated);
          } catch {
            // If not a RobotSnapshot, try TopicsSnapshot
            try {
              const validated = schemas.TopicsSnapshot.parse(parsed);
              onMessage(validated);
            } catch (validationErr) {
              // If neither validates, report the error
              const error = new TelemetryError(
                'PARSE',
                `Invalid WebSocket message structure: ${getErrorMessage(validationErr)}`,
                undefined,
                validationErr
              );
              onError?.(error);
            }
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
        const error = new TelemetryError('NETWORK', 'WebSocket connection error');
        onError?.(error);
      };

      ws.onclose = () => {
        // Guard clause: connection was explicitly closed
        if (isClosed) return;

        // Attempt to reconnect with exponential backoff
        const { delay, canRetry } = backoff.getNextDelay();

        if (canRetry) {
          setTimeout(connect, delay);
        } else {
          // Max retry attempts exceeded
          const error = new TelemetryError(
            'NETWORK',
            `Failed to reconnect after ${backoff.getAttemptCount()} attempts`
          );
          onError?.(error);
        }
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
        const { delay, canRetry } = backoff.getNextDelay();
        if (canRetry) {
          setTimeout(connect, delay);
        }
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
