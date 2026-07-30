/**
 * src/api/robot.ts
 *
 * Robot fleet-management API client. Served by the same backend process and
 * base URL as telemetry.ts (internal/edge mounts both route groups on one
 * gin.Engine) — no separate backend base URL needed.
 */

import type { Robot, RobotCommandResponse, SetVisionDebugParams } from './generated/robot';
import { TelemetryError, classifyHttpError } from './errors';
import { API_CONFIG } from '../config';
import { getErrorMessage } from '../utils/formatting';

const resolveUrl = (path: string): string => {
  const baseUrl = API_CONFIG.BASE_URL;
  return baseUrl ? `${baseUrl}${path}` : path;
};

async function postJson<T>(path: string, body: unknown): Promise<T> {
  try {
    const response = await fetch(resolveUrl(path), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(API_CONFIG.TIMEOUT_MS),
    });

    if (!response.ok) {
      const code = classifyHttpError(response.status);
      throw new TelemetryError(code, `HTTP ${response.status}: ${response.statusText}`, response.status);
    }

    return (await response.json()) as T;
  } catch (err) {
    if (err instanceof TelemetryError) throw err;
    throw new TelemetryError('NETWORK', `Network error: ${getErrorMessage(err)}`, undefined, err);
  }
}

/**
 * This is a single-robot project (see backend's `defaultRobotName`/
 * `DefaultRobotID`) — fetch the one registered robot rather than surfacing a
 * fleet picker the product doesn't have.
 */
export async function fetchDefaultRobotId(): Promise<string> {
  const response = await fetch(resolveUrl(API_CONFIG.ENDPOINTS.ROBOTS), {
    signal: AbortSignal.timeout(API_CONFIG.TIMEOUT_MS),
  });
  if (!response.ok) {
    const code = classifyHttpError(response.status);
    throw new TelemetryError(code, `HTTP ${response.status}: ${response.statusText}`, response.status);
  }
  const robots = (await response.json()) as Robot[];
  if (robots.length === 0) {
    throw new TelemetryError('NETWORK', 'No robots registered');
  }
  return robots[0].id;
}

/** Toggle the robot's vision debug annotated-image stream at runtime. */
export const setVisionDebug = (
  robotId: string,
  parameters: SetVisionDebugParams
): Promise<RobotCommandResponse> =>
  postJson(API_CONFIG.ENDPOINTS.ROBOT_COMMAND(robotId), {
    command_type: 'SET_VISION_DEBUG',
    parameters,
  });
