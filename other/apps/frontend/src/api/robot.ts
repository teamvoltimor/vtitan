/**
 * src/api/robot.ts
 *
 * Robot fleet-management API client. Served by the same backend process and
 * base URL as telemetry.ts (internal/edge mounts both route groups on one
 * gin.Engine) — no separate backend base URL needed.
 *
 * Request/response models come from the OpenAPI-generated types
 * (./generated/robot); this file only adapts them to the app's needs.
 */

import { API_CONFIG } from '../config';
import { TelemetryError } from './errors';
import type {
  ListRobotsResponses,
  RobotCommandResponse,
  SendRobotCommandData,
  SendRobotCommandResponses,
  SetTelemetryChannelParams,
  SetVisionDebugParams,
} from './generated/robot';
import { fetchJson, postJson } from './http';

/**
 * This is a single-robot project (see backend's `defaultRobotName`/
 * `DefaultRobotID`) — fetch the one registered robot rather than surfacing a
 * fleet picker the product doesn't have.
 *
 * The result is cached: every command previously paid an extra round trip to
 * /v1/robots before its POST, and the ID cannot change within a session in a
 * single-robot product (audit §10.2).
 */
let cachedRobotId: string | null = null;

export async function fetchDefaultRobotId(): Promise<string> {
  if (cachedRobotId !== null) return cachedRobotId;
  const robots = await fetchJson<ListRobotsResponses['200']>(API_CONFIG.ENDPOINTS.ROBOTS);
  if (robots.length === 0) {
    throw new TelemetryError('NETWORK', 'No robots registered');
  }
  cachedRobotId = robots[0].id;
  return cachedRobotId;
}

/** Toggle the robot's vision debug annotated-image stream at runtime. */
export const setVisionDebug = (
  robotId: string,
  parameters: SetVisionDebugParams
): Promise<RobotCommandResponse> => {
  const command: SendRobotCommandData['body'] = { command_type: 'SET_VISION_DEBUG', parameters };
  return postJson<SendRobotCommandResponses['202']>(
    API_CONFIG.ENDPOINTS.ROBOT_COMMAND(robotId),
    command
  );
};

/** Toggle the robot's telemetry gRPC stream at runtime — remotely reversible either way. */
export const setTelemetryChannel = (
  robotId: string,
  parameters: SetTelemetryChannelParams
): Promise<RobotCommandResponse> => {
  const command: SendRobotCommandData['body'] = {
    command_type: 'SET_TELEMETRY_CHANNEL',
    parameters,
  };
  return postJson<SendRobotCommandResponses['202']>(
    API_CONFIG.ENDPOINTS.ROBOT_COMMAND(robotId),
    command
  );
};

/**
 * One-way: disables the robot's gRPC command channel remotely. Not
 * remotely reversible, since disabling it closes the only channel a
 * remote re-enable command would need to travel over.
 */
export const disableCommandChannel = (robotId: string): Promise<RobotCommandResponse> => {
  const command: SendRobotCommandData['body'] = { command_type: 'DISABLE_COMMAND_CHANNEL' };
  return postJson<SendRobotCommandResponses['202']>(
    API_CONFIG.ENDPOINTS.ROBOT_COMMAND(robotId),
    command
  );
};
