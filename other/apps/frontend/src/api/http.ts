/**
 * src/api/http.ts
 *
 * Shared HTTP client for the Go backend — the single fetch wrapper for the
 * whole frontend. api/telemetry.ts and api/robot.ts previously carried their
 * own resolveUrl + fetch/post pairs with identical error classification and
 * timeout handling (audit §10.2); both route groups are served by the same
 * gin.Engine, so one client, one base URL, one timeout, and one error path.
 *
 * Non-2xx responses are parsed as RFC 7807 Problem Details (internal/problem
 * in the backend) so the server's title/detail reach the caller instead of
 * degrading to a bare "HTTP 500: Internal Server Error" (audit §7.3).
 */

import { API_CONFIG, URL_PROTOCOL_MAP } from '../config';
import { getErrorMessage } from '../utils/formatting';
import { classifyHttpError, TelemetryError } from './errors';
import { schemas } from './schemas';

/** Prepend the configured base URL (empty base → same-origin path). */
export const resolveUrl = (path: string): string => {
  const baseUrl = API_CONFIG.BASE_URL;
  return baseUrl ? `${baseUrl}${path}` : path;
};

/** Convert an http(s) URL to its WebSocket counterpart. */
export const httpToWsUrl = (url: string): string =>
  Object.entries(URL_PROTOCOL_MAP).reduce(
    (acc, [httpProtocol, wsProtocol]) => acc.replace(httpProtocol, wsProtocol),
    url
  );

interface JsonRequestOptions<T> {
  method?: 'GET' | 'POST';
  body?: unknown;
  schema?: { parse: (data: unknown) => T };
  signal?: AbortSignal;
}

/** Read the RFC 7807 `title`/`detail` out of an error response, if present. */
async function extractServerMessage(response: Response): Promise<string | null> {
  try {
    const data: unknown = await response.json();
    const parsed = schemas.ProblemDetails.safeParse(data);
    if (parsed.success) {
      return parsed.data.detail ? `${parsed.data.title}: ${parsed.data.detail}` : parsed.data.title;
    }
  } catch {
    // Non-JSON error body — fall through to the generic status line.
  }
  return null;
}

async function requestJson<T>(path: string, options: JsonRequestOptions<T> = {}): Promise<T> {
  const { method = 'GET', body, schema, signal } = options;

  try {
    const response = await fetch(resolveUrl(path), {
      method,
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
      cache: API_CONFIG.FETCH_CACHE,
      signal: signal ?? AbortSignal.timeout(API_CONFIG.TIMEOUT_MS),
    });

    if (!response.ok) {
      const code = classifyHttpError(response.status);
      const serverMessage = await extractServerMessage(response);
      throw new TelemetryError(
        code,
        serverMessage ?? `HTTP ${response.status}: ${response.statusText}`,
        response.status
      );
    }

    if (response.status === 204) {
      return undefined as T;
    }

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
    if (err instanceof TelemetryError) {
      throw err;
    }
    if (err instanceof Error && err.name === 'AbortError') {
      throw new TelemetryError('TIMEOUT', 'Request timeout', undefined, err);
    }
    throw new TelemetryError('NETWORK', `Network error: ${getErrorMessage(err)}`, undefined, err);
  }
}

/** GET a JSON resource, optionally validating the body against a Zod schema. */
export function fetchJson<T>(path: string, options: JsonRequestOptions<T> = {}): Promise<T> {
  return requestJson<T>(path, options);
}

/** POST a JSON body and parse the JSON response. */
export function postJson<T>(
  path: string,
  body: unknown,
  options: JsonRequestOptions<T> = {}
): Promise<T> {
  return requestJson<T>(path, { ...options, method: 'POST', body });
}
