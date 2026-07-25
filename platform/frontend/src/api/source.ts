/**
 * src/api/source.ts
 *
 * TelemetrySource port: a single interface the rest of the app depends on for
 * telemetry, with two adapters — LiveSource (real backend over HTTP + WebSocket)
 * and MockSource (locally generated demo data). This keeps the context free of
 * `demoMode` branching: it just selects a source and calls it uniformly.
 */

import type { RobotSnapshot, TopicsSnapshot, ReplaySessionInfo } from '../types';
import type { HealthResponse } from './generated';
import type { TelemetryMessage } from './guards';
import {
  fetchHealth,
  fetchLatestTelemetry,
  fetchRawTopics,
  fetchHistory,
  fetchSessions,
  fetchSession,
  connectTelemetryWS,
  updateRobotSpeed,
} from './telemetry';
import {
  generateMockSnapshot,
  generateMockTopics,
  generateMockSession,
  generateMockSessions,
} from './mockData';
import { TELEMETRY_CONFIG } from '../config';

/** Number of snapshots pre-seeded into history when Demo Mode starts. */
export const DEMO_SEED_FRAMES = 24;

export interface TelemetrySource {
  /** Independent connectivity backstop — see fetchHealth's docstring in telemetry.ts. */
  fetchHealth(): Promise<HealthResponse>;
  fetchLatest(): Promise<RobotSnapshot>;
  fetchTopics(): Promise<TopicsSnapshot>;
  fetchHistory(): Promise<RobotSnapshot[]>;
  fetchSessions(): Promise<ReplaySessionInfo[]>;
  fetchSession(id: string): Promise<RobotSnapshot[]>;
  /**
   * Subscribe to the live stream. `onConnected` fires on (re)connect so
   * callers can distinguish "briefly reconnecting" from "never connected".
   * Returns an unsubscribe function.
   */
  connect(
    onMessage: (msg: TelemetryMessage) => void,
    onError?: (error: Error) => void,
    onConnected?: () => void
  ): () => void;
  updateSpeed(speed: number): Promise<void>;
}

/** Real backend: HTTP fetches + WebSocket stream. */
export class LiveSource implements TelemetrySource {
  fetchHealth = fetchHealth;
  fetchLatest = fetchLatestTelemetry;
  fetchTopics = fetchRawTopics;
  fetchHistory = fetchHistory;
  fetchSessions = fetchSessions;
  fetchSession = fetchSession;
  updateSpeed = updateRobotSpeed;

  connect(
    onMessage: (msg: TelemetryMessage) => void,
    onError?: (error: Error) => void,
    onConnected?: () => void
  ) {
    return connectTelemetryWS(onMessage, onError, onConnected);
  }
}

/** Locally generated demo data; live stream driven by a timer. */
export class MockSource implements TelemetrySource {
  private tick = DEMO_SEED_FRAMES - 1;

  async fetchHealth(): Promise<HealthResponse> {
    return { status: 'ok', version: 'demo' };
  }
  async fetchLatest() {
    return generateMockSnapshot(this.tick);
  }
  async fetchTopics() {
    return generateMockTopics(this.tick);
  }
  async fetchHistory() {
    return Array.from({ length: DEMO_SEED_FRAMES }, (_, i) => generateMockSnapshot(i));
  }
  async fetchSessions() {
    return generateMockSessions();
  }
  async fetchSession(id: string) {
    return generateMockSession(id);
  }
  async updateSpeed() {
    /* no backend in demo mode */
  }

  connect(
    onMessage: (msg: TelemetryMessage) => void,
    _onError?: (error: Error) => void,
    onConnected?: () => void
  ) {
    onConnected?.();
    const intervalId = window.setInterval(() => {
      this.tick += 1;
      onMessage(generateMockSnapshot(this.tick));
      onMessage(generateMockTopics(this.tick));
    }, TELEMETRY_CONFIG.POLL_INTERVAL_MS);
    return () => window.clearInterval(intervalId);
  }
}

/** Factory: pick the source implementation for the current mode. */
export function createTelemetrySource(demoMode: boolean): TelemetrySource {
  return demoMode ? new MockSource() : new LiveSource();
}
