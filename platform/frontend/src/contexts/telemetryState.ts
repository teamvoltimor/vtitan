/**
 * src/contexts/telemetryState.ts
 *
 * The TelemetryContext object, its type, and the useTelemetry hook. Kept
 * separate from the provider component so the provider file only exports a
 * component (Fast Refresh requirement).
 */

import { createContext, useContext } from 'react';
import type { RobotSnapshot, TopicsSnapshot, ReplaySessionInfo } from '../types';

export interface TelemetryContextType {
  // State
  /** The latest snapshot received from the source, live or not. */
  snapshot: RobotSnapshot | null;
  /** What the UI should actually render: `snapshot` while live, or the
   *  scrubbed `history[timelineIndex]` frame while replaying/paused. */
  displaySnapshot: RobotSnapshot | null;
  topics: TopicsSnapshot | null;
  history: RobotSnapshot[];
  sessions: ReplaySessionInfo[];
  /** Accumulated event-feed log lines (capped), not just the latest snapshot's. */
  logs: string[];
  selectedSessionId: string | null;
  timelineIndex: number;
  liveMode: boolean;
  demoMode: boolean;
  loading: boolean;
  /** Fatal error — no data has ever loaded. Gates the full-screen AsyncState error view. */
  error: string | null;
  /** False while a live connection is down but a last-good snapshot is still on screen. */
  connected: boolean;

  // Actions
  retry: () => void;
  setTimelineIndex: (index: number) => void;
  loadSession: (sessionId: string) => Promise<void>;
  goLive: () => void;
  setDemoMode: (enabled: boolean) => void;
  toggleDemoMode: () => void;
  updateSpeed: (speed: number) => Promise<void>;
}

export const TelemetryContext = createContext<TelemetryContextType | null>(null);

/** Access telemetry state/actions. Must be called within <TelemetryProvider>. */
export function useTelemetry(): TelemetryContextType {
  const context = useContext(TelemetryContext);
  if (!context) {
    throw new Error('useTelemetry must be used within <TelemetryProvider>');
  }
  return context;
}
