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
  snapshot: RobotSnapshot | null;
  topics: TopicsSnapshot | null;
  history: RobotSnapshot[];
  sessions: ReplaySessionInfo[];
  selectedSessionId: string | null;
  timelineIndex: number;
  liveMode: boolean;
  demoMode: boolean;
  loading: boolean;
  error: string | null;

  // Actions
  retry: () => void;
  setTimelineIndex: (index: number) => void;
  loadSession: (sessionId: string) => Promise<void>;
  goLive: () => void;
  setDemoMode: (enabled: boolean) => void;
  toggleDemoMode: () => void;
  updateSpeed: (speed: number) => void;
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
