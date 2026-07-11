/**
 * src/contexts/TelemetryContext.tsx
 *
 * Global telemetry state. Data access goes through a TelemetrySource port
 * (LiveSource or MockSource), so this context contains no demo-vs-live
 * branching — it selects a source from `demoMode` and calls it uniformly.
 */

import { useState, useCallback, useEffect, useMemo, useRef, type ReactNode } from 'react';
import type { RobotSnapshot, TopicsSnapshot, ReplaySessionInfo } from '../types';
import { isRobotSnapshot, isTopicsSnapshot } from '../api/guards';
import { createTelemetrySource } from '../api/source';
import { getErrorMessage } from '../utils/formatting';
import { TELEMETRY_CONFIG } from '../config';
import { TelemetryContext, type TelemetryContextType } from './telemetryState';

/**
 * Demo Mode on first load: `?demo`/`?demo=true` query param or `VITE_DEMO=true`
 * env var. `?demo=false` forces it off.
 */
function initialDemoMode(): boolean {
  if (typeof window !== 'undefined') {
    const params = new URLSearchParams(window.location.search);
    if (params.has('demo')) {
      return params.get('demo') !== 'false';
    }
  }
  return import.meta.env.VITE_DEMO === 'true';
}

export function TelemetryProvider({ children }: { children: ReactNode }) {
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null);
  const [topics, setTopics] = useState<TopicsSnapshot | null>(null);
  const [history, setHistory] = useState<RobotSnapshot[]>([]);
  const [sessions, setSessions] = useState<ReplaySessionInfo[]>([]);

  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null);
  const [timelineIndex, setTimelineIndex] = useState(0);
  const [liveMode, setLiveMode] = useState(true);
  const [demoMode, setDemoModeState] = useState(initialDemoMode);

  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [retryCount, setRetryCount] = useState(0);

  const mounted = useRef(true);

  // Select the data source for the current mode. Recreated when demoMode flips.
  const source = useMemo(() => createTelemetrySource(demoMode), [demoMode]);

  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);

  // Append a snapshot to history, capped at HISTORY_MAX_SIZE.
  const pushHistory = useCallback((snap: RobotSnapshot) => {
    setHistory((prev) => {
      const updated = [...prev, snap];
      return updated.length > TELEMETRY_CONFIG.HISTORY_MAX_SIZE
        ? updated.slice(-TELEMETRY_CONFIG.HISTORY_MAX_SIZE)
        : updated;
    });
  }, []);

  // Initial load (and on retry / source change).
  const loadInitialData = useCallback(async () => {
    if (!mounted.current) return;
    setLoading(true);
    setError(null);

    try {
      const [latest, rawTopics, historyData, sessionsList] = await Promise.all([
        source.fetchLatest(),
        source.fetchTopics(),
        source.fetchHistory(),
        source.fetchSessions(),
      ]);
      if (!mounted.current) return;

      setSnapshot(latest);
      setTopics(rawTopics);
      setHistory(historyData);
      setSessions(sessionsList);
      setSelectedSessionId(null);
      setTimelineIndex(Math.max(historyData.length - 1, 0));
    } catch (err) {
      if (!mounted.current) return;
      setError(getErrorMessage(err, 'Failed to load telemetry data'));
    } finally {
      if (mounted.current) setLoading(false);
    }
  }, [source]);

  const retry = useCallback(() => setRetryCount((c) => c + 1), []);

  // biome-ignore lint/correctness/useExhaustiveDependencies: retryCount is an intentional re-run trigger, not read inside the effect
  useEffect(() => {
    loadInitialData();
  }, [retryCount, loadInitialData]);

  const loadSession = useCallback(
    async (sessionId: string) => {
      if (!mounted.current) return;
      setError(null);
      try {
        const sessionSnapshots = await source.fetchSession(sessionId);
        if (!mounted.current) return;

        setHistory(sessionSnapshots);
        setTimelineIndex(Math.max(sessionSnapshots.length - 1, 0));
        setSelectedSessionId(sessionId);
        setLiveMode(false);
        if (sessionSnapshots.length > 0) {
          setSnapshot(sessionSnapshots[sessionSnapshots.length - 1]);
        }
      } catch (err) {
        if (!mounted.current) return;
        setError(getErrorMessage(err, `Failed to load session ${sessionId}`));
      }
    },
    [source]
  );

  const goLive = useCallback(() => {
    setSelectedSessionId(null);
    setTimelineIndex(0);
    setError(null);
    setLiveMode(true);
  }, []);

  const setDemoMode = useCallback((enabled: boolean) => {
    setSelectedSessionId(null);
    setTimelineIndex(0);
    setError(null);
    setLiveMode(true);
    setDemoModeState(enabled);
  }, []);

  const toggleDemoMode = useCallback(() => setDemoMode(!demoMode), [demoMode, setDemoMode]);

  const updateSpeed = useCallback(
    (speed: number) => {
      source.updateSpeed(speed).catch((err) => {
        console.error('Failed to update robot speed:', err);
      });
    },
    [source]
  );

  // Live stream subscription (WebSocket for live, timer for demo).
  useEffect(() => {
    if (!liveMode || !mounted.current) return;

    const unsubscribe = source.connect(
      (msg) => {
        if (!mounted.current) return;
        if (isRobotSnapshot(msg)) {
          setSnapshot(msg);
          pushHistory(msg);
          setTimelineIndex((prev) => prev + 1);
        } else if (isTopicsSnapshot(msg)) {
          setTopics(msg);
        }
      },
      (err) => {
        if (!mounted.current) return;
        setError(getErrorMessage(err, 'Live telemetry connection error'));
      }
    );

    return unsubscribe;
  }, [liveMode, source, pushHistory]);

  const value: TelemetryContextType = {
    snapshot,
    topics,
    history,
    sessions,
    selectedSessionId,
    timelineIndex,
    liveMode,
    demoMode,
    loading,
    error,
    retry,
    setTimelineIndex,
    loadSession,
    goLive,
    setDemoMode,
    toggleDemoMode,
    updateSpeed,
  };

  return <TelemetryContext.Provider value={value}>{children}</TelemetryContext.Provider>;
}
