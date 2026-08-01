/**
 * src/contexts/TelemetryContext.tsx
 *
 * Global telemetry state. Data access goes through a TelemetrySource port
 * (LiveSource or MockSource), so this context contains no demo-vs-live
 * branching — it selects a source from `demoMode` and calls it uniformly.
 */

import { useState, useCallback, useEffect, useMemo, useRef, type ReactNode } from 'react';
import type { RobotSnapshot, TopicsSnapshot, ReplaySessionInfo } from '../types';
import type { SetTelemetryChannelParams, SetVisionDebugParams } from '../api/generated/robot';
import { isRobotSnapshot, isTopicsSnapshot } from '../api/guards';
import { createTelemetrySource } from '../api/source';
import { getErrorMessage } from '../utils/formatting';
import { TELEMETRY_CONFIG, API_CONFIG } from '../config';
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

/** Append `logs` to a buffer, skipping an exact repeat of the last entry, capped at LOG_BUFFER_MAX_SIZE. */
function appendLogs(prev: string[], logs: string[]): string[] {
  if (!logs || logs.length === 0) return prev;
  const lastExisting = prev[prev.length - 1];
  const incoming = logs.filter((line, i) => !(i === 0 && line === lastExisting));
  if (incoming.length === 0) return prev;
  const updated = [...prev, ...incoming];
  return updated.length > TELEMETRY_CONFIG.LOG_BUFFER_MAX_SIZE
    ? updated.slice(-TELEMETRY_CONFIG.LOG_BUFFER_MAX_SIZE)
    : updated;
}

export function TelemetryProvider({ children }: { children: ReactNode }) {
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null);
  const [topics, setTopics] = useState<TopicsSnapshot | null>(null);
  const [history, setHistory] = useState<RobotSnapshot[]>([]);
  const [sessions, setSessions] = useState<ReplaySessionInfo[]>([]);
  const [logs, setLogs] = useState<string[]>([]);

  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null);
  const [timelineIndex, setTimelineIndex] = useState(0);
  const [liveMode, setLiveMode] = useState(true);
  const [demoMode, setDemoModeState] = useState(initialDemoMode);

  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  // Two independent connectivity signals, combined into the exposed
  // `connected` value: the WebSocket pipe (fast, but a wedged backend can
  // hold a socket open with no onerror/onclose) and a periodic health poll
  // (catches that case, and generally any request-serving failure).
  const [wsConnected, setWsConnected] = useState(true);
  const [healthOk, setHealthOk] = useState(true);
  const [retryCount, setRetryCount] = useState(0);

  const mounted = useRef(true);
  // Mirrors `snapshot` for use inside the live-subscription effect's callbacks
  // without adding `snapshot` to that effect's dependency array (which would
  // tear down and reopen the WebSocket on every incoming frame).
  const snapshotRef = useRef<RobotSnapshot | null>(null);

  // Select the data source for the current mode. Recreated when demoMode flips.
  const source = useMemo(() => createTelemetrySource(demoMode), [demoMode]);

  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);

  useEffect(() => {
    snapshotRef.current = snapshot;
  }, [snapshot]);

  // Append a snapshot to history, capped at HISTORY_MAX_SIZE.
  const pushHistory = useCallback((snap: RobotSnapshot) => {
    setHistory((prev) => {
      const updated = [...prev, snap];
      return updated.length > TELEMETRY_CONFIG.HISTORY_MAX_SIZE
        ? updated.slice(-TELEMETRY_CONFIG.HISTORY_MAX_SIZE)
        : updated;
    });
  }, []);

  const pushLogs = useCallback((snap: RobotSnapshot) => {
    setLogs((prev) => appendLogs(prev, snap.logs));
  }, []);

  // While live, the timeline should always track the newest frame — clamped
  // to the actual (capped) history length rather than incremented without
  // bound, which previously let the counter run far past `history.length`.
  useEffect(() => {
    if (liveMode) {
      setTimelineIndex(history.length > 0 ? history.length - 1 : 0);
    }
  }, [history, liveMode]);

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
      setLogs(
        historyData.flatMap((snap) => snap.logs ?? []).slice(-TELEMETRY_CONFIG.LOG_BUFFER_MAX_SIZE)
      );
      setSelectedSessionId(null);
      setTimelineIndex(Math.max(historyData.length - 1, 0));
      setWsConnected(true);
      setHealthOk(true);
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
        setLogs(
          sessionSnapshots
            .flatMap((snap) => snap.logs ?? [])
            .slice(-TELEMETRY_CONFIG.LOG_BUFFER_MAX_SIZE)
        );
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
    (speed: number): Promise<void> =>
      source.updateSpeed(speed).catch((err) => {
        console.error('Failed to update robot speed:', err);
        throw err;
      }),
    [source]
  );

  const setVisionDebug = useCallback(
    (parameters: SetVisionDebugParams): Promise<void> =>
      source.setVisionDebug(parameters).catch((err) => {
        console.error('Failed to update vision debug stream:', err);
        throw err;
      }),
    [source]
  );

  const setTelemetryChannel = useCallback(
    (parameters: SetTelemetryChannelParams): Promise<void> =>
      source.setTelemetryChannel(parameters).catch((err) => {
        console.error('Failed to update telemetry channel:', err);
        throw err;
      }),
    [source]
  );

  const disableCommandChannel = useCallback(
    (): Promise<void> =>
      source.disableCommandChannel().catch((err) => {
        console.error('Failed to disable command channel:', err);
        throw err;
      }),
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
          pushLogs(msg);
        } else if (isTopicsSnapshot(msg)) {
          setTopics(msg);
        }
      },
      (err) => {
        if (!mounted.current) return;
        setWsConnected(false);
        // Only fatal (blocks the whole dashboard behind AsyncState) if we've
        // never successfully loaded any data. Once there's a last-good
        // snapshot on screen, a dropped connection is transient — surfaced
        // via `connected` for a small reconnecting indicator instead.
        if (!snapshotRef.current) {
          setError(getErrorMessage(err, 'Live telemetry connection error'));
        }
      },
      () => {
        if (!mounted.current) return;
        setWsConnected(true);
      }
    );

    return unsubscribe;
  }, [liveMode, source, pushHistory, pushLogs]);

  // Periodic health poll — independent of the WebSocket callbacks above, so a
  // backend that's wedged (socket stays open, but stops actually serving
  // requests) still gets caught instead of showing a false "connected" state.
  useEffect(() => {
    if (!liveMode) return;

    let cancelled = false;
    const checkHealth = () => {
      source
        .fetchHealth()
        .then((health) => {
          if (cancelled || !mounted.current) return;
          setHealthOk(health.status === 'ok');
        })
        .catch(() => {
          if (cancelled || !mounted.current) return;
          setHealthOk(false);
        });
    };

    checkHealth();
    const intervalId = window.setInterval(checkHealth, API_CONFIG.HEALTH_CHECK_INTERVAL_MS);
    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [liveMode, source]);

  const connected = wsConnected && healthOk;

  // displaySnapshot is what the UI should render: the live snapshot while
  // live, or the scrubbed history frame while replaying/paused on the
  // timeline. Previously the scene always rendered the live `snapshot`
  // regardless of `timelineIndex`, so dragging the timeline changed only the
  // label, not the 3D view or sidebar.
  const displaySnapshot = liveMode ? snapshot : (history[timelineIndex] ?? snapshot);

  const value: TelemetryContextType = useMemo(
    () => ({
      snapshot,
      displaySnapshot,
      topics,
      history,
      sessions,
      logs,
      selectedSessionId,
      timelineIndex,
      liveMode,
      demoMode,
      loading,
      error,
      connected,
      retry,
      setTimelineIndex,
      loadSession,
      goLive,
      setDemoMode,
      toggleDemoMode,
      updateSpeed,
      setVisionDebug,
      setTelemetryChannel,
      disableCommandChannel,
    }),
    [
      snapshot,
      displaySnapshot,
      topics,
      history,
      sessions,
      logs,
      selectedSessionId,
      timelineIndex,
      liveMode,
      demoMode,
      loading,
      error,
      connected,
      retry,
      loadSession,
      goLive,
      setDemoMode,
      toggleDemoMode,
      updateSpeed,
      setVisionDebug,
      setTelemetryChannel,
      disableCommandChannel,
    ]
  );

  return <TelemetryContext.Provider value={value}>{children}</TelemetryContext.Provider>;
}
