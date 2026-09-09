import { useCallback, useState } from 'react';
import type { TimelineEvent } from './appState.types';

/**
 * Owns the activity log, timeline, and gallery stats — the append-only
 * "what just happened" state that the rest of the app records into via
 * `recordAction`.
 */
export const useActivityLog = () => {
  const [logEntries, setLogEntries] = useState<string[]>(['Ready.']);
  const [stats, setStats] = useState({ processed: '0', skipped: '0', labels: '0' });
  const [timeline, setTimeline] = useState<TimelineEvent[]>([]);

  const pushLog = useCallback((entry: string) => {
    setLogEntries((prev) => [entry, ...prev].slice(0, 5));
  }, []);

  const pushTimeline = useCallback((event: TimelineEvent) => {
    setTimeline((prev) => [event, ...prev].slice(0, 6));
  }, []);

  const recordAction = useCallback(
    (label: string) => {
      const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
      pushLog(`${time} · ${label}`);
      pushTimeline({ time, label });
    },
    [pushLog, pushTimeline]
  );

  return { logEntries, pushLog, stats, setStats, timeline, pushTimeline, recordAction };
};
