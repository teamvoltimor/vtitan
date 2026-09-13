import { useEffect, useRef } from 'react';
import { UI_STRINGS } from '../../config';
import { useTelemetry } from '../../contexts/telemetryState';

/**
 * Renders the accumulated log buffer (see TelemetryContext's `logs`), not
 * just the latest snapshot's log lines — those are replaced wholesale every
 * update, so anything logged between two snapshots used to flash and vanish
 * instead of persisting in this "feed".
 */
export function LogPanel() {
  const { logs } = useTelemetry();
  const listRef = useRef<HTMLUListElement>(null);

  // biome-ignore lint/correctness/useExhaustiveDependencies: logs.length is an intentional re-run trigger, not read inside the effect
  useEffect(() => {
    listRef.current?.scrollTo({ top: listRef.current.scrollHeight });
  }, [logs.length]);

  if (logs.length === 0) return null;

  return (
    <div className="log-panel">
      <div className="log-header">
        <p>{UI_STRINGS.EVENT_FEED}</p>
        <span>{UI_STRINGS.NODE_BRIDGE}</span>
      </div>
      <ul ref={listRef}>
        {logs.map((log, i) => (
          // biome-ignore lint/suspicious/noArrayIndexKey: log entries are plain strings with no stable id
          <li key={i}>{log}</li>
        ))}
      </ul>
    </div>
  );
}
