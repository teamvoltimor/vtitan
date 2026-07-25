import { useTelemetry } from '../../contexts/telemetryState';
import { UI_STRINGS } from '../../config';

export function LogPanel() {
  const { snapshot } = useTelemetry();
  if (!snapshot) return null;

  return (
    <div className="log-panel">
      <div className="log-header">
        <p>{UI_STRINGS.EVENT_FEED}</p>
        <span>{UI_STRINGS.NODE_BRIDGE}</span>
      </div>
      <ul>
        {snapshot.logs.map((log, i) => (
          // biome-ignore lint/suspicious/noArrayIndexKey: log entries are plain strings with no stable id
          <li key={i}>{log}</li>
        ))}
      </ul>
    </div>
  );
}
