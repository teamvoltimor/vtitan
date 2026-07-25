import { useTelemetry } from '../../contexts/telemetryState';
import { formatTimestamp } from '../../utils/formatting';
import { UI_STRINGS } from '../../config';
import { Label } from '../ui';

export function SessionList() {
  const { sessions, selectedSessionId, loadSession } = useTelemetry();

  if (sessions.length === 0) return null;

  return (
    <div className="session-list">
      <Label>{UI_STRINGS.REPLAYS}</Label>
      {sessions.map((session) => {
        const isSelected = session.session_id === selectedSessionId;
        return (
          <button
            key={session.session_id}
            type="button"
            className={`session-item${isSelected ? ' selected' : ''}`}
            onClick={() => loadSession(session.session_id)}
          >
            <span>{session.session_id}</span>
            <small>{formatTimestamp(session.created_at, 'full')}</small>
            <strong>{session.entry_count} snaps</strong>
          </button>
        );
      })}
    </div>
  );
}
