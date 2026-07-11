/**
 * src/components/Sidebar.tsx
 *
 * Telemetry control panel: sensor health, metrics, speed control, timeline,
 * replay sessions and the event feed. Consumes TelemetryContext directly.
 */

import { NodeHealth } from '../types';
import { useTelemetry } from '../contexts/telemetryState';
import { formatNumber, formatTimestamp } from '../utils/formatting';
import { SENSOR_CONFIG, SPEED_CONTROL_CONFIG, UI_STRINGS, THEME } from '../config';
import { Label, MetricRow, StatTile } from './ui';

/** Sensor health status grid. */
function SensorHealthPanel() {
  const { snapshot } = useTelemetry();
  if (!snapshot) return null;
  const metrics = snapshot.metrics;

  return (
    <div className="sensor-health">
      <Label>{UI_STRINGS.SENSOR_STATUS}</Label>
      <div className="sensor-grid">
        {SENSOR_CONFIG.map(({ id, name, key }) => {
          const available = metrics[key];
          return (
            <div key={id} className={`sensor-item ${available ? 'online' : 'offline'}`}>
              <span className="sensor-dot" />
              <div className="sensor-info">
                <strong>{name}</strong>
                <span className={available ? 'status-ok' : 'status-error'}>
                  {available ? 'ONLINE' : 'OFFLINE'}
                </span>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}

export function Sidebar() {
  const {
    snapshot,
    history,
    timelineIndex,
    liveMode,
    sessions,
    selectedSessionId,
    setTimelineIndex,
    goLive,
    loadSession,
    updateSpeed,
  } = useTelemetry();

  if (!snapshot) return null;

  const metrics = snapshot.metrics;

  const statRows: Array<[string, number | null | undefined]> = [
    ['Forward', metrics.forward],
    ['Left', metrics.left],
    ['Right', metrics.right],
    ['Back', metrics.back],
  ];

  const telemetryEntries: Array<[string, string]> = [
    ['Node Health', metrics.node_health],
    ['Stage', metrics.stage ?? 'unknown'],
    ['Speed', formatNumber(metrics.speed, { unit: 'm/s' })],
    ['Range Min', formatNumber(metrics.range_min)],
    ['Range Mean', formatNumber(metrics.range_mean)],
    ['Range Max', formatNumber(metrics.range_max)],
    ['Points', metrics.points_captured?.toString() ?? '0'],
  ];

  const hasHistory = history.length > 0;
  const timelineLength = history.length;
  const timelineTimestamp =
    history.length > 0
      ? (history[timelineIndex]?.timestamp ?? snapshot.timestamp)
      : snapshot.timestamp;
  const timelineLabel = liveMode
    ? 'Live timeline'
    : selectedSessionId
      ? 'Replay timeline'
      : 'Timeline';

  const handleSpeedChange = (value: string) => {
    updateSpeed(parseFloat(value));
  };

  return (
    <aside className="control-panel" style={{ background: THEME.COLORS.PANEL }}>
      <div className="panel-header">
        <Label>{liveMode ? UI_STRINGS.LIVE_TRACKING : 'REPLAY'}</Label>
        <h1>{snapshot.mission_name}</h1>
        <Label>Updated {formatTimestamp(snapshot.timestamp)}</Label>
      </div>

      <SensorHealthPanel />

      <div className="metrics-grid">
        {statRows.map(([label, value]) => (
          <StatTile key={label} label={label} value={formatNumber(value)} />
        ))}
      </div>

      <div className="speed-control">
        <Label>{UI_STRINGS.SPEED_CONTROL}</Label>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px', paddingBottom: '16px' }}>
          <input
            type="range"
            aria-label={UI_STRINGS.SPEED_CONTROL}
            min={SPEED_CONTROL_CONFIG.MIN}
            max={SPEED_CONTROL_CONFIG.MAX}
            step={SPEED_CONTROL_CONFIG.STEP}
            disabled={!liveMode || metrics.node_health !== NodeHealth.NOMINAL}
            defaultValue={SPEED_CONTROL_CONFIG.DEFAULT}
            onChange={(e) => handleSpeedChange(e.target.value)}
            style={{ flex: 1 }}
          />
          <span style={{ minWidth: '60px' }}>
            {formatNumber(metrics.speed, { decimals: 2, unit: 'm/s' })}
          </span>
        </div>
      </div>

      <div className="telemetry-list">
        {telemetryEntries.map(([label, value]) => (
          <MetricRow key={label} label={label} value={value} />
        ))}
      </div>

      {hasHistory && (
        <div className="timeline">
          <div className="timeline-header">
            <p>{timelineLabel}</p>
            <button className="timeline-button" type="button" onClick={goLive} disabled={liveMode}>
              {liveMode ? UI_STRINGS.LIVE : UI_STRINGS.GO_LIVE}
            </button>
          </div>
          <input
            type="range"
            aria-label="Timeline position"
            min="0"
            max={Math.max(timelineLength - 1, 0).toString()}
            value={timelineIndex}
            onChange={(event) => setTimelineIndex(Number(event.target.value))}
          />
          <div className="timeline-meta">
            <span>
              {timelineIndex + 1}/{Math.max(timelineLength, 1)} ·{' '}
              {formatTimestamp(timelineTimestamp)}
            </span>
          </div>
        </div>
      )}

      {sessions.length > 0 && (
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
      )}

      <div className="log-panel">
        <div className="log-header">
          <p>{UI_STRINGS.EVENT_FEED}</p>
          <span>{UI_STRINGS.NODE_BRIDGE}</span>
        </div>
        <ul>
          {snapshot.logs.map((log, i) => (
            // biome-ignore lint/suspicious/noArrayIndexKey: logs is fully replaced each snapshot, has no stable id, and entries may repeat
            <li key={i}>{log}</li>
          ))}
        </ul>
      </div>
    </aside>
  );
}
