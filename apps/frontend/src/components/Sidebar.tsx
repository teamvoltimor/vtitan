import { UI_STRINGS } from '../config';
import { useTelemetry } from '../contexts/telemetryState';
import { formatNumber, formatTimestamp } from '../utils/formatting';
import {
  ChannelsGroup,
  LogPanel,
  SensorHealthPanel,
  SessionList,
  SpeedControl,
  TimelineSlider,
} from './sidebar-panels';
import { Label, MetricRow, StatTile } from './ui';

export function Sidebar() {
  const { displaySnapshot: snapshot, liveMode, goLive } = useTelemetry();

  if (!snapshot) return null;

  const metrics = snapshot.metrics;

  const statRows: Array<[string, number | null | undefined]> = [
    ['Forward', metrics.forward],
    ['Left', metrics.left],
    ['Right', metrics.right],
    ['Back', metrics.back],
  ];

  // Range min/mean/max collapsed to a single row — the radar chart already
  // visualises the same distances, so three list rows were redundant
  // (audit §3.8).
  const rangeRow: [string, string] | null =
    metrics.range_min !== null &&
    metrics.range_min !== undefined &&
    metrics.range_mean !== null &&
    metrics.range_mean !== undefined &&
    metrics.range_max !== null &&
    metrics.range_max !== undefined
      ? [
          'Range',
          `${formatNumber(metrics.range_min)} · ${formatNumber(metrics.range_mean)} · ${formatNumber(metrics.range_max)} m`,
        ]
      : null;

  const telemetryEntries: Array<[string, string]> = [
    ['Node Health', metrics.node_health],
    ['Stage', metrics.stage ?? 'unknown'],
    ['Speed', formatNumber(metrics.speed, { unit: 'm/s' })],
    ['Points', metrics.points_captured?.toString() ?? '0'],
  ];
  if (rangeRow) telemetryEntries.push(rangeRow);

  // Drive state from the live snapshot (audit §3.6): steering angle is a
  // heading delta in radians, surfaced as degrees for operator legibility.
  const motorState = snapshot.motor_state;
  if (motorState) {
    telemetryEntries.push(
      ['Steering', `${formatNumber(motorState.steering_angle * (180 / Math.PI))}°`],
      ['Drive', formatNumber(motorState.drive_speed, { unit: 'm/s' })]
    );
  }

  return (
    <aside className="control-panel">
      <div className="panel-header">
        <img className="panel-logo" src="/voltimor-logo-square.png" alt="Voltimor" />
        <button
          type="button"
          className={`live-switch${liveMode ? ' is-live' : ' is-replay'}`}
          onClick={goLive}
          disabled={liveMode}
          aria-pressed={liveMode}
        >
          {liveMode ? UI_STRINGS.LIVE_TRACKING : 'REPLAY — return to live'}
        </button>
        <h1>{snapshot.mission_name}</h1>
        <Label>Updated {formatTimestamp(snapshot.timestamp)}</Label>
      </div>

      <SensorHealthPanel />

      <div className="metrics-grid">
        {statRows.map(([label, value]) => (
          <StatTile key={label} label={label} value={formatNumber(value)} />
        ))}
      </div>

      <SpeedControl />
      <ChannelsGroup />

      <div className="telemetry-list">
        {telemetryEntries.map(([label, value]) => (
          <MetricRow key={label} label={label} value={value} />
        ))}
      </div>

      <TimelineSlider />
      <SessionList />
      <LogPanel />
    </aside>
  );
}
