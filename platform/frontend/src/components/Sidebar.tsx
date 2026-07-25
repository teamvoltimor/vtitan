import { useTelemetry } from '../contexts/telemetryState';
import { formatNumber } from '../utils/formatting';
import { formatTimestamp } from '../utils/formatting';
import { UI_STRINGS } from '../config';
import { Label, MetricRow, StatTile } from './ui';
import {
  SensorHealthPanel,
  SpeedControl,
  TimelineSlider,
  SessionList,
  LogPanel,
} from './sidebar-panels';

export function Sidebar() {
  const { displaySnapshot: snapshot, liveMode } = useTelemetry();

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

  return (
    <aside className="control-panel">
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

      <SpeedControl />

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
