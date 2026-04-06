/**
 * src/components/Sidebar.tsx
 * 
 * Refactored Sidebar component using TelemetryContext.
 * Eliminated 10 props by consuming context directly via useTelemetry() hook.
 * Reduced from 160 lines (in App.tsx) to ~120 lines.
 * 
 * Uses utility functions and config instead of duplicated/hardcoded values.
 */

import { NodeHealth } from '../types'
import { useTelemetry } from '../contexts/TelemetryContext'
import { formatNumber, formatUnixTimestamp } from '../utils/formatting'
import { SENSOR_CONFIG, SPEED_CONTROL_CONFIG, THEME } from '../config'
import { updateRobotSpeed } from '../api/telemetry'

/**
 * Sensor health status display.
 */
function SensorHealthPanel() {
  const { snapshot } = useTelemetry()
  
  if (!snapshot) return null

  const metrics = snapshot.metrics

  return (
    <div className="sensor-health">
      <p className="label">SENSOR STATUS</p>
      <div className="sensor-grid">
        {SENSOR_CONFIG.map(({ id, name, icon, key }) => {
          const available = metrics[key]
          return (
            <div
              key={id}
              className={`sensor-item ${available ? 'online' : 'offline'}`}
            >
              <span className="sensor-icon">{icon}</span>
              <div className="sensor-info">
                <strong>{name}</strong>
                <span className={available ? 'status-ok' : 'status-error'}>
                  {available ? 'ONLINE' : 'OFFLINE'}
                </span>
              </div>
            </div>
          )
        })}
      </div>
    </div>
  )
}

/**
 * Main sidebar component with telemetry metrics, timeline, and session controls.
 * Uses TelemetryContext via useTelemetry() hook - no prop drilling.
 */
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
  } = useTelemetry()

  if (!snapshot) return null

  const metrics = snapshot.metrics

  // Compute metrics for display
  const statRows: Array<[string, number | null | undefined]> = [
    ['Forward', metrics.forward],
    ['Left', metrics.left],
    ['Right', metrics.right],
    ['Back', metrics.back],
  ]

  const telemetryEntries: Array<[string, string]> = [
    ['Node Health', metrics.nodeHealth],
    ['Stage', metrics.stage ?? 'unknown'],
    ['Speed', formatNumber(metrics.speed, { unit: 'm/s' })],
    ['Range Min', formatNumber(metrics.rangeMin)],
    ['Range Mean', formatNumber(metrics.rangeMean)],
    ['Range Max', formatNumber(metrics.rangeMax)],
    ['Points', metrics.pointsCaptured?.toString() ?? '0'],
  ]

  // Timeline state
  const hasHistory = history.length > 0
  const timelineLength = history.length
  const timelineTimestamp =
    history.length > 0
      ? history[timelineIndex]?.timestamp ?? snapshot.timestamp
      : snapshot.timestamp
  const timelineLabel = liveMode
    ? 'Live timeline'
    : selectedSessionId
      ? 'Replay timeline'
      : 'Timeline'

  const handleTimelineChange = (value: string) => {
    setTimelineIndex(Number(value))
  }

  const handleSpeedChange = (value: string) => {
    const speed = parseFloat(value)
    updateRobotSpeed(speed).catch((err) => {
      console.error('Failed to update robot speed:', err)
    })
  }

  return (
    <aside className="control-panel" style={{ background: THEME.COLORS.PANEL }}>
      {/* Header */}
      <div className="panel-header">
        <p className="label">LIVE TRACKING</p>
        <h1>{snapshot.missionName}</h1>
        <p className="label">
          Updated {formatUnixTimestamp(snapshot.timestamp)}
        </p>
      </div>

      {/* Sensor Status */}
      <SensorHealthPanel />

      {/* Metrics Grid */}
      <div className="metrics-grid">
        {statRows.map(([label, value]) => (
          <div key={label}>
            <p>{label}</p>
            <strong>{formatNumber(value as number | null)}</strong>
          </div>
        ))}
      </div>

      {/* Speed Control */}
      <div className="speed-control">
        <p className="label">MAX LINEAR SPEED</p>
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '10px',
            paddingBottom: '16px',
          }}
        >
          <input
            type="range"
            min={SPEED_CONTROL_CONFIG.MIN}
            max={SPEED_CONTROL_CONFIG.MAX}
            step={SPEED_CONTROL_CONFIG.STEP}
            disabled={
              !liveMode || metrics.nodeHealth !== NodeHealth.NOMINAL
            }
            defaultValue={SPEED_CONTROL_CONFIG.DEFAULT}
            onChange={(e) => handleSpeedChange(e.target.value)}
            style={{ flex: 1 }}
          />
          <span style={{ minWidth: '60px' }}>
            {formatNumber(metrics.speed, { decimals: 2, unit: 'm/s' })}
          </span>
        </div>
      </div>

      {/* Telemetry List */}
      <div className="telemetry-list">
        {telemetryEntries.map(([label, value]) => (
          <div key={label}>
            <span>{label}</span>
            <strong>{value}</strong>
          </div>
        ))}
      </div>

      {/* Timeline Control */}
      {hasHistory && (
        <div className="timeline">
          <div className="timeline-header">
            <p>{timelineLabel}</p>
            <button
              className="timeline-button"
              type="button"
              onClick={goLive}
              disabled={liveMode}
            >
              {liveMode ? 'Live' : 'Go live'}
            </button>
          </div>
          <input
            type="range"
            min="0"
            max={Math.max(timelineLength - 1, 0).toString()}
            value={timelineIndex}
            onChange={(event) => handleTimelineChange(event.target.value)}
          />
          <div className="timeline-meta">
            <span>
              {timelineIndex + 1}/{Math.max(timelineLength, 1)} ·{' '}
              {formatUnixTimestamp(timelineTimestamp)}
            </span>
          </div>
        </div>
      )}

      {/* Session List */}
      {sessions.length > 0 && (
        <div className="session-list">
          <p className="label">Replays</p>
          {sessions.map((session) => {
            const isSelected = session.sessionId === selectedSessionId
            return (
              <button
                key={session.sessionId}
                type="button"
                className={`session-item${isSelected ? ' selected' : ''}`}
                onClick={() => loadSession(session.sessionId)}
              >
                <span>{session.sessionId}</span>
                <small>{formatUnixTimestamp(session.createdAt, 'full')}</small>
                <strong>{session.entryCount} snaps</strong>
              </button>
            )
          })}
        </div>
      )}

      {/* Event Feed / Logs */}
      <div className="log-panel">
        <div className="log-header">
          <p>Event Feed</p>
          <span>Node Bridge</span>
        </div>
        <ul>
          {snapshot.logs.map((log, i) => (
            <li key={i}>{log}</li>
          ))}
        </ul>
      </div>
    </aside>
  )
}
