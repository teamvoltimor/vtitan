import { useEffect, useMemo, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera } from '@react-three/drei'
import * as THREE from 'three'
import './App.css'

import { NodeHealth, type Position3D, type ReplaySessionInfo, type RobotSnapshot, type TopicsSnapshot } from './types'
import { fetchLatestTelemetry, fetchHistory, fetchSession, fetchSessions, fetchRawTopics, connectTelemetryWS, updateRobotSpeed } from './api/telemetry'
import { TopicInspector } from './components/TopicInspector'

/**
 * Map simulation coords (origin bottom-left, 0–3 range) to Three.js (XZ floor, Y up).
 * Subtracts the track centre (1.5, 1.5) so the floor mesh centred at origin lines up.
 */
const simToThree = ([x, y, z]: Position3D): [number, number, number] => [x - 1.5, z, -(y - 1.5)]

const colors = {
  background: '#050b12',
  panel: '#0f1c2b',
  accent: '#ff8a65',
  highlight: '#5fdde5',
}

function LiDARPointCloud({ snapshot }: { snapshot: RobotSnapshot }) {
  const positions = useMemo(() => {
    if (!snapshot.metrics.lidarAvailable || snapshot.lidarPoints.length === 0) {
      return new Float32Array(0)
    }
    
    const data = new Float32Array(snapshot.lidarPoints.length * 3)
    snapshot.lidarPoints.forEach((pt, index) => {
      const [x, y, z] = simToThree(pt)
      data[index * 3 + 0] = x
      data[index * 3 + 1] = y
      data[index * 3 + 2] = z
    })
    return data
  }, [snapshot])

  if (positions.length === 0) {
    return (
      <mesh position={[0, 0.5, 0]}>
        <sphereGeometry args={[0.05, 8, 8]} />
        <meshStandardMaterial color="#ff0000" emissive="#ff0000" />
      </mesh>
    )
  }

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial 
        size={0.03} 
        color={colors.highlight} 
        transparent 
        opacity={snapshot.metrics.lidarAvailable ? 0.9 : 0.3} 
        depthTest={false} 
      />
    </points>
  )
}

function RobotPath({ snapshot }: { snapshot: RobotSnapshot }) {
  const curve = useMemo(
    () => {
      if (!snapshot.pathHistory || snapshot.pathHistory.length < 2) return null;
      const pts = snapshot.pathHistory.map((pt) => {
        const [x, y, z] = simToThree(pt)
        return new THREE.Vector3(x, y + 0.03, z)
      })
      return new THREE.CatmullRomCurve3(pts)
    },
    [snapshot]
  )

  const tubeGeometry = useMemo(
    () => (curve ? new THREE.TubeGeometry(curve, 80, 0.01, 5, false) : null),
    [curve]
  )
  useEffect(() => () => tubeGeometry?.dispose(), [tubeGeometry])

  if (!tubeGeometry) return null

  return (
    <mesh geometry={tubeGeometry}>
      <meshStandardMaterial color={colors.accent} emissive={colors.accent} emissiveIntensity={0.6} />
    </mesh>
  )
}

function Robot({ snapshot }: { snapshot: RobotSnapshot }) {
  const position = snapshot.robotPosition ?? [1.5, 1.5, 0.1]
  const orientation = snapshot.robotOrientation ?? 0
  const available = snapshot.metrics.odometryAvailable

  return (
    <mesh 
      position={simToThree(position as Position3D)} 
      rotation={[0, -orientation, 0]}
    >
      <boxGeometry args={[0.2, 0.08, 0.14]} />
      <meshStandardMaterial 
        color={available ? "#f3c677" : "#666666"} 
        emissive={available ? "#e57f2e" : "#333333"}
        opacity={available ? 1.0 : 0.5}
        transparent={!available}
      />
    </mesh>
  )
}

function TrackFloor() {
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[3.2, 3.2]} />
      <meshStandardMaterial color="#111b27" metalness={0.4} roughness={0.7} side={THREE.DoubleSide} />
    </mesh>
  )
}

function SceneCanvas({ snapshot }: { snapshot: RobotSnapshot }) {
  return (
    <Canvas shadows dpr={[1, 2]}>
      <color attach="background" args={[colors.background]} />
      <ambientLight intensity={0.6} />
      <directionalLight intensity={1.1} position={[3, -3, 4]} castShadow />
      <TrackFloor />
      <LiDARPointCloud snapshot={snapshot} />
      <RobotPath snapshot={snapshot} />
      <Robot snapshot={snapshot} />
      <PerspectiveCamera makeDefault position={[0, 5, 3]} fov={45} />
      <OrbitControls enablePan={false} maxPolarAngle={Math.PI / 2.1} />
    </Canvas>
  )
}

function SensorHealthPanel({ metrics }: { metrics: import('./types').TelemetryMetrics }) {
  const sensors = [
    { name: 'LiDAR', available: metrics.lidarAvailable, icon: '📡' },
    { name: 'IMU', available: metrics.imuAvailable, icon: '🧭' },
    { name: 'Camera', available: metrics.cameraAvailable, icon: '📷' },
    { name: 'Odometry', available: metrics.odometryAvailable, icon: '⚙️' },
  ]
  
  return (
    <div className="sensor-health">
      <p className="label">SENSOR STATUS</p>
      <div className="sensor-grid">
        {sensors.map(({ name, available, icon }) => (
          <div key={name} className={`sensor-item ${available ? 'online' : 'offline'}`}>
            <span className="sensor-icon">{icon}</span>
            <div className="sensor-info">
              <strong>{name}</strong>
              <span className={available ? 'status-ok' : 'status-error'}>
                {available ? 'ONLINE' : 'OFFLINE'}
              </span>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}

interface SidebarProps {
  snapshot: RobotSnapshot
  history: RobotSnapshot[]
  timelineIndex: number
  timelineLength: number
  liveMode: boolean
  onTimelineChange: (value: number) => void
  goLive: () => void
  sessions: ReplaySessionInfo[]
  loadSession: (sessionId: string) => Promise<void>
  selectedSessionId: string | null
}

function Sidebar({
  snapshot,
  history,
  timelineIndex,
  timelineLength,
  liveMode,
  onTimelineChange,
  goLive,
  sessions,
  loadSession,
  selectedSessionId,
}: SidebarProps) {
  const metrics = snapshot.metrics

  const formatMetric = (value: number | null | undefined, unit: string = 'm') => {
    if (value === null || value === undefined) return 'N/A'
    return `${value.toFixed(2)} ${unit}`
  }

  const statRows: [string, number | null | undefined][] = [
    ['Forward', metrics.forward],
    ['Left', metrics.left],
    ['Right', metrics.right],
    ['Back', metrics.back],
  ]

  const telemetryEntries: Array<[string, string]> = [
    ['Node Health', metrics.nodeHealth],
    ['Stage', metrics.stage ?? 'unknown'],
    ['Speed', metrics.speed !== null && metrics.speed !== undefined 
      ? `${metrics.speed.toFixed(2)} m/s` 
      : 'N/A'],
    ['Range Min', formatMetric(metrics.rangeMin)],
    ['Range Mean', formatMetric(metrics.rangeMean)],
    ['Range Max', formatMetric(metrics.rangeMax)],
    ['Points', metrics.pointsCaptured?.toString() ?? '0'],
  ]

  const hasHistory = history.length > 0
  const timelineTimestamp = history.length > 0 ? history[timelineIndex]?.timestamp ?? snapshot.timestamp : snapshot.timestamp
  const timelineLabel = liveMode ? 'Live timeline' : selectedSessionId ? 'Replay timeline' : 'Timeline'
  const formattedTimestamp = new Date(timelineTimestamp * 1000).toLocaleTimeString()

  return (
    <aside className="control-panel" style={{ background: colors.panel }}>
      <div className="panel-header">
        <p className="label">LIVE TRACKING</p>
        <h1>{snapshot.missionName}</h1>
        <p className="label">Updated {new Date(snapshot.timestamp * 1000).toLocaleTimeString()}</p>
      </div>

      <SensorHealthPanel metrics={metrics} />

      <div className="metrics-grid">
        {statRows.map(([label, value]) => (
          <div key={label}>
            <p>{label}</p>
            <strong>{formatMetric(value as number | null)}</strong>
          </div>
        ))}
      </div>

      <div className="speed-control">
        <p className="label">MAX LINEAR SPEED</p>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px', paddingBottom: '16px' }}>
          <input
            type="range"
            min="0"
            max="2"
            step="0.1"
            disabled={!liveMode || metrics.nodeHealth !== NodeHealth.NOMINAL}
            defaultValue={1.0}
            onChange={(e) => updateRobotSpeed(parseFloat(e.target.value))}
            style={{ flex: 1 }}
          />
          <span style={{ minWidth: '60px' }}>{metrics.speed !== null && metrics.speed !== undefined ? metrics.speed.toFixed(2) : '1.00'} m/s</span>
        </div>
      </div>

      <div className="telemetry-list">
        {telemetryEntries.map(([label, value]) => (
          <div key={label}>
            <span>{label}</span>
            <strong>{value}</strong>
          </div>
        ))}
      </div>
      {hasHistory && (
        <div className="timeline">
          <div className="timeline-header">
            <p>{timelineLabel}</p>
            <button className="timeline-button" type="button" onClick={goLive} disabled={liveMode}>
              {liveMode ? 'Live' : 'Go live'}
            </button>
          </div>
          <input
            type="range"
            min={0}
            max={Math.max(timelineLength - 1, 0)}
            value={timelineIndex}
            onChange={(event) => onTimelineChange(Number(event.target.value))}
          />
          <div className="timeline-meta">
            <span>
              {timelineIndex + 1}/{Math.max(timelineLength, 1)} · {formattedTimestamp}
            </span>
          </div>
        </div>
      )}

      {sessions.length > 0 && (
        <div className="session-list">
          <p className="label">Replays</p>
          {sessions.map((session) => {
            const createdAt = new Date(session.createdAt * 1000)
            const isSelected = session.sessionId === selectedSessionId
            return (
              <button
                key={session.sessionId}
                type="button"
                className={`session-item${isSelected ? ' selected' : ''}`}
                onClick={() => loadSession(session.sessionId)}
              >
                <span>{session.sessionId}</span>
                <small>{createdAt.toLocaleString()}</small>
                <strong>{session.entryCount} snaps</strong>
              </button>
            )
          })}
        </div>
      )}

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

function App() {
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null)
  const [topics, setTopics] = useState<TopicsSnapshot | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [history, setHistory] = useState<RobotSnapshot[]>([])
  const [timelineIndex, setTimelineIndex] = useState(0)
  const [liveMode, setLiveMode] = useState(true)
  const [sessions, setSessions] = useState<ReplaySessionInfo[]>([])
  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null)
  const [retryCount, setRetryCount] = useState(0)

  useEffect(() => {
    let mounted = true
    const fetchMeta = async () => {
      try {
        const [latest, rawTopics, historyPayload, recordedSessions] = await Promise.all([
          fetchLatestTelemetry(),
          fetchRawTopics(),
          fetchHistory(),
          fetchSessions(),
        ])
        if (!mounted) return
        setSnapshot(latest)
        setTopics(rawTopics)
        setHistory(historyPayload)
        setSessions(recordedSessions)
        setTimelineIndex(historyPayload.length - 1)
        setError(null)
      } catch (err) {
        if (!mounted) return
        setError(err instanceof Error ? err.message : 'Unable to load telemetry')
      }
    }
    fetchMeta()
    return () => {
      mounted = false
    }
  }, [retryCount])

  useEffect(() => {
    if (!liveMode) return

    const disconnectWS = connectTelemetryWS((data) => {
      // The payload might just be the snapshot, or it might contain both snapshot and topics.
      // We'll handle both cases to be robust.
      const fresh: RobotSnapshot = data.snapshot || data
      const rawTopics: TopicsSnapshot | undefined = data.topics

      if (fresh && fresh.timestamp) {
        setSnapshot(fresh)
        setHistory((prev) => [...prev.slice(-59), fresh])
        setTimelineIndex((prev) => Math.min(prev + 1, 59))
      }

      if (rawTopics) {
        setTopics(rawTopics)
      }
    })

    return () => {
      disconnectWS()
    }
  }, [liveMode])

  const handleTimelineChange = (value: number) => {
    setTimelineIndex(value)
    setLiveMode(false)
  }

  useEffect(() => {
    if (!liveMode && history.length > 0) {
      const index = Math.min(Math.max(timelineIndex, 0), history.length - 1)
      setSnapshot(history[index])
    }
  }, [history, liveMode, timelineIndex])

  const goLive = () => {
    setLiveMode(true)
    setSelectedSessionId(null)
    setTimelineIndex(history.length - 1)
  }

  const loadSession = async (sessionId: string) => {
    try {
      const sessionSnapshots = await fetchSession(sessionId)
      setHistory(sessionSnapshots)
      setTimelineIndex(sessionSnapshots.length - 1)
      setSelectedSessionId(sessionId)
      setLiveMode(false)
      setSnapshot(sessionSnapshots[sessionSnapshots.length - 1])
    } catch (err) {
      setError(err instanceof Error ? err.message : `Failed to load session ${sessionId}`)
    }
  }

  if (error) {
    return (
      <div className="shell-loading">
        <p>{error}</p>
        <button type="button" onClick={() => setRetryCount((c) => c + 1)}>Retry</button>
      </div>
    )
  }

  if (!snapshot) {
    return <div className="shell-loading">Loading telemetry…</div>
  }

  return (
    <div className="shell">
      <div className="canvas-wrapper">
        <SceneCanvas snapshot={snapshot} />
      </div>
      <Sidebar
        snapshot={snapshot}
        history={history}
        timelineIndex={timelineIndex}
        timelineLength={history.length}
        liveMode={liveMode}
        onTimelineChange={handleTimelineChange}
        goLive={goLive}
        sessions={sessions}
        loadSession={loadSession}
        selectedSessionId={selectedSessionId}
      />
      <div className="inspector-panel">
        <TopicInspector topics={topics} />
      </div>
    </div>
  )
}

export default App
