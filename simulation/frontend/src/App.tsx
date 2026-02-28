import { useEffect, useMemo, useRef, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera } from '@react-three/drei'
import * as THREE from 'three'
import './App.css'

import type { ReplaySessionInfo, SimulationSnapshot } from './types'
import { fetchLatestTelemetry, fetchHistory, fetchSession, fetchSessions } from './api/telemetry'

const colors = {
  background: '#050b12',
  panel: '#0f1c2b',
  accent: '#ff8a65',
  highlight: '#5fdde5',
}

function LiDARPointCloud({ snapshot }: { snapshot: SimulationSnapshot }) {
  const positions = useMemo(() => {
    const data = new Float32Array(snapshot.lidarPoints.length * 3)
    snapshot.lidarPoints.forEach(([x, y, z], index) => {
      data[index * 3 + 0] = x
      data[index * 3 + 1] = y
      data[index * 3 + 2] = z
    })
    return data
  }, [snapshot])

  if (positions.length === 0) return null

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial size={0.03} color={colors.highlight} transparent opacity={0.9} depthTest={false} />
    </points>
  )
}

function RobotPath({ snapshot }: { snapshot: SimulationSnapshot }) {
  const curve = useMemo(
    () => {
      const pts = snapshot.pathHistory.map(([x, y]) => new THREE.Vector3(x, y, 0.03))
      return pts.length >= 2 ? new THREE.CatmullRomCurve3(pts) : null
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

function Robot({ snapshot }: { snapshot: SimulationSnapshot }) {
  return (
    <mesh position={snapshot.robotPosition} rotation={[0, 0, snapshot.robotOrientation]}>
      <boxGeometry args={[0.2, 0.14, 0.08]} />
      <meshStandardMaterial color="#f3c677" emissive="#e57f2e" />
    </mesh>
  )
}

function TrackFloor() {
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[6, 6]} />
      <meshStandardMaterial color="#111b27" metalness={0.4} roughness={0.7} />
    </mesh>
  )
}

function SceneCanvas({ snapshot }: { snapshot: SimulationSnapshot }) {
  return (
    <Canvas shadows dpr={[1, 2]}>
      <color attach="background" args={[colors.background]} />
      <ambientLight intensity={0.6} />
      <directionalLight intensity={1.1} position={[3, -3, 4]} castShadow />
      <TrackFloor />
      <LiDARPointCloud snapshot={snapshot} />
      <RobotPath snapshot={snapshot} />
      <Robot snapshot={snapshot} />
      <PerspectiveCamera makeDefault position={[0, -4, 2.4]} fov={45} />
      <OrbitControls enablePan={false} maxPolarAngle={Math.PI / 2.4} />
    </Canvas>
  )
}

interface SidebarProps {
  snapshot: SimulationSnapshot
  history: SimulationSnapshot[]
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
  const statRows: [string, number][] = [
    ['Forward', metrics.forward],
    ['Left', metrics.left],
    ['Right', metrics.right],
    ['Back', metrics.back],
  ]

  const telemetryEntries: Array<[string, string | number]> = [
    ['Node Health', metrics.nodeHealth],
    ['Stage', metrics.stage],
    ['Speed', `${metrics.speed.toFixed(2)} m/s`],
    ['Range Min', `${metrics.rangeMin.toFixed(2)} m`],
    ['Range Mean', `${metrics.rangeMean.toFixed(2)} m`],
    ['Range Max', `${metrics.rangeMax.toFixed(2)} m`],
    ['Points', metrics.pointsCaptured],
  ]

  const hasHistory = history.length > 0
  const timelineTimestamp = history.length > 0 ? history[timelineIndex]?.timestamp ?? snapshot.timestamp : snapshot.timestamp
  const timelineLabel = liveMode ? 'Live timeline' : selectedSessionId ? 'Replay timeline' : 'Timeline'
  const formattedTimestamp = new Date(timelineTimestamp * 1000).toLocaleTimeString()

  return (
    <aside className="control-panel" style={{ background: colors.panel }}>
      <div className="panel-header">
        <p className="label">LIVE TRACKING</p>
        <h1>{snapshot.scenarioName}</h1>
        <p className="label">Updated {new Date(snapshot.timestamp * 1000).toLocaleTimeString()}</p>
      </div>
      <div className="metrics-grid">
        {statRows.map(([label, value]) => (
          <div key={label}>
            <p>{label}</p>
            <strong>{`${value.toFixed(2)} m`}</strong>
          </div>
        ))}
      </div>
      <div className="telemetry-list">
        {telemetryEntries.map(([label, value]) => (
          <div key={label}>
            <span>{label}</span>
            <strong>{typeof value === 'number' ? value.toFixed(2) : value}</strong>
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
          <span>ROS Bridge</span>
        </div>
        <ul>
          {snapshot.logs.map((log) => (
            <li key={log}>{log}</li>
          ))}
        </ul>
      </div>
    </aside>
  )
}

function App() {
  const [snapshot, setSnapshot] = useState<SimulationSnapshot | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [history, setHistory] = useState<SimulationSnapshot[]>([])
  const [timelineIndex, setTimelineIndex] = useState(0)
  const [liveMode, setLiveMode] = useState(true)
  const [sessions, setSessions] = useState<ReplaySessionInfo[]>([])
  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null)
  const liveInterval = useRef<number | null>(null)

  useEffect(() => {
    let mounted = true
    const fetchMeta = async () => {
      try {
        const [latest, historyPayload, recordedSessions] = await Promise.all([
          fetchLatestTelemetry(),
          fetchHistory(),
          fetchSessions(),
        ])
        if (!mounted) return
        setSnapshot(latest)
        setHistory(historyPayload)
        setSessions(recordedSessions)
        setTimelineIndex(historyPayload.length - 1)
      } catch (err) {
        if (!mounted) return
        setError(err instanceof Error ? err.message : 'Unable to load telemetry')
      }
    }
    fetchMeta()
    return () => {
      mounted = false
    }
  }, [])

  useEffect(() => {
    if (liveInterval.current) {
      clearInterval(liveInterval.current)
      liveInterval.current = null
    }

    if (liveMode) {
      liveInterval.current = window.setInterval(async () => {
        const fresh = await fetchLatestTelemetry()
        setSnapshot(fresh)
      }, 2500)
    }

    return () => {
      if (liveInterval.current) {
        clearInterval(liveInterval.current)
      }
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
    const sessionSnapshots = await fetchSession(sessionId)
    setHistory(sessionSnapshots)
    setTimelineIndex(sessionSnapshots.length - 1)
    setSelectedSessionId(sessionId)
    setLiveMode(false)
    setSnapshot(sessionSnapshots[sessionSnapshots.length - 1])
  }

  if (error) {
    return <div className="shell-loading">{error}</div>
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
      <div className="atmosphere" />
    </div>
  )
}

export default App
