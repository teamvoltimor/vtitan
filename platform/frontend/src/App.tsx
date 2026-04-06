import { useEffect, useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera } from '@react-three/drei'
import * as THREE from 'three'
import './App.css'

import { type Position3D, type RobotSnapshot } from './types'
import { TelemetryProvider, useTelemetry } from './contexts/TelemetryContext'
import { SIMULATION_CONFIG, THEME } from './config'
import { Sidebar } from './components/Sidebar'
import { TopicInspector } from './components/TopicInspector'

/**
 * Map simulation coords (origin bottom-left, 0–3 range) to Three.js (XZ floor, Y up).
 * Uses config instead of hardcoded values.
 */
const simToThree = ([x, y, z]: Position3D): [number, number, number] => {
  const center = SIMULATION_CONFIG.TRACK.CENTER
  return [x - center.x, z, -(y - center.y)]
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
        <sphereGeometry
          args={[
            SIMULATION_CONFIG.ROBOT.DIMENSIONS[0],
            8,
            8,
          ]}
        />
        <meshStandardMaterial
          color={THEME.COLORS.ERROR}
          emissive={THEME.COLORS.ERROR}
        />
      </mesh>
    )
  }

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial
        size={SIMULATION_CONFIG.ROBOT.DIMENSIONS[0]}
        color={THEME.COLORS.HIGHLIGHT}
        transparent
        opacity={snapshot.metrics.lidarAvailable ? 0.9 : 0.3}
        depthTest={false}
      />
    </points>
  )
}

function RobotPath({ snapshot }: { snapshot: RobotSnapshot }) {
  const curve = useMemo(() => {
    if (!snapshot.pathHistory || snapshot.pathHistory.length < 2) return null
    const pts = snapshot.pathHistory.map((pt) => {
      const [x, y, z] = simToThree(pt)
      return new THREE.Vector3(
        x,
        y + SIMULATION_CONFIG.ROBOT.DIMENSIONS[0],
        z
      )
    })
    return new THREE.CatmullRomCurve3(pts)
  }, [snapshot])

  const tubeGeometry = useMemo(
    () =>
      curve
        ? new THREE.TubeGeometry(
            curve,
            SIMULATION_CONFIG.ROBOT.DIMENSIONS[0],
            SIMULATION_CONFIG.ROBOT.DIMENSIONS[1],
            SIMULATION_CONFIG.ROBOT.DIMENSIONS[2],
            false
          )
        : null,
    [curve]
  )
  useEffect(() => () => tubeGeometry?.dispose(), [tubeGeometry])

  if (!tubeGeometry) return null

  return (
    <mesh geometry={tubeGeometry}>
      <meshStandardMaterial
        color={THEME.COLORS.ACCENT}
        emissive={THEME.COLORS.ACCENT}
        emissiveIntensity={0.6}
      />
    </mesh>
  )
}

function Robot({ snapshot }: { snapshot: RobotSnapshot }) {
  const position = snapshot.robotPosition ?? SIMULATION_CONFIG.ROBOT.DEFAULT_POSITION
  const orientation = snapshot.robotOrientation ?? SIMULATION_CONFIG.ROBOT.DEFAULT_ORIENTATION_RADIANS
  const available = snapshot.metrics.odometryAvailable

  return (
    <mesh
      position={simToThree(position as Position3D)}
      rotation={[0, -orientation, 0]}
    >
      <boxGeometry args={[...SIMULATION_CONFIG.ROBOT.DIMENSIONS]} />
      <meshStandardMaterial
        color={available ? '#f3c677' : '#666666'}
        emissive={available ? '#e57f2e' : '#333333'}
        opacity={available ? 1.0 : 0.5}
        transparent={!available}
      />
    </mesh>
  )
}

function TrackFloor() {
  const size = SIMULATION_CONFIG.TRACK.SIZE
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[size.width, size.height]} />
      <meshStandardMaterial
        color="#111b27"
        metalness={0.4}
        roughness={0.7}
        side={THREE.DoubleSide}
      />
    </mesh>
  )
}

function SceneCanvas({ snapshot }: { snapshot: RobotSnapshot }) {
  const camera = SIMULATION_CONFIG.CAMERA
  return (
    <Canvas shadows dpr={[1, 2]}>
      <color attach="background" args={[THEME.COLORS.BACKGROUND]} />
      <ambientLight intensity={0.6} />
      <directionalLight intensity={1.1} position={[3, -3, 4]} castShadow />
      <TrackFloor />
      <LiDARPointCloud snapshot={snapshot} />
      <RobotPath snapshot={snapshot} />
      <Robot snapshot={snapshot} />
      <PerspectiveCamera
        makeDefault
        position={camera.POSITION}
        fov={camera.FOV}
      />
      <OrbitControls
        enablePan={false}
        maxPolarAngle={camera.MAX_POLAR_ANGLE}
      />
    </Canvas>
  )
}

/**
 * Main app content using TelemetryContext.
 * Manages loading, error states, and layout composition.
 */
function AppContent() {
  const { snapshot, topics, loading, error, retry } = useTelemetry()

  if (error) {
    return (
      <div className="shell-loading">
        <p>{error}</p>
        <button type="button" onClick={retry}>
          Retry
        </button>
      </div>
    )
  }

  if (loading || !snapshot) {
    return <div className="shell-loading">Loading telemetry…</div>
  }

  return (
    <div className="shell">
      <div className="canvas-wrapper">
        <SceneCanvas snapshot={snapshot} />
      </div>
      <Sidebar />
      <div className="inspector-panel">
        <TopicInspector topics={topics} />
      </div>
    </div>
  )
}

/**
 * Root App component wrapped with TelemetryProvider.
 */
function App() {
  return (
    <TelemetryProvider>
      <AppContent />
    </TelemetryProvider>
  )
}

export default App

