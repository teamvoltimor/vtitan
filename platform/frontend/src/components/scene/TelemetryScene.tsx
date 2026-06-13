import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera } from '@react-three/drei'
import type { RobotSnapshot } from '../../types'
import { SIMULATION_CONFIG, SCENE_CONFIG, THEME } from '../../config'
import { TrackFloor } from './TrackFloor'
import { TrackWalls } from './TrackWalls'
import { LidarPointCloud } from './LidarPointCloud'
import { RobotPath } from './RobotPath'
import { RobotModel } from './RobotModel'

/** Full 3D telemetry scene: track, LiDAR, path and robot. */
export function TelemetryScene({ snapshot }: { snapshot: RobotSnapshot }) {
  const camera = SIMULATION_CONFIG.CAMERA
  return (
    <Canvas shadows dpr={[1, 2]}>
      <color attach="background" args={[THEME.COLORS.BACKGROUND]} />
      <ambientLight intensity={SCENE_CONFIG.AMBIENT_INTENSITY} />
      <directionalLight
        intensity={SCENE_CONFIG.DIRECTIONAL.INTENSITY}
        position={SCENE_CONFIG.DIRECTIONAL.POSITION}
        castShadow
      />
      <TrackFloor />
      <TrackWalls />
      <LidarPointCloud snapshot={snapshot} />
      <RobotPath snapshot={snapshot} />
      <RobotModel snapshot={snapshot} />
      <PerspectiveCamera makeDefault position={camera.POSITION} fov={camera.FOV} />
      <OrbitControls enablePan={false} maxPolarAngle={camera.MAX_POLAR_ANGLE} />
    </Canvas>
  )
}
