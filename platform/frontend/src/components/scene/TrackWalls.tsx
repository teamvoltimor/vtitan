import type { Position3D, Vec3 } from '../../types'
import { SCENE_CONFIG } from '../../config'
import { simToThree } from '../../utils/coords'

interface WallSpec {
  center: Position3D // world coordinates
  args: Vec3 // three.js box dimensions [x, y, z]
}

/**
 * Build the four walls of an axis-aligned square between [lo, hi] on both axes.
 * Walls extend up to `height`; `thickness` is the wall depth.
 */
function squareWalls(lo: number, hi: number, height: number, thickness: number): WallSpec[] {
  const mid = (lo + hi) / 2
  const span = hi - lo
  const z = height / 2
  return [
    { center: { x: mid, y: lo, z }, args: [span, height, thickness] }, // south
    { center: { x: mid, y: hi, z }, args: [span, height, thickness] }, // north
    { center: { x: lo, y: mid, z }, args: [thickness, height, span] }, // west
    { center: { x: hi, y: mid, z }, args: [thickness, height, span] }, // east
  ]
}

/**
 * Renders the WRO track boundary: outer 3×3 m perimeter, inner square, and a
 * floor grid — giving the LiDAR point cloud real geometry to sit against.
 */
export function TrackWalls() {
  const t = SCENE_CONFIG.TRACK
  const walls = [
    ...squareWalls(t.OUTER_MIN, t.OUTER_MAX, t.WALL_HEIGHT, t.WALL_THICKNESS),
    ...squareWalls(t.INNER_MIN, t.INNER_MAX, t.WALL_HEIGHT, t.WALL_THICKNESS),
  ]

  const gridSize = t.OUTER_MAX - t.OUTER_MIN

  return (
    <group>
      <gridHelper
        args={[gridSize, t.GRID_DIVISIONS, t.WALL_COLOR, t.WALL_COLOR]}
        position={[0, 0.001, 0]}
      />
      {walls.map((wall, i) => (
        <mesh key={i} position={simToThree(wall.center)}>
          <boxGeometry args={wall.args} />
          <meshStandardMaterial
            color={t.WALL_COLOR}
            transparent
            opacity={t.WALL_OPACITY}
          />
        </mesh>
      ))}
    </group>
  )
}
