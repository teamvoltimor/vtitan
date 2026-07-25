import { memo } from 'react';
import type { Position3D, Vec3 } from '../../types';
import { SCENE_CONFIG } from '../../config';
import { simToThree } from '../../utils/coords';

interface WallSpec {
  center: Position3D; // world coordinates
  args: Vec3; // three.js box dimensions [x, y, z]
}

/**
 * Build the four walls of an axis-aligned square between [lo, hi] on both axes.
 * Walls extend up to `height`; `thickness` is the wall depth.
 */
function squareWalls(lo: number, hi: number, height: number, thickness: number): WallSpec[] {
  const mid = (lo + hi) / 2;
  const span = hi - lo;
  const z = height / 2;
  return [
    { center: { x: mid, y: lo, z }, args: [span, height, thickness] }, // south
    { center: { x: mid, y: hi, z }, args: [span, height, thickness] }, // north
    { center: { x: lo, y: mid, z }, args: [thickness, height, span] }, // west
    { center: { x: hi, y: mid, z }, args: [thickness, height, span] }, // east
  ];
}

// Static geometry — computed once at module load rather than per render.
const TRACK = SCENE_CONFIG.TRACK;
const WALLS = [
  ...squareWalls(TRACK.OUTER_MIN, TRACK.OUTER_MAX, TRACK.WALL_HEIGHT, TRACK.WALL_THICKNESS),
  ...squareWalls(TRACK.INNER_MIN, TRACK.INNER_MAX, TRACK.WALL_HEIGHT, TRACK.WALL_THICKNESS),
];
const GRID_SIZE = TRACK.OUTER_MAX - TRACK.OUTER_MIN;

/**
 * Renders the WRO track boundary: outer 3×3 m perimeter, inner square, and a
 * floor grid — giving the LiDAR point cloud real geometry to sit against.
 * Static geometry — memoized since it takes no props.
 */
export const TrackWalls = memo(function TrackWalls() {
  return (
    <group>
      <gridHelper
        args={[GRID_SIZE, TRACK.GRID_DIVISIONS, TRACK.WALL_COLOR, TRACK.WALL_COLOR]}
        position={[0, 0.001, 0]}
      />
      {WALLS.map((wall) => (
        <mesh
          key={`${wall.center.x}-${wall.center.y}-${wall.center.z}`}
          position={simToThree(wall.center)}
        >
          <boxGeometry args={wall.args} />
          <meshStandardMaterial color={TRACK.WALL_COLOR} transparent opacity={TRACK.WALL_OPACITY} />
        </mesh>
      ))}
    </group>
  );
});
