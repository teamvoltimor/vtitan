import { useEffect, useMemo, useRef } from 'react';
import type { BufferAttribute, BufferGeometry } from 'three';
import type { RobotSnapshot } from '../../types';
import { LIDAR_CONFIG, SCENE_CONFIG, THEME } from '../../config';
import { simToThree } from '../../utils/coords';

const MAX_POINTS = LIDAR_CONFIG.POINT_CLOUD.MAX_POINTS;

/** Renders the LiDAR scan as a point cloud, or a marker when no data exists. */
export function LidarPointCloud({ snapshot }: { snapshot: RobotSnapshot }) {
  const geometryRef = useRef<BufferGeometry>(null);
  // Fixed-capacity buffer, allocated once and updated in place per snapshot
  // rather than reallocated — avoids a new Float32Array + GPU upload every frame.
  const positions = useMemo(() => new Float32Array(MAX_POINTS * 3), []);

  const points = snapshot.lidar_points;
  const hasData = snapshot.metrics.lidar_available && points.length > 0;

  useEffect(() => {
    if (!geometryRef.current || !hasData) return;
    const count = Math.min(points.length, MAX_POINTS);
    for (let i = 0; i < count; i++) {
      const [x, y, z] = simToThree(points[i]);
      positions[i * 3 + 0] = x;
      positions[i * 3 + 1] = y;
      positions[i * 3 + 2] = z;
    }
    const attribute = geometryRef.current.getAttribute('position') as BufferAttribute;
    attribute.needsUpdate = true;
    geometryRef.current.setDrawRange(0, count);
  }, [points, hasData, positions]);

  if (!hasData) {
    const sphere = SCENE_CONFIG.NO_DATA_SPHERE;
    return (
      <mesh position={sphere.POSITION}>
        <sphereGeometry
          args={[LIDAR_CONFIG.POINT_CLOUD.NO_DATA_RADIUS, sphere.SEGMENTS, sphere.SEGMENTS]}
        />
        <meshStandardMaterial color={THEME.COLORS.ERROR} emissive={THEME.COLORS.ERROR} />
      </mesh>
    );
  }

  const opacity = snapshot.metrics.lidar_available
    ? LIDAR_CONFIG.POINT_CLOUD.OPACITY.AVAILABLE
    : LIDAR_CONFIG.POINT_CLOUD.OPACITY.UNAVAILABLE;

  return (
    <points>
      <bufferGeometry ref={geometryRef}>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial
        size={LIDAR_CONFIG.POINT_CLOUD.POINT_SIZE}
        color={THEME.COLORS.HIGHLIGHT}
        transparent
        opacity={opacity}
      />
    </points>
  );
}
