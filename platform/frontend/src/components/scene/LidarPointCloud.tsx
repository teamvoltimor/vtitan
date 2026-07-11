import { useMemo } from 'react';
import type { RobotSnapshot } from '../../types';
import { LIDAR_CONFIG, SCENE_CONFIG, THEME } from '../../config';
import { simToThree } from '../../utils/coords';

/** Renders the LiDAR scan as a point cloud, or a marker when no data exists. */
export function LidarPointCloud({ snapshot }: { snapshot: RobotSnapshot }) {
  const positions = useMemo(() => {
    if (!snapshot.metrics.lidar_available || snapshot.lidar_points.length === 0) {
      return new Float32Array(0);
    }

    const data = new Float32Array(snapshot.lidar_points.length * 3);
    snapshot.lidar_points.forEach((pt, index) => {
      const [x, y, z] = simToThree(pt);
      data[index * 3 + 0] = x;
      data[index * 3 + 1] = y;
      data[index * 3 + 2] = z;
    });
    return data;
  }, [snapshot]);

  if (positions.length === 0) {
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
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial
        size={LIDAR_CONFIG.POINT_CLOUD.POINT_SIZE}
        color={THEME.COLORS.HIGHLIGHT}
        transparent
        opacity={opacity}
        depthTest={false}
      />
    </points>
  );
}
