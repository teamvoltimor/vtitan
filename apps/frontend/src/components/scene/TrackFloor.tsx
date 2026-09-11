import { memo } from 'react';
import { SCENE_CONFIG, SIMULATION_CONFIG } from '../../config';

/** Flat reflective track floor. Static geometry — memoized since it takes no props. */
export const TrackFloor = memo(function TrackFloor() {
  const size = SIMULATION_CONFIG.TRACK.SIZE;
  const floor = SCENE_CONFIG.FLOOR;
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[size.width, size.height]} />
      <meshStandardMaterial
        color={floor.COLOR}
        metalness={floor.METALNESS}
        roughness={floor.ROUGHNESS}
      />
    </mesh>
  );
});
