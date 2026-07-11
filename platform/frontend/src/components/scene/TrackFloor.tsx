import * as THREE from 'three';
import { SIMULATION_CONFIG, SCENE_CONFIG } from '../../config';

/** Flat reflective track floor. */
export function TrackFloor() {
  const size = SIMULATION_CONFIG.TRACK.SIZE;
  const floor = SCENE_CONFIG.FLOOR;
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[size.width, size.height]} />
      <meshStandardMaterial
        color={floor.COLOR}
        metalness={floor.METALNESS}
        roughness={floor.ROUGHNESS}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}
