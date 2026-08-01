import { SCENE_CONFIG, SIMULATION_CONFIG } from '../../config';
import type { RobotSnapshot } from '../../types';
import { simToThree } from '../../utils/coords';

/** Renders the robot chassis plus a forward-heading marker. */
export function RobotModel({ snapshot }: { snapshot: RobotSnapshot }) {
  const robot = SCENE_CONFIG.ROBOT;
  const dims = SIMULATION_CONFIG.ROBOT.DIMENSIONS;
  const position = snapshot.robot_position ?? SIMULATION_CONFIG.ROBOT.DEFAULT_POSITION;
  const orientation =
    snapshot.robot_orientation ?? SIMULATION_CONFIG.ROBOT.DEFAULT_ORIENTATION_RADIANS;
  const available = snapshot.metrics.odometry_available;

  return (
    <group position={simToThree(position)} rotation={[0, -orientation, 0]}>
      <mesh castShadow>
        <boxGeometry args={[...dims]} />
        <meshStandardMaterial
          color={available ? robot.COLOR_ACTIVE : robot.COLOR_INACTIVE}
          emissive={available ? robot.EMISSIVE_ACTIVE : robot.EMISSIVE_INACTIVE}
          opacity={available ? 1.0 : robot.INACTIVE_OPACITY}
          transparent={!available}
        />
      </mesh>
      {/* Forward-heading marker (points along the robot's local +X / front). */}
      <mesh
        position={[dims[0] / 2 + robot.HEADING_LENGTH / 2, 0, 0]}
        rotation={[0, 0, -Math.PI / 2]}
      >
        <coneGeometry args={[robot.HEADING_RADIUS, robot.HEADING_LENGTH, 8]} />
        <meshStandardMaterial color={robot.EMISSIVE_ACTIVE} emissive={robot.EMISSIVE_ACTIVE} />
      </mesh>
    </group>
  );
}
