import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import type { RobotSnapshot } from '../../types';
import { ROBOT_PATH_CONFIG, THEME } from '../../config';
import { simToThree } from '../../utils/coords';

/**
 * Renders the robot's travelled path as a thin line trail.
 * Cheap by design: a `<Line>` over the raw point list, recomputed only when
 * `path_history` actually changes — no per-snapshot geometry construction.
 */
export function RobotPath({ snapshot }: { snapshot: RobotSnapshot }) {
  const points = useMemo(() => {
    if (!snapshot.path_history || snapshot.path_history.length < 2) return null;
    return snapshot.path_history.map((pt) => {
      const [x, y, z] = simToThree(pt);
      return [x, y + ROBOT_PATH_CONFIG.Y_OFFSET, z] as [number, number, number];
    });
  }, [snapshot.path_history]);

  if (!points) return null;

  return <Line points={points} color={THEME.COLORS.ACCENT} lineWidth={2} toneMapped={false} />;
}
