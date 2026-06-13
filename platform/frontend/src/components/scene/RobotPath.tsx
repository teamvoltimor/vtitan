import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import type { RobotSnapshot } from '../../types'
import { ROBOT_PATH_CONFIG, SCENE_CONFIG, THEME } from '../../config'
import { simToThree } from '../../utils/coords'

/** Renders the robot's travelled path as a glowing tube. */
export function RobotPath({ snapshot }: { snapshot: RobotSnapshot }) {
  const curve = useMemo(() => {
    if (!snapshot.path_history || snapshot.path_history.length < 2) return null
    const pts = snapshot.path_history.map((pt) => {
      const [x, y, z] = simToThree(pt)
      return new THREE.Vector3(x, y + ROBOT_PATH_CONFIG.Y_OFFSET, z)
    })
    return new THREE.CatmullRomCurve3(pts)
  }, [snapshot])

  const tubeGeometry = useMemo(
    () =>
      curve
        ? new THREE.TubeGeometry(
            curve,
            ROBOT_PATH_CONFIG.TUBE_SEGMENTS,
            ROBOT_PATH_CONFIG.TUBE_RADIUS,
            ROBOT_PATH_CONFIG.TUBE_RADIAL_SEGMENTS,
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
        emissiveIntensity={SCENE_CONFIG.PATH.EMISSIVE_INTENSITY}
      />
    </mesh>
  )
}
