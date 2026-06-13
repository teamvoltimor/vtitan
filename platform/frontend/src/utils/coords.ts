/**
 * src/utils/coords.ts
 *
 * Coordinate transforms between simulation space and Three.js space.
 */

import type { Position3D, Vec3 } from '../types'
import { SIMULATION_CONFIG } from '../config'

/**
 * Map simulation coords (origin bottom-left, 0–3 range, Z up) to Three.js
 * (XZ floor plane, Y up), centred on the track. Position3D is a proto message
 * {x, y, z}, not a tuple.
 */
export function simToThree({ x, y, z }: Position3D): Vec3 {
  const center = SIMULATION_CONFIG.TRACK.CENTER
  return [x - center.x, z, -(y - center.y)]
}
