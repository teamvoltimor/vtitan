/**
 * src/utils/gauges.ts
 *
 * SVG gauge geometry helpers: arc path and needle endpoint calculation.
 */

import { clamp } from './formatting'

export interface GaugeConfig {
  center: { x: number; y: number }
  startPoint: { x: number; y: number }
  radius: number
  max: number
  angleRange?: {
    start: number // radians
    end: number // radians
  }
}

/**
 * SVG path for a gauge arc representing `value` out of `config.max`.
 *
 * @example
 * calculateGaugePath(50, {
 *   center: { x: 100, y: 100 }, startPoint: { x: 20, y: 100 },
 *   radius: 80, max: 100, angleRange: { start: Math.PI, end: 2 * Math.PI },
 * })
 */
export function calculateGaugePath(value: number, config: GaugeConfig): string {
  const { center, startPoint, radius, max, angleRange = { start: Math.PI, end: 2 * Math.PI } } =
    config

  const percent = clamp(value, 0, max) / max
  const angleSpan = angleRange.end - angleRange.start
  const angle = angleRange.start + percent * angleSpan

  const endX = center.x + radius * Math.cos(angle)
  const endY = center.y + radius * Math.sin(angle)
  const largeArc = percent > 0.5 ? 1 : 0

  return `M ${startPoint.x} ${startPoint.y} A ${radius} ${radius} 0 ${largeArc} 1 ${endX} ${endY}`
}

export interface NeedlePosition {
  x: number
  y: number
}

/**
 * Endpoint of a gauge needle for `value`, rotated by `config.angle`.
 *
 * @example
 * calculateNeedle(50, { center: { x: 100, y: 100 }, length: 70, max: 100, angle: Math.PI })
 */
export function calculateNeedle(
  value: number,
  config: { center: { x: number; y: number }; length: number; max: number; angle?: number }
): NeedlePosition {
  const { center, length, max, angle = 0 } = config

  const percent = clamp(value, 0, max) / max
  const rad = angle + percent * Math.PI

  return {
    x: center.x + length * Math.cos(rad),
    y: center.y + length * Math.sin(rad),
  }
}
