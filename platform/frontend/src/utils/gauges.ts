/**
 * src/utils/gauges.ts
 * 
 * SVG gauge rendering utilities to eliminate duplication.
 * Provides generic arc path and needle calculation for visual gauges.
 */

import { clamp } from './formatting'

/**
 * SVG arc path configuration.
 * Used for speed gauges, motor dials, and other circular visualizations.
 */
export interface GaugeConfig {
  center: { x: number; y: number }
  startPoint: { x: number; y: number }
  radius: number
  max: number
  angleRange?: {
    start: number // Start angle in radians
    end: number   // End angle in radians
  }
}

/**
 * Calculate SVG path for gauge arc visualization.
 * 
 * @param value - Current value to display
 * @param config - Gauge configuration
 * @returns SVG path string for the arc
 * 
 * @example
 * const config = {
 *   center: { x: 100, y: 100 },
 *   startPoint: { x: 20, y: 100 },
 *   radius: 80,
 *   max: 100,
 *   angleRange: { start: Math.PI, end: 2 * Math.PI }
 * }
 * const path = calculateGaugePath(50, config)
 * // Returns SVG arc path representing 50% of the gauge
 */
export function calculateGaugePath(value: number, config: GaugeConfig): string {
  const {
    center,
    startPoint,
    radius,
    max,
    angleRange = { start: Math.PI, end: 2 * Math.PI },
  } = config

  // Clamp value to [0, max] range
  const clampedValue = clamp(value, 0, max)
  const percent = clampedValue / max

  // Calculate angle based on value and angle range
  const angleSpan = angleRange.end - angleRange.start
  const angle = angleRange.start + percent * angleSpan

  // Calculate end point on the circle
  const endX = center.x + radius * Math.cos(angle)
  const endY = center.y + radius * Math.sin(angle)

  // Determine if arc should be greater than 180 degrees
  const largeArc = percent > 0.5 ? 1 : 0

  // Return SVG arc path
  // M = move to, A = arc to
  return `M ${startPoint.x} ${startPoint.y} A ${radius} ${radius} 0 ${largeArc} 1 ${endX} ${endY}`
}

/**
 * Needle position for gauge visualization.
 */
export interface NeedlePosition {
  x: number
  y: number
}

/**
 * Calculate needle endpoint for gauge visualization.
 * 
 * @param value - Current value
 * @param config - Needle configuration
 * @returns Position of needle endpoint
 * 
 * @example
 * const needle = calculateNeedle(50, {
 *   center: { x: 100, y: 100 },
 *   length: 70,
 *   max: 100,
 *   angle: Math.PI  // Rotation offset
 * })
 * // Returns { x: 65, y: 30 } (approximate)
 */
export function calculateNeedle(
  value: number,
  config: {
    center: { x: number; y: number }
    length: number
    max: number
    angle?: number // Base angle offset in radians
  }
): NeedlePosition {
  const { center, length, max, angle = 0 } = config

  // Clamp value to [0, max] range
  const clampedValue = clamp(value, 0, max)
  const percent = clampedValue / max

  // Calculate angle: base angle + percent through PI range
  const rad = angle + percent * Math.PI

  return {
    x: center.x + length * Math.cos(rad),
    y: center.y + length * Math.sin(rad),
  }
}

/**
 * Create SVG arc path data for circular progress indicator.
 * Useful for progress bars, circular gauges, etc.
 * 
 * @param value - Current value (0-max)
 * @param max - Maximum value
 * @param radius - Radius of the arc
 * @param config - Additional arc configuration
 * @returns Object with path and metrics
 */
export function createArcPath(
  value: number,
  max: number,
  radius: number,
  config: {
    center?: { x: number; y: number }
    startAngle?: number
    direction?: 'clockwise' | 'counterclockwise'
  } = {}
) {
  const {
    center = { x: 0, y: 0 },
    startAngle = -Math.PI / 2,
    direction = 'clockwise',
  } = config

  const clampedValue = clamp(value, 0, max)
  const percent = clampedValue / max
  const angle = percent * 2 * Math.PI
  const isLargeArc = percent > 0.5 ? 1 : 0
  const sweep = direction === 'clockwise' ? 1 : 0

  const x1 = center.x + radius * Math.cos(startAngle)
  const y1 = center.y + radius * Math.sin(startAngle)

  const endAngle = startAngle + angle
  const x2 = center.x + radius * Math.cos(endAngle)
  const y2 = center.y + radius * Math.sin(endAngle)

  const path = `M ${x1} ${y1} A ${radius} ${radius} 0 ${isLargeArc} ${sweep} ${x2} ${y2}`

  return {
    path,
    percent,
    startPoint: { x: x1, y: y1 },
    endPoint: { x: x2, y: y2 },
  }
}

/**
 * Convert radians to SVG angle (0-360 degrees, where 0° is right, 90° is down).
 * SVG uses clockwise rotation from the positive x-axis.
 */
export function radiansToSvgDegrees(radians: number): number {
  // Convert to degrees and normalize to [0, 360)
  let degrees = (radians * 180) / Math.PI
  degrees = ((degrees % 360) + 360) % 360
  return degrees
}

/**
 * Calculate polygon points for gauge tick marks.
 * 
 * @param count - Number of ticks
 * @param radius - Distance from center
 * @param innerRadius - Inner radius of tick
 * @param startAngle - Starting angle in radians
 * @param angleSpan - Total angle span in radians
 * @returns Array of tick mark positions
 */
export function generateGaugeTicks(
  count: number,
  radius: number,
  innerRadius: number,
  startAngle: number = Math.PI,
  angleSpan: number = Math.PI
): Array<{ outer: { x: number; y: number }; inner: { x: number; y: number } }> {
  const ticks = []
  const angleIncrement = angleSpan / (count - 1)

  for (let i = 0; i < count; i++) {
    const angle = startAngle + i * angleIncrement

    ticks.push({
      outer: {
        x: radius * Math.cos(angle),
        y: radius * Math.sin(angle),
      },
      inner: {
        x: innerRadius * Math.cos(angle),
        y: innerRadius * Math.sin(angle),
      },
    })
  }

  return ticks
}
