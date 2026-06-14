import { describe, it, expect } from 'vitest'
import { calculateGaugePath, calculateNeedle } from './gauges'

const cfg = {
  center: { x: 100, y: 100 },
  startPoint: { x: 20, y: 100 },
  radius: 80,
  max: 100,
  angleRange: { start: Math.PI, end: 2 * Math.PI },
}

describe('calculateGaugePath', () => {
  it('produces an SVG arc command from the start point', () => {
    const path = calculateGaugePath(50, cfg)
    expect(path.startsWith('M 20 100 A 80 80')).toBe(true)
  })
  it('clamps out-of-range values', () => {
    expect(calculateGaugePath(999, cfg)).toBe(calculateGaugePath(100, cfg))
    expect(calculateGaugePath(-5, cfg)).toBe(calculateGaugePath(0, cfg))
  })
})

describe('calculateNeedle', () => {
  it('returns the centre-left point at zero (angle = PI)', () => {
    const { x, y } = calculateNeedle(0, { center: { x: 100, y: 100 }, length: 70, max: 100, angle: Math.PI })
    expect(x).toBeCloseTo(30) // 100 + 70*cos(PI)
    expect(y).toBeCloseTo(100)
  })
})
