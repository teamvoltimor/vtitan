import { describe, it, expect } from 'vitest'
import {
  generateMockSnapshot,
  generateMockTopics,
  generateMockSession,
  generateMockSessions,
} from './mockData'
import { schemas } from './schemas'

describe('mock data conforms to the backend wire schema', () => {
  it('generateMockSnapshot passes RobotSnapshot validation', () => {
    expect(() => schemas.RobotSnapshot.parse(generateMockSnapshot(7))).not.toThrow()
  })

  it('generateMockTopics passes TopicsSnapshot validation', () => {
    expect(() => schemas.TopicsSnapshot.parse(generateMockTopics(7))).not.toThrow()
  })

  it('generateMockSessions passes the sessions schema', () => {
    expect(() => schemas.SessionsResponse.parse(generateMockSessions())).not.toThrow()
  })
})

describe('mock data shape', () => {
  it('keeps lidar points inside the 0–3 m track bounds', () => {
    const snap = generateMockSnapshot(3)
    expect(snap.lidar_points.length).toBeGreaterThan(0)
    for (const p of snap.lidar_points) {
      expect(p.x).toBeGreaterThanOrEqual(-0.1)
      expect(p.x).toBeLessThanOrEqual(3.1)
      expect(p.y).toBeGreaterThanOrEqual(-0.1)
      expect(p.y).toBeLessThanOrEqual(3.1)
    }
  })

  it('exposes the expected ROS topics', () => {
    const names = generateMockTopics(1).topics.map((t) => t.topic_name)
    expect(names).toEqual(
      expect.arrayContaining(['/scan', '/odom', '/imu', '/cmd_vel', '/joint_states', '/detections'])
    )
  })

  it('generates a replay session of the requested length', () => {
    expect(generateMockSession('demo-lap-2')).toHaveLength(60)
    expect(generateMockSession('demo-lap-1')).toHaveLength(48)
  })
})
