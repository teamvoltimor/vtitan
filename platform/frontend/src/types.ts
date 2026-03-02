export interface TelemetryMetrics {
  timestamp: number
  nodeHealth: 'nominal' | 'watchdog' | 'replanning'
  pointsCaptured: number
  rangeMin: number
  rangeMax: number
  rangeMean: number
  forward: number
  left: number
  right: number
  back: number
  speed: number
  stage: string
}

export type Position3D = [number, number, number]

export interface RobotSnapshot {
  timestamp: number
  missionName: string
  robotPosition: Position3D
  robotOrientation: number
  lidarPoints: Array<Position3D>
  pathHistory: Array<Position3D>
  logs: Array<string>
  metrics: TelemetryMetrics
}

export interface ReplaySessionInfo {
  sessionId: string
  createdAt: number
  entryCount: number
}
