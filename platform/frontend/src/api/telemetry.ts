import type { ReplaySessionInfo, RobotSnapshot, TopicsSnapshot } from '../types'

const TELEMETRY_BASE = (import.meta.env.VITE_TELEMETRY_BASE ?? '').replace(/\/$/, '')

const resolveUrl = (path: string) => (TELEMETRY_BASE ? `${TELEMETRY_BASE}${path}` : path)

async function fetchJson<T>(path: string): Promise<T> {
  const response = await fetch(resolveUrl(path), { cache: 'no-store' })
  if (!response.ok) {
    throw new Error(`Failed to load ${path}`)
  }
  return response.json()
}

export const fetchLatestTelemetry = () => fetchJson<RobotSnapshot>('/telemetry/latest')
export const fetchRawTopics = () => fetchJson<TopicsSnapshot>('/telemetry/topics')
export const fetchHistory = () => fetchJson<RobotSnapshot[]>('/telemetry/history')
export const fetchSessions = () => fetchJson<ReplaySessionInfo[]>('/telemetry/sessions')
export const fetchSession = (sessionId: string) =>
  fetchJson<RobotSnapshot[]>(`/telemetry/sessions/${sessionId}`)

export const updateRobotSpeed = async (speed: number) => {
  const response = await fetch(resolveUrl('/telemetry/robot/config/speed'), {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ max_linear_speed: speed }),
  })
  if (!response.ok) {
    throw new Error('Failed to update robot speed')
  }
  return response.json()
}

export function connectTelemetryWS(onMessage: (data: any) => void): () => void {
  // Try to use VITE_TELEMETRY_BASE or default to current host
  const baseUrl = TELEMETRY_BASE || window.location.origin
  // Replace http/https with ws/wss
  const wsUrl = baseUrl.replace(/^http/, 'ws') + '/telemetry/stream'

  let ws: WebSocket | null = null
  let isClosed = false

  const connect = () => {
    if (isClosed) return
    ws = new WebSocket(wsUrl)

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        onMessage(data)
      } catch (err) {
        console.error('Failed to parse telemetry WS message', err)
      }
    }

    ws.onclose = () => {
      if (!isClosed) {
        setTimeout(connect, 2000) // Reconnect after 2 seconds
      }
    }

    ws.onerror = (err) => {
      console.error('Telemetry WS error', err)
      ws?.close()
    }
  }

  connect()

  return () => {
    isClosed = true
    ws?.close()
  }
}
