import type { ReplaySessionInfo, SimulationSnapshot } from '../types'

const TELEMETRY_BASE = (import.meta.env.VITE_TELEMETRY_BASE ?? '').replace(/\/$/, '')

const resolveUrl = (path: string) => (TELEMETRY_BASE ? `${TELEMETRY_BASE}${path}` : path)

async function fetchJson<T>(path: string): Promise<T> {
  const response = await fetch(resolveUrl(path), { cache: 'no-store' })
  if (!response.ok) {
    throw new Error(`Failed to load ${path}`)
  }
  return response.json()
}

export const fetchLatestTelemetry = () => fetchJson<SimulationSnapshot>('/telemetry/latest')
export const fetchHistory = () => fetchJson<SimulationSnapshot[]>('/telemetry/history')
export const fetchSessions = () => fetchJson<ReplaySessionInfo[]>('/telemetry/sessions')
export const fetchSession = (sessionId: string) =>
  fetchJson<SimulationSnapshot[]>(`/telemetry/sessions/${sessionId}`)
