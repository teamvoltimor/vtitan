/**
 * src/contexts/TelemetryContext.tsx
 * 
 * Global state management for telemetry data.
 * Eliminates prop drilling and centralizes all telemetry-related state and actions.
 * 
 * Follows SOLID principles:
 * - Single Responsibility: Manages only telemetry state and actions
 * - Open/Closed: Extensible through additional actions
 * - Dependency Inversion: Components depend on context interface, not implementation
 */

import {
  createContext,
  useContext,
  useState,
  useCallback,
  useEffect,
  useRef,
  type ReactNode,
} from 'react'
import type { RobotSnapshot, TopicsSnapshot, ReplaySessionInfo } from '../types'
import {
  fetchLatestTelemetry,
  fetchRawTopics,
  fetchHistory,
  fetchSessions,
  fetchSession,
  connectTelemetryWS,
} from '../api/telemetry'
import { getErrorMessage } from '../utils/formatting'
import { TELEMETRY_CONFIG } from '../config'

/**
 * Telemetry context interface.
 * Provides both state and actions for managing telemetry data.
 */
export interface TelemetryContextType {
  // State: Robot telemetry
  snapshot: RobotSnapshot | null
  topics: TopicsSnapshot | null
  history: RobotSnapshot[]
  sessions: ReplaySessionInfo[]

  // State: UI/Control
  selectedSessionId: string | null
  timelineIndex: number
  liveMode: boolean

  // State: Loading/Error
  loading: boolean
  error: string | null

  // Actions: Data loading
  loadInitialData: () => Promise<void>
  retry: () => void

  // Actions: Timeline/Replay control
  setTimelineIndex: (index: number) => void
  loadSession: (sessionId: string) => Promise<void>

  // Actions: Live mode
  startLiveMode: () => void
  stopLiveMode: () => void
  goLive: () => void
}

/**
 * Create the context (private to module).
 * Use useTelemetry() hook instead of importing directly.
 */
const TelemetryContext = createContext<TelemetryContextType | null>(null)

/**
 * Provider component that wraps the application.
 * Must wrap any component that uses useTelemetry() hook.
 * 
 * @example
 * <TelemetryProvider>
 *   <App />
 * </TelemetryProvider>
 */
export function TelemetryProvider({ children }: { children: ReactNode }) {
  // ========================================================================
  // STATE
  // ========================================================================

  // Robot telemetry state
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null)
  const [topics, setTopics] = useState<TopicsSnapshot | null>(null)
  const [history, setHistory] = useState<RobotSnapshot[]>([])
  const [sessions, setSessions] = useState<ReplaySessionInfo[]>([])

  // UI/Control state
  const [selectedSessionId, setSelectedSessionId] = useState<string | null>(null)
  const [timelineIndex, setTimelineIndex] = useState(0)
  const [liveMode, setLiveMode] = useState(true)

  // Loading/Error state
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  // Retry counter to trigger refetch
  const [retryCount, setRetryCount] = useState(0)

  // Track mounted status to prevent state updates after unmount
  const mounted = useRef(true)

  // Track active WebSocket connection
  const disconnectWS = useRef<(() => void) | null>(null)

  // ========================================================================
  // LIFECYCLE
  // ========================================================================

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mounted.current = false
      disconnectWS.current?.()
    }
  }, [])

  // ========================================================================
  // ACTIONS: DATA LOADING
  // ========================================================================

  /**
   * Load initial telemetry data from API.
   * Fetches latest snapshot, topics, history, and available sessions.
   */
  const loadInitialData = useCallback(async () => {
    // Guard clause: component unmounted
    if (!mounted.current) return

    setLoading(true)
    setError(null)

    try {
      // Fetch all data in parallel
      const [latest, rawTopics, historyData, sessionsList] = await Promise.all([
        fetchLatestTelemetry(),
        fetchRawTopics(),
        fetchHistory(),
        fetchSessions(),
      ])

      // Guard clause: component unmounted while fetching
      if (!mounted.current) return

      // Update state
      setSnapshot(latest)
      setTopics(rawTopics)
      setHistory(historyData)
      setSessions(sessionsList)
      setError(null)
      setTimelineIndex(0)
    } catch (err) {
      // Guard clause: component unmounted while handling error
      if (!mounted.current) return

      const message = getErrorMessage(
        err,
        'Failed to load telemetry data'
      )
      setError(message)
    } finally {
      if (mounted.current) {
        setLoading(false)
      }
    }
  }, [])

  /**
   * Retry loading initial data (triggered by user or retry button).
   */
  const retry = useCallback(() => {
    setRetryCount((c) => c + 1)
  }, [])

  // Trigger loadInitialData when retry count changes
  useEffect(() => {
    loadInitialData()
  }, [retryCount, loadInitialData])

  // ========================================================================
  // ACTIONS: TIMELINE/REPLAY CONTROL
  // ========================================================================

  /**
   * Load a specific replay session by ID.
   * Stops live mode and switches to replay.
   */
  const loadSession = useCallback(async (sessionId: string) => {
    // Guard clause: component unmounted
    if (!mounted.current) return

    setError(null)

    try {
      const sessionSnapshots = await fetchSession(sessionId)

      // Guard clause: component unmounted while fetching
      if (!mounted.current) return

      // Update state for replay
      setHistory(sessionSnapshots)
      setTimelineIndex(sessionSnapshots.length - 1)
      setSelectedSessionId(sessionId)
      setLiveMode(false)

      // Update snapshot to last frame of session
      if (sessionSnapshots.length > 0) {
        setSnapshot(sessionSnapshots[sessionSnapshots.length - 1])
      }
    } catch (err) {
      // Guard clause: component unmounted while handling error
      if (!mounted.current) return

      const message = getErrorMessage(
        err,
        `Failed to load session ${sessionId}`
      )
      setError(message)
    }
  }, [])

  // ========================================================================
  // ACTIONS: LIVE MODE CONTROL
  // ========================================================================

  /**
   * Switch to live mode and start WebSocket connection.
   */
  const startLiveMode = useCallback(() => {
    // Guard clause: already in live mode
    if (liveMode) return

    setLiveMode(true)
    setSelectedSessionId(null)
    setError(null)
  }, [liveMode])

  /**
   * Stop live mode and disconnect WebSocket.
   */
  const stopLiveMode = useCallback(() => {
    disconnectWS.current?.()
    setLiveMode(false)
  }, [])

  /**
   * Convenience method to switch to live mode and clear replay state.
   */
  const goLive = useCallback(() => {
    startLiveMode()
    setSelectedSessionId(null)
    setTimelineIndex(0)
  }, [startLiveMode])

  // ========================================================================
  // WEBSOCKET CONNECTION
  // ========================================================================

  /**
   * Manage WebSocket connection for live telemetry.
   * Connects when liveMode is true, disconnects when false.
   */
  useEffect(() => {
    // Guard clause: not in live mode
    if (!liveMode) {
      disconnectWS.current?.()
      disconnectWS.current = null
      return
    }

    // Guard clause: component unmounted
    if (!mounted.current) return

    // Connect to WebSocket
    try {
      disconnectWS.current = connectTelemetryWS(
        (data: RobotSnapshot | TopicsSnapshot) => {
          // Guard clause: component unmounted
          if (!mounted.current) return

          // Update state based on message type
          if ('robotPosition' in data) {
            // RobotSnapshot
            const snapshot = data as RobotSnapshot

            setSnapshot(snapshot)

            // Add to history (keep limited size)
            setHistory((prev) => {
              const updated = [...prev, snapshot]
              // Keep last HISTORY_MAX_SIZE snapshots
              if (updated.length > TELEMETRY_CONFIG.HISTORY_MAX_SIZE) {
                return updated.slice(-TELEMETRY_CONFIG.HISTORY_MAX_SIZE)
              }
              return updated
            })

            // Reset timeline index to end (showing latest)
            setTimelineIndex((prev) => prev + 1)
          } else if ('topics' in data) {
            // TopicsSnapshot
            setTopics(data as TopicsSnapshot)
          }
        }
      )
    } catch (err) {
      // Guard clause: component unmounted
      if (!mounted.current) return

      const message = getErrorMessage(
        err,
        'Failed to connect to live telemetry'
      )
      setError(message)
      setLiveMode(false)
    }

    // Cleanup: disconnect on unmount or live mode change
    return () => {
      disconnectWS.current?.()
      disconnectWS.current = null
    }
  }, [liveMode])

  // ========================================================================
  // CONTEXT VALUE
  // ========================================================================

  const value: TelemetryContextType = {
    // State
    snapshot,
    topics,
    history,
    sessions,
    selectedSessionId,
    timelineIndex,
    liveMode,
    loading,
    error,

    // Actions
    loadInitialData,
    retry,
    setTimelineIndex,
    loadSession,
    startLiveMode,
    stopLiveMode,
    goLive,
  }

  return (
    <TelemetryContext.Provider value={value}>
      {children}
    </TelemetryContext.Provider>
  )
}

// ============================================================================
// CUSTOM HOOK
// ============================================================================

/**
 * Hook to use telemetry context.
 * Must be called within a TelemetryProvider.
 * 
 * @throws Error if used outside TelemetryProvider
 * 
 * @example
 * function MyComponent() {
 *   const { snapshot, history, liveMode } = useTelemetry()
 *   return <div>...</div>
 * }
 */
export function useTelemetry(): TelemetryContextType {
  const context = useContext(TelemetryContext)

  if (!context) {
    throw new Error('useTelemetry must be used within <TelemetryProvider>')
  }

  return context
}
