import './App.css'

import { TelemetryProvider } from './contexts/TelemetryContext'
import { useTelemetry } from './contexts/telemetryState'
import { TelemetryScene } from './components/scene/TelemetryScene'
import { Sidebar } from './components/Sidebar'
import { TopicInspector } from './components/TopicInspector'
import { AsyncState } from './components/ui'

/** Floating control to toggle Demo Mode (mock data, no backend) at any time. */
function DemoToggle() {
  const { demoMode, toggleDemoMode } = useTelemetry()

  return (
    <button
      type="button"
      className={`demo-toggle${demoMode ? ' active' : ''}`}
      onClick={toggleDemoMode}
      aria-pressed={demoMode}
      title={demoMode ? 'Switch to live backend' : 'Visualise with mock data'}
    >
      {demoMode ? '● Demo Mode' : 'Demo Mode'}
    </button>
  )
}

/** Main app content: gates on async state, then composes the layout. */
function AppContent() {
  const { snapshot, topics, loading, error, retry, demoMode, setDemoMode } = useTelemetry()

  return (
    <AsyncState
      loading={loading || !snapshot}
      error={error}
      onRetry={retry}
      errorActions={
        !demoMode && (
          <button type="button" className="demo-launch" onClick={() => setDemoMode(true)}>
            Launch Demo Mode
          </button>
        )
      }
    >
      <div className="shell">
        <div className="canvas-wrapper">
          {snapshot && <TelemetryScene snapshot={snapshot} />}
          <DemoToggle />
        </div>
        <Sidebar />
        <div className="inspector-panel">
          <TopicInspector topics={topics} />
        </div>
      </div>
    </AsyncState>
  )
}

function App() {
  return (
    <TelemetryProvider>
      <AppContent />
    </TelemetryProvider>
  )
}

export default App
