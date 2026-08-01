import './App.css';

import { InspectorPanel } from './components/InspectorPanel';
import { Sidebar } from './components/Sidebar';
import { TelemetryScene } from './components/scene/TelemetryScene';
import { AsyncState, ErrorBoundary } from './components/ui';
import { DemoToggle } from './components/ui/DemoToggle';
import { TelemetryProvider } from './contexts/TelemetryContext';
import { useTelemetry } from './contexts/telemetryState';

function AppContent() {
  const {
    snapshot,
    displaySnapshot,
    topics,
    loading,
    error,
    connected,
    retry,
    demoMode,
    setDemoMode,
  } = useTelemetry();

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
        {!connected && (
          <div className="connection-banner" role="status">
            Reconnecting to robot…
          </div>
        )}
        <div className="canvas-wrapper">
          <ErrorBoundary fallback={<div className="scene-error">3D scene failed to render</div>}>
            {displaySnapshot && <TelemetryScene snapshot={displaySnapshot} />}
          </ErrorBoundary>
          <DemoToggle />
        </div>
        <Sidebar />
        <ErrorBoundary
          fallback={<div className="inspector-error">Topic inspector failed to render</div>}
        >
          <InspectorPanel topics={topics} />
        </ErrorBoundary>
      </div>
    </AsyncState>
  );
}

function App() {
  return (
    <ErrorBoundary
      fallback={<div className="app-fatal-error">Something went wrong. Please reload.</div>}
    >
      <TelemetryProvider>
        <AppContent />
      </TelemetryProvider>
    </ErrorBoundary>
  );
}

export default App;
