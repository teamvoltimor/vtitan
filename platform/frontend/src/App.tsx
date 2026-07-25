import './App.css';

import { TelemetryProvider } from './contexts/TelemetryContext';
import { useTelemetry } from './contexts/telemetryState';
import { TelemetryScene } from './components/scene/TelemetryScene';
import { Sidebar } from './components/Sidebar';
import { DemoToggle } from './components/ui/DemoToggle';
import { InspectorPanel } from './components/InspectorPanel';
import { AsyncState } from './components/ui';

function AppContent() {
  const { snapshot, topics, loading, error, retry, demoMode, setDemoMode } = useTelemetry();

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
        <InspectorPanel topics={topics} />
      </div>
    </AsyncState>
  );
}

function App() {
  return (
    <TelemetryProvider>
      <AppContent />
    </TelemetryProvider>
  );
}

export default App;
