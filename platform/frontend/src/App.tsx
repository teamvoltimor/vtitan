import './App.css';

import { useRef, useState, useEffect } from 'react';
import { TelemetryProvider } from './contexts/TelemetryContext';
import { useTelemetry } from './contexts/telemetryState';
import { TelemetryScene } from './components/scene/TelemetryScene';
import { Sidebar } from './components/Sidebar';
import { TopicInspector } from './components/TopicInspector';
import { AsyncState } from './components/ui';

const INSPECTOR_MIN_HEIGHT = 150;
const INSPECTOR_MAX_RATIO = 0.85;

/** Floating control to toggle Demo Mode (mock data, no backend) at any time. */
function DemoToggle() {
  const { demoMode, toggleDemoMode } = useTelemetry();

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
  );
}

/** Main app content: gates on async state, then composes the layout. */
function AppContent() {
  const { snapshot, topics, loading, error, retry, demoMode, setDemoMode } = useTelemetry();
  const [inspectorHeight, setInspectorHeight] = useState(300);
  const isResizing = useRef(false);
  const startY = useRef(0);
  const startHeight = useRef(0);

  const onResizeStart = (e: React.MouseEvent) => {
    isResizing.current = true;
    startY.current = e.clientY;
    startHeight.current = inspectorHeight;
    e.preventDefault();
  };

  const onResizeKeyDown = (e: React.KeyboardEvent) => {
    const step = 20;
    const max = window.innerHeight * INSPECTOR_MAX_RATIO;
    if (e.key === 'ArrowUp') {
      setInspectorHeight((h) => Math.min(max, h + step));
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      setInspectorHeight((h) => Math.max(INSPECTOR_MIN_HEIGHT, h - step));
      e.preventDefault();
    }
  };

  useEffect(() => {
    const onMove = (e: MouseEvent) => {
      if (!isResizing.current) return;
      const delta = startY.current - e.clientY;
      const max = window.innerHeight * INSPECTOR_MAX_RATIO;
      setInspectorHeight(
        Math.max(INSPECTOR_MIN_HEIGHT, Math.min(max, startHeight.current + delta))
      );
    };
    const onUp = () => {
      isResizing.current = false;
    };
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
    return () => {
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    };
  }, []);

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
        <div className="inspector-panel" style={{ height: inspectorHeight }}>
          {/* biome-ignore lint/a11y/useSemanticElements: this is a draggable/keyboard-resizable handle, not a static <hr> divider */}
          <div
            className="inspector-resize-handle"
            onMouseDown={onResizeStart}
            onKeyDown={onResizeKeyDown}
            aria-label="Drag to resize"
            role="separator"
            aria-orientation="horizontal"
            aria-valuenow={inspectorHeight}
            aria-valuemin={INSPECTOR_MIN_HEIGHT}
            aria-valuemax={Math.round(window.innerHeight * INSPECTOR_MAX_RATIO)}
            tabIndex={0}
          />
          <TopicInspector topics={topics} />
        </div>
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
