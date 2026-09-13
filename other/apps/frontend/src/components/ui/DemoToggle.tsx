import { useTelemetry } from '../../contexts/telemetryState';

export function DemoToggle() {
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
