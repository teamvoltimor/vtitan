import { NodeHealth } from '../../types';
import { useTelemetry } from '../../contexts/telemetryState';
import { formatNumber } from '../../utils/formatting';
import { SPEED_CONTROL_CONFIG, UI_STRINGS } from '../../config';
import { Label } from '../ui';

export function SpeedControl() {
  const { snapshot, liveMode, updateSpeed } = useTelemetry();
  if (!snapshot) return null;

  const handleSpeedChange = (value: string) => {
    updateSpeed(parseFloat(value));
  };

  return (
    <div className="speed-control">
      <Label>{UI_STRINGS.SPEED_CONTROL}</Label>
      <div style={{ display: 'flex', alignItems: 'center', gap: '10px', paddingBottom: '16px' }}>
        <input
          type="range"
          aria-label={UI_STRINGS.SPEED_CONTROL}
          min={SPEED_CONTROL_CONFIG.MIN}
          max={SPEED_CONTROL_CONFIG.MAX}
          step={SPEED_CONTROL_CONFIG.STEP}
          disabled={!liveMode || snapshot.metrics.node_health !== NodeHealth.NOMINAL}
          defaultValue={SPEED_CONTROL_CONFIG.DEFAULT}
          onChange={(e) => handleSpeedChange(e.target.value)}
          style={{ flex: 1 }}
        />
        <span style={{ minWidth: '60px' }}>
          {formatNumber(snapshot.metrics.speed, { decimals: 2, unit: 'm/s' })}
        </span>
      </div>
    </div>
  );
}
