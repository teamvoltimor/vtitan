import { useEffect, useRef, useState } from 'react';
import { SPEED_CONTROL_CONFIG, SPEED_CONTROL_DEBOUNCE_MS, UI_STRINGS } from '../../config';
import { useTelemetry } from '../../contexts/telemetryState';
import { NodeHealth } from '../../types';
import { formatNumber } from '../../utils/formatting';
import { Label } from '../ui';

/**
 * The value shown here is the *configured* max-speed setting the slider is
 * being dragged to — not `snapshot.metrics.speed` (the robot's current
 * measured speed, already shown separately in the Sidebar telemetry list).
 * Conflating the two previously made this control's readout lie about what
 * it was controlling.
 */
export function SpeedControl() {
  const { snapshot, liveMode, updateSpeed } = useTelemetry();
  const [value, setValue] = useState<number>(SPEED_CONTROL_CONFIG.DEFAULT);
  const [writeError, setWriteError] = useState<string | null>(null);
  const lastConfirmed = useRef<number>(SPEED_CONTROL_CONFIG.DEFAULT);
  const debounceHandle = useRef<number | null>(null);

  useEffect(() => {
    return () => {
      if (debounceHandle.current != null) window.clearTimeout(debounceHandle.current);
    };
  }, []);

  if (!snapshot) return null;

  const commit = (next: number) => {
    updateSpeed(next)
      .then(() => {
        lastConfirmed.current = next;
        setWriteError(null);
      })
      .catch(() => {
        // Backend rejected/failed the write — roll the slider back so it
        // doesn't keep showing a value the robot never actually applied.
        setValue(lastConfirmed.current);
        setWriteError('Failed to update speed — reverted');
      });
  };

  const handleSpeedChange = (raw: string) => {
    const next = Number.parseFloat(raw);
    setValue(next);
    setWriteError(null);
    if (debounceHandle.current != null) window.clearTimeout(debounceHandle.current);
    debounceHandle.current = window.setTimeout(() => commit(next), SPEED_CONTROL_DEBOUNCE_MS);
  };

  return (
    <div className="speed-control">
      <Label>{UI_STRINGS.SPEED_CONTROL}</Label>
      <div className="control-row">
        <input
          type="range"
          aria-label={UI_STRINGS.SPEED_CONTROL}
          min={SPEED_CONTROL_CONFIG.MIN}
          max={SPEED_CONTROL_CONFIG.MAX}
          step={SPEED_CONTROL_CONFIG.STEP}
          disabled={!liveMode || snapshot.metrics.node_health !== NodeHealth.NOMINAL}
          value={value}
          onChange={(e) => handleSpeedChange(e.target.value)}
          style={{ flex: 1 }}
        />
        <span style={{ minWidth: '60px' }}>
          {formatNumber(value, { decimals: 2, unit: 'm/s' })}
        </span>
      </div>
      {writeError && <p className="speed-control-error">{writeError}</p>}
    </div>
  );
}
