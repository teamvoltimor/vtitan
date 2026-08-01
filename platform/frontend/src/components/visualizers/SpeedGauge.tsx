import { SPEED_GAUGE_CONFIG } from '../../config';
import type { TwistMsg } from '../../types';
import { clamp, formatNumber } from '../../utils/formatting';
import { Gauge } from '../ui';

/** Linear-speed gauge plus a steering-rate indicator, driven by a Twist. */
export function SpeedGauge({ data }: { data: TwistMsg }) {
  const linearSpeed = data?.linear?.x || 0;
  const angularSpeed = data?.angular?.z || 0;
  // Clamp the marker to the bar — an angular velocity past STEERING_SENSITIVITY
  // previously pushed the marker outside the track entirely.
  const markerLeft = clamp(
    50 + (angularSpeed / SPEED_GAUGE_CONFIG.STEERING_SENSITIVITY) * 50,
    0,
    100
  );

  return (
    <div className="specialized-viz speed-gauge">
      <Gauge value={linearSpeed} max={SPEED_GAUGE_CONFIG.MAX_LINEAR_SPEED} unit="m/s" />

      <div className="steering-indicator">
        <span>Steering Rate</span>
        <div className="steering-bar">
          <div className="steering-marker" style={{ left: `${markerLeft}%` }} />
        </div>
        <span>{formatNumber(angularSpeed, { decimals: 2, unit: 'rad/s' })}</span>
      </div>
    </div>
  );
}
