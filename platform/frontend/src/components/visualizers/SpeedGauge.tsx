import type { TwistMsg } from '../../types'
import { formatNumber } from '../../utils/formatting'
import { SPEED_GAUGE_CONFIG } from '../../config'
import { Gauge } from '../ui'

/** Linear-speed gauge plus a steering-rate indicator, driven by a Twist. */
export function SpeedGauge({ data }: { data: TwistMsg }) {
  const linearSpeed = data?.linear?.x || 0
  const angularSpeed = data?.angular?.z || 0

  return (
    <div className="specialized-viz speed-gauge">
      <Gauge value={linearSpeed} max={SPEED_GAUGE_CONFIG.MAX_LINEAR_SPEED} unit="m/s" />

      <div className="steering-indicator">
        <span>Steering Rate</span>
        <div className="steering-bar">
          <div
            className="steering-marker"
            style={{
              left: `${50 + (angularSpeed / SPEED_GAUGE_CONFIG.STEERING_SENSITIVITY) * 50}%`,
            }}
          />
        </div>
        <span>{formatNumber(angularSpeed, { decimals: 2, unit: 'rad/s' })}</span>
      </div>
    </div>
  )
}
