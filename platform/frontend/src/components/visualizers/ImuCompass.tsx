import type { ImuMsg, Quaternion } from '../../types';
import { formatNumber, radiansToDegrees } from '../../utils/formatting';
import { IMU_METRICS_CONFIG } from '../../config';
import { BarChart } from '../ui';

function quaternionToEuler(q: Quaternion) {
  const { x, y, z, w } = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

type Axis = 'x' | 'y' | 'z';

/** IMU orientation + per-axis acceleration/gyro bars, driven by an Imu message. */
export function ImuCompass({ data }: { data: ImuMsg }) {
  const { roll, pitch, yaw } = quaternionToEuler(data.orientation);

  return (
    <div className="specialized-viz imu-compass">
      <div className="imu-bars">
        {IMU_METRICS_CONFIG.map((config) => {
          const [group, axis] = config.key.split('.') as [
            'linear_acceleration' | 'angular_velocity',
            Axis,
          ];
          const source =
            group === 'linear_acceleration' ? data.linear_acceleration : data.angular_velocity;
          return (
            <BarChart
              key={config.key}
              label={config.label}
              value={source[axis]}
              range={config.range as [number, number]}
              unit={config.unit}
            />
          );
        })}
      </div>

      <div className="orientation-values">
        <span>Roll: {formatNumber(radiansToDegrees(roll), { decimals: 1, unit: '°' })}</span>
        <span>Pitch: {formatNumber(radiansToDegrees(pitch), { decimals: 1, unit: '°' })}</span>
        <span>Yaw: {formatNumber(radiansToDegrees(yaw), { decimals: 1, unit: '°' })}</span>
      </div>
    </div>
  );
}
