import { useTelemetry } from '../../contexts/telemetryState';
import { SENSOR_CONFIG, UI_STRINGS } from '../../config';
import { Label } from '../ui';

export function SensorHealthPanel() {
  const { snapshot } = useTelemetry();
  if (!snapshot) return null;
  const metrics = snapshot.metrics;

  return (
    <div className="sensor-health">
      <Label>{UI_STRINGS.SENSOR_STATUS}</Label>
      <div className="sensor-grid">
        {SENSOR_CONFIG.map(({ id, name, key }) => {
          const available = metrics[key];
          return (
            <div key={id} className={`sensor-item ${available ? 'online' : 'offline'}`}>
              <span className="sensor-dot" />
              <div className="sensor-info">
                <strong>{name}</strong>
                <span className={available ? 'status-ok' : 'status-error'}>
                  {available ? 'ONLINE' : 'OFFLINE'}
                </span>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
