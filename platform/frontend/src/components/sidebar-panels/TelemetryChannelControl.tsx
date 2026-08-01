import { useState } from 'react';
import { useTelemetry } from '../../contexts/telemetryState';
import { UI_STRINGS } from '../../config';
import { Label } from '../ui';

/**
 * Remote toggle for the robot's telemetry gRPC channel, wired through the
 * backend's SET_TELEMETRY_CHANNEL command (backend↔robot gRPC command
 * channel). Bidirectional — unlike the command channel, disabling this
 * doesn't cut off the path a remote re-enable would need, so it's safe to
 * toggle either way. Starts checked (telemetry is on by default) and
 * reflects only local optimistic state — there's no telemetry field that
 * reports the robot's actual current channel state (would be circular).
 */
export function TelemetryChannelControl() {
  const { setTelemetryChannel, liveMode } = useTelemetry();
  const [enabled, setEnabled] = useState(true);
  const [pending, setPending] = useState(false);
  const [writeError, setWriteError] = useState<string | null>(null);

  const handleToggle = (next: boolean) => {
    setPending(true);
    setWriteError(null);
    setTelemetryChannel({ enabled: next })
      .then(() => setEnabled(next))
      .catch(() => setWriteError('Failed to update telemetry channel'))
      .finally(() => setPending(false));
  };

  return (
    <div className="telemetry-channel-control">
      <Label>{UI_STRINGS.TELEMETRY_CHANNEL}</Label>
      <div style={{ display: 'flex', alignItems: 'center', gap: '10px', paddingBottom: '4px' }}>
        <input
          type="checkbox"
          aria-label={UI_STRINGS.TELEMETRY_CHANNEL}
          checked={enabled}
          disabled={!liveMode || pending}
          onChange={(e) => handleToggle(e.target.checked)}
        />
        <span>{enabled ? 'Enabled' : 'Disabled'}</span>
      </div>
      {writeError && <p className="telemetry-channel-control-error">{writeError}</p>}
    </div>
  );
}
