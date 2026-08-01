import { useState } from 'react';
import { UI_STRINGS } from '../../config';
import { useTelemetry } from '../../contexts/telemetryState';
import { ChannelToggle } from '../ui';

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
    <ChannelToggle
      label={UI_STRINGS.TELEMETRY_CHANNEL}
      enabled={enabled}
      pending={pending}
      disabled={!liveMode}
      error={writeError}
      onToggle={handleToggle}
    />
  );
}
