import { useState } from 'react';
import { UI_STRINGS } from '../../config';
import { useTelemetry } from '../../contexts/telemetryState';
import { ChannelToggle } from '../ui';

/**
 * Remote toggle for the robot's vision debug annotated-image stream, wired
 * through the backend's SET_VISION_DEBUG command (backend↔robot gRPC command
 * channel). Disabled by default on the robot, so this starts unchecked and
 * reflects only local optimistic state — there's no telemetry field yet that
 * reports the robot's actual current debug-stream state.
 */
export function VisionDebugControl() {
  const { setVisionDebug, liveMode } = useTelemetry();
  const [enabled, setEnabled] = useState(false);
  const [pending, setPending] = useState(false);
  const [writeError, setWriteError] = useState<string | null>(null);

  const handleToggle = (next: boolean) => {
    setPending(true);
    setWriteError(null);
    setVisionDebug({ enabled: next })
      .then(() => setEnabled(next))
      .catch(() => setWriteError('Failed to update vision debug stream'))
      .finally(() => setPending(false));
  };

  return (
    <ChannelToggle
      label={UI_STRINGS.VISION_DEBUG}
      enabled={enabled}
      pending={pending}
      disabled={!liveMode}
      error={writeError}
      showLabel={false}
      onToggle={handleToggle}
    />
  );
}
