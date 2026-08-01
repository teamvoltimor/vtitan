import { useState } from 'react';
import { useTelemetry } from '../../contexts/telemetryState';
import { UI_STRINGS } from '../../config';
import { Label } from '../ui';

/**
 * One-way disable button for the robot's gRPC command channel, wired
 * through the backend's DISABLE_COMMAND_CHANNEL command. Unlike
 * TelemetryChannelControl this isn't a toggle: disabling the command
 * channel closes the only path a remote re-enable command could travel
 * over, so once the call succeeds there's no way for this frontend to ever
 * learn if/when it comes back — the control just shows a terminal disabled
 * notice until the page is reloaded. Two-click confirm (no confirm-dialog
 * component exists elsewhere in this codebase to reuse).
 */
export function CommandChannelControl() {
  const { disableCommandChannel, liveMode } = useTelemetry();
  const [confirming, setConfirming] = useState(false);
  const [disabled, setDisabled] = useState(false);
  const [pending, setPending] = useState(false);
  const [writeError, setWriteError] = useState<string | null>(null);

  const handleDisable = () => {
    if (!confirming) {
      setConfirming(true);
      return;
    }
    setPending(true);
    setWriteError(null);
    disableCommandChannel()
      .then(() => setDisabled(true))
      .catch(() => setWriteError('Failed to disable command channel'))
      .finally(() => {
        setPending(false);
        setConfirming(false);
      });
  };

  return (
    <div className="command-channel-control">
      <Label>{UI_STRINGS.COMMAND_CHANNEL}</Label>
      {disabled ? (
        <p className="command-channel-control-notice">
          Command channel disabled — re-enabling requires robot-side access
        </p>
      ) : (
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px', paddingBottom: '4px' }}>
          <button type="button" disabled={!liveMode || pending} onClick={handleDisable}>
            {confirming ? 'Confirm disable' : 'Disable'}
          </button>
          {confirming && (
            <button type="button" disabled={pending} onClick={() => setConfirming(false)}>
              Cancel
            </button>
          )}
        </div>
      )}
      {writeError && <p className="command-channel-control-error">{writeError}</p>}
    </div>
  );
}
