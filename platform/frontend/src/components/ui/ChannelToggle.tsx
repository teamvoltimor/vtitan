import { Label } from './Label';

interface ChannelToggleProps {
  label: string;
  enabled: boolean;
  /** In-flight command — disables the control while the backend call resolves. */
  pending?: boolean;
  /** Disable the control entirely (e.g. not in live mode). */
  disabled?: boolean;
  /** Backend write failure message, rendered as an inline error. */
  error?: string | null;
  onToggle: (next: boolean) => void;
}

/**
 * Checkbox toggle with an enabled/disabled readout and inline error. Shared by
 * the vision-debug and telemetry-channel controls — they were previously the
 * same component modulo strings (audit §10.2).
 */
export function ChannelToggle({
  label,
  enabled,
  pending = false,
  disabled = false,
  error,
  onToggle,
}: ChannelToggleProps) {
  return (
    <div className="channel-toggle">
      <Label>{label}</Label>
      <div className="control-row">
        <input
          type="checkbox"
          aria-label={label}
          checked={enabled}
          disabled={disabled || pending}
          onChange={(e) => onToggle(e.target.checked)}
        />
        <span>{enabled ? 'Enabled' : 'Disabled'}</span>
      </div>
      {error && <p className="channel-toggle-error">{error}</p>}
    </div>
  );
}
