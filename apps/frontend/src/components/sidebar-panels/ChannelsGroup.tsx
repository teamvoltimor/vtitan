/**
 * src/components/sidebar-panels/ChannelsGroup.tsx
 *
 * Groups the three remote robot-channel controls (vision debug, telemetry
 * channel, command channel) under a single shared label. They were previously
 * three identically-chromed stacked panels, each with its own header —
 * redundant vertical sprawl for three related toggles (audit §4.4).
 */

import { Label } from '../ui';
import { CommandChannelControl } from './CommandChannelControl';
import { TelemetryChannelControl } from './TelemetryChannelControl';
import { VisionDebugControl } from './VisionDebugControl';

export function ChannelsGroup() {
  return (
    <fieldset className="channels-group">
      <legend>
        <Label>Channels</Label>
      </legend>
      <VisionDebugControl />
      <TelemetryChannelControl />
      <CommandChannelControl showLabel={false} />
    </fieldset>
  );
}
