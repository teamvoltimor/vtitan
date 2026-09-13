import { useTelemetry } from '../../contexts/telemetryState';
import { formatTimestamp } from '../../utils/formatting';

export function TimelineSlider() {
  const { history, timelineIndex, selectedSessionId, setTimelineIndex } = useTelemetry();

  if (history.length === 0) return null;

  const timelineLabel = selectedSessionId ? 'Replay timeline' : 'Live timeline';
  const timelineLength = history.length;
  const timelineTimestamp = history[timelineIndex]?.timestamp ?? '';

  return (
    <div className="timeline">
      <div className="timeline-header">
        <p>{timelineLabel}</p>
      </div>
      <input
        type="range"
        aria-label="Timeline position"
        min="0"
        max={Math.max(timelineLength - 1, 0).toString()}
        value={timelineIndex}
        onChange={(event) => setTimelineIndex(Number(event.target.value))}
      />
      <div className="timeline-meta">
        <span>
          {timelineIndex + 1}/{Math.max(timelineLength, 1)} · {formatTimestamp(timelineTimestamp)}
        </span>
      </div>
    </div>
  );
}
