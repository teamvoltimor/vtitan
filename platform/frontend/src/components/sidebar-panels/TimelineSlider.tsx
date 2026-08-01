import { UI_STRINGS } from '../../config';
import { useTelemetry } from '../../contexts/telemetryState';
import { formatTimestamp } from '../../utils/formatting';

export function TimelineSlider() {
  const { history, timelineIndex, liveMode, selectedSessionId, setTimelineIndex, goLive } =
    useTelemetry();

  if (history.length === 0) return null;

  const timelineLabel = liveMode
    ? 'Live timeline'
    : selectedSessionId
      ? 'Replay timeline'
      : 'Timeline';
  const timelineLength = history.length;
  const timelineTimestamp = history[timelineIndex]?.timestamp ?? '';

  return (
    <div className="timeline">
      <div className="timeline-header">
        <p>{timelineLabel}</p>
        <button className="timeline-button" type="button" onClick={goLive} disabled={liveMode}>
          {liveMode ? UI_STRINGS.LIVE : UI_STRINGS.GO_LIVE}
        </button>
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
