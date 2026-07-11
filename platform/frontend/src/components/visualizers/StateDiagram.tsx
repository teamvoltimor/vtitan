import type { StringMsg } from '../../types';
import { RobotState } from '../../types';
import { COLORS } from '../../config';

const STATES = [
  { name: RobotState.BOOT_CHECK, x: 50, y: 30 },
  { name: RobotState.READY, x: 150, y: 30 },
  { name: RobotState.RACING, x: 250, y: 30 },
  { name: RobotState.FINISHED, x: 350, y: 30 },
];
const TRANSITIONS = [
  [90, 120],
  [190, 220],
  [290, 320],
];

/** Robot state-machine diagram, highlighting the current state. */
export function StateDiagram({ data }: { data: StringMsg }) {
  const currentState = data?.data || 'UNKNOWN';
  const muted = 'rgba(255,255,255,0.3)';

  return (
    <div className="state-diagram specialized-viz">
      <svg width="400" height="80" viewBox="0 0 400 80">
        <title>Robot state: {currentState}</title>
        {TRANSITIONS.map(([x1, x2]) => (
          <line
            key={x1}
            x1={x1}
            y1="30"
            x2={x2}
            y2="30"
            stroke={muted}
            strokeWidth="2"
            markerEnd="url(#arrow)"
          />
        ))}

        {STATES.map((state) => {
          const active = currentState === state.name;
          return (
            <g key={state.name}>
              <circle
                cx={state.x}
                cy={state.y}
                r="20"
                fill={active ? COLORS.SUCCESS : 'rgba(255,255,255,0.1)'}
                stroke={active ? COLORS.SUCCESS : muted}
                strokeWidth="2"
              />
              <text
                x={state.x}
                y={state.y + 35}
                textAnchor="middle"
                fill={active ? COLORS.SUCCESS : 'rgba(255,255,255,0.7)'}
                fontSize="10"
                fontWeight={active ? 'bold' : 'normal'}
              >
                {state.name}
              </text>
            </g>
          );
        })}

        <defs>
          <marker id="arrow" markerWidth="10" markerHeight="10" refX="5" refY="5" orient="auto">
            <polygon points="0,0 10,5 0,10" fill={muted} />
          </marker>
        </defs>
      </svg>
    </div>
  );
}
