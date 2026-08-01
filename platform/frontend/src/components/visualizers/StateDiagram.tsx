import { useId } from 'react';
import { COLORS } from '../../config';
import type { StringMsg } from '../../types';
import { RobotState } from '../../types';

const STATE_NAMES = Object.values(RobotState);
const NODE_SPACING = 100;
const NODE_Y = 30;
const NODE_RADIUS = 20;
const DIAGRAM_HEIGHT = 80;
const DIAGRAM_WIDTH = NODE_SPACING * STATE_NAMES.length + 50;

const STATES = STATE_NAMES.map((name, i) => ({ name, x: 50 + i * NODE_SPACING, y: NODE_Y }));

/** Robot state-machine diagram, highlighting the current state. */
export function StateDiagram({ data }: { data: StringMsg }) {
  // The wire carries lowercase enum values (state_machine_node publishes
  // current_state.value). Normalise at the boundary so any casing mismatch
  // can never mute the whole diagram — audit §3.3.
  const currentState = (data?.data || 'unknown').toLowerCase();
  const isKnownState = (STATE_NAMES as readonly string[]).includes(currentState);
  const muted = 'rgba(255,255,255,0.3)';
  // Unique per instance — the inspector and the "expand to full view" modal
  // can render this diagram simultaneously, and a duplicate DOM id makes
  // marker resolution undefined per the SVG spec.
  const arrowId = useId();

  return (
    <div className="state-diagram specialized-viz">
      <svg
        width={DIAGRAM_WIDTH}
        height={DIAGRAM_HEIGHT}
        viewBox={`0 0 ${DIAGRAM_WIDTH} ${DIAGRAM_HEIGHT}`}
      >
        <title>Robot state: {currentState}</title>
        {STATES.slice(0, -1).map((state, i) => (
          <line
            key={state.name}
            x1={state.x + NODE_RADIUS}
            y1={NODE_Y}
            x2={STATES[i + 1].x - NODE_RADIUS}
            y2={NODE_Y}
            stroke={muted}
            strokeWidth="2"
            markerEnd={`url(#${arrowId})`}
          />
        ))}

        {STATES.map((state) => {
          const active = currentState === state.name;
          return (
            <g key={state.name}>
              <circle
                cx={state.x}
                cy={state.y}
                r={NODE_RADIUS}
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
          <marker id={arrowId} markerWidth="10" markerHeight="10" refX="5" refY="5" orient="auto">
            <polygon points="0,0 10,5 0,10" fill={muted} />
          </marker>
        </defs>
      </svg>
      {!isKnownState && <p className="state-diagram-unknown">Unrecognized state: {currentState}</p>}
    </div>
  );
}
