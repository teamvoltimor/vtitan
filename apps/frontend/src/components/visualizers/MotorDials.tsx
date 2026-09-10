import { COLORS, MOTOR_DIALS_CONFIG } from '../../config';
import type { JointStateMsg } from '../../types';
import { formatNumber, radiansToDegrees } from '../../utils/formatting';

const { CIRCLE_CENTER: C, CIRCLE_RADIUS: R } = MOTOR_DIALS_CONFIG;

/**
 * Angle convention shared by the arc and the needle: 0 rad points to 12
 * o'clock, increasing clockwise. Both must use this exact mapping or they
 * visually disagree for the same value.
 */
function dialAngle(pos: number): number {
  return pos - Math.PI / 2;
}

function pointOnDial(angle: number, radius: number): { x: number; y: number } {
  return { x: C.x + radius * Math.cos(angle), y: C.y + radius * Math.sin(angle) };
}

/** Radial dial arc path, sweeping clockwise from the 12 o'clock zero reference to `value`. */
function arcPath(value: number): string {
  let normalized = value % (Math.PI * 2);
  if (normalized < 0) normalized += Math.PI * 2; // [0, 2π)

  const start = pointOnDial(dialAngle(0), R);
  const end = pointOnDial(dialAngle(normalized), R);
  const largeArc = normalized > Math.PI ? 1 : 0;

  return `M ${start.x} ${start.y} A ${R} ${R} 0 ${largeArc} 1 ${end.x} ${end.y}`;
}

/** Per-joint radial dials, driven by a JointState message. */
export function MotorDials({ data }: { data: JointStateMsg }) {
  if (!data?.name || !data?.position) return null;
  const cfg = MOTOR_DIALS_CONFIG;

  return (
    <div className="motor-dials specialized-viz">
      {data.name.map((name, index) => {
        const pos = data.position[index];
        // Ragged JointState (names/positions of different lengths) — skip rather
        // than draw a NaN path.
        if (pos == null || Number.isNaN(pos)) return null;
        const needle = pointOnDial(dialAngle(pos), cfg.NEEDLE_LENGTH);
        return (
          <div key={name} className="motor-dial">
            <svg width={cfg.CANVAS_WIDTH} height={cfg.CANVAS_HEIGHT} viewBox={cfg.VIEWBOX}>
              <title>
                {name}: {formatNumber(radiansToDegrees(pos), { decimals: 0, unit: '°' })}
              </title>
              <circle cx={C.x} cy={C.y} r={R} fill="rgba(255,255,255,0.05)" />
              <path
                d={arcPath(pos)}
                fill="none"
                stroke={COLORS.HIGHLIGHT}
                strokeWidth={cfg.ARC_STROKE_WIDTH}
                strokeLinecap="round"
              />
              <line
                x1={C.x}
                y1={C.y}
                x2={needle.x}
                y2={needle.y}
                stroke={COLORS.WHITE}
                strokeWidth={cfg.NEEDLE_STROKE}
              />
              <circle cx={C.x} cy={C.y} r={cfg.HUB_RADIUS} fill={COLORS.WHITE} />
              <text x={C.x} y={cfg.LABEL_Y} textAnchor="middle" fill={COLORS.WHITE} fontSize="12">
                {formatNumber(radiansToDegrees(pos), { decimals: 0, unit: '°' })}
              </text>
            </svg>

            <div className="motor-info">
              <strong>{name}</strong>
              <span>
                Vel: {formatNumber(data.velocity?.[index], { decimals: 2, unit: 'rad/s' })}
              </span>
              <span>Eff: {formatNumber(data.effort?.[index], { decimals: 2, unit: 'Nm' })}</span>
            </div>
          </div>
        );
      })}
    </div>
  );
}
