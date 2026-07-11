import type { JointStateMsg } from '../../types';
import { formatNumber, radiansToDegrees } from '../../utils/formatting';
import { MOTOR_DIALS_CONFIG, COLORS } from '../../config';

const { CIRCLE_CENTER: C, CIRCLE_RADIUS: R } = MOTOR_DIALS_CONFIG;

/** Radial dial arc path for an angle (radians) around the dial centre. */
function arcPath(value: number): string {
  let normalized = value % (Math.PI * 2);
  if (normalized < -Math.PI) normalized += Math.PI * 2;
  if (normalized > Math.PI) normalized -= Math.PI * 2;

  const percent = (normalized + Math.PI) / (Math.PI * 2);
  const angle = Math.PI - percent * Math.PI * 2;

  const endX = C.x - R * Math.cos(Math.PI - angle);
  const endY = C.y - R * Math.sin(Math.PI - angle);

  return `M ${C.x} ${C.y + R} A ${R} ${R} 0 ${percent > 0.5 ? 1 : 0} 1 ${endX} ${endY}`;
}

/** Per-joint radial dials, driven by a JointState message. */
export function MotorDials({ data }: { data: JointStateMsg }) {
  if (!data?.name || !data?.position) return null;
  const cfg = MOTOR_DIALS_CONFIG;

  return (
    <div className="motor-dials specialized-viz">
      {data.name.map((name, index) => {
        const pos = data.position[index];
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
                x2={C.x + cfg.NEEDLE_LENGTH * Math.cos(pos - Math.PI / 2)}
                y2={C.y + cfg.NEEDLE_LENGTH * Math.sin(pos - Math.PI / 2)}
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
