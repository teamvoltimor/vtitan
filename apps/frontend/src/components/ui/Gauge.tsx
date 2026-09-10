import { COLORS, SPEED_GAUGE_CONFIG } from '../../config';
import { formatNumber } from '../../utils/formatting';
import { calculateGaugePath, calculateNeedle } from '../../utils/gauges';

interface GaugeProps {
  value: number;
  max: number;
  unit?: string;
  decimals?: number;
}

/**
 * Semicircular value gauge (background arc + filled value arc + needle + hub).
 * Geometry comes entirely from SPEED_GAUGE_CONFIG.
 */
export function Gauge({ value, max, unit = '', decimals = 2 }: GaugeProps) {
  const cfg = SPEED_GAUGE_CONFIG;

  const arcPath = calculateGaugePath(value, {
    center: cfg.CENTER,
    startPoint: { x: cfg.ARC.START_X, y: cfg.ARC.START_Y },
    radius: cfg.ARC.RADIUS,
    max,
    angleRange: { start: Math.PI, end: 2 * Math.PI },
  });

  const needle = calculateNeedle(value, {
    center: cfg.CENTER,
    length: cfg.NEEDLE_LENGTH,
    max,
    angle: Math.PI,
  });

  return (
    <svg width={cfg.CANVAS_WIDTH} height={cfg.CANVAS_HEIGHT} viewBox={cfg.VIEWBOX}>
      <title>
        Gauge: {formatNumber(value, { decimals })} {unit} of {max} max
      </title>
      <path
        d={cfg.BG_ARC_PATH}
        fill="none"
        stroke={COLORS.GRID_LINE}
        strokeWidth={cfg.ARC.STROKE_WIDTH}
      />
      <path
        d={arcPath}
        fill="none"
        stroke={COLORS.HIGHLIGHT}
        strokeWidth={cfg.ARC.STROKE_WIDTH}
        strokeLinecap="round"
      />
      <line
        x1={cfg.CENTER.x}
        y1={cfg.CENTER.y}
        x2={needle.x}
        y2={needle.y}
        stroke={COLORS.WHITE}
        strokeWidth={cfg.NEEDLE_STROKE}
      />
      <circle cx={cfg.CENTER.x} cy={cfg.CENTER.y} r={cfg.HUB_RADIUS} fill={COLORS.WHITE} />
      <text
        x={cfg.CENTER.x}
        y={cfg.CENTER.y - 15}
        textAnchor="middle"
        fill={COLORS.WHITE}
        fontSize="20"
        fontWeight="bold"
      >
        {formatNumber(value, { decimals })}
      </text>
      {unit && (
        <text
          x={cfg.CENTER.x}
          y={cfg.CENTER.y}
          textAnchor="middle"
          fill="rgba(255,255,255,0.5)"
          fontSize="10"
        >
          {unit}
        </text>
      )}
    </svg>
  );
}
