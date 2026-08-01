import { COLORS } from '../../config';
import { clamp, formatNumber } from '../../utils/formatting';

interface BarChartProps {
  label: string;
  value: number;
  range: [number, number];
  unit: string;
}

/** Horizontal centre-zero bar, red when the value nears the range edges. */
export function BarChart({ label, value, range, unit }: BarChartProps) {
  const [min, max] = range;
  const clamped = clamp(value, min, max);
  const percentage = ((clamped - min) / (max - min)) * 100;
  // Danger threshold is 80% of the axis's own extent, not of its full span
  // (min..max) — for a symmetric range like [-10, 10] that's 8, not 16.
  const dangerThreshold = Math.max(Math.abs(min), Math.abs(max)) * 0.8;
  const color = Math.abs(value) > dangerThreshold ? COLORS.DANGER : COLORS.SUCCESS;

  return (
    <div className="bar-chart">
      <span className="bar-label">{label}</span>
      <div className="bar-container">
        <div className="bar-background">
          <div className="bar-zero" style={{ left: '50%' }} />
          <div
            className="bar-fill"
            style={{
              width: `${Math.abs(percentage - 50)}%`,
              left: `${Math.min(50, percentage)}%`,
              backgroundColor: color,
            }}
          />
        </div>
        <span className="bar-value">{formatNumber(value, { decimals: 2, unit })}</span>
      </div>
    </div>
  );
}
