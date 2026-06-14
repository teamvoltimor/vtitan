import { formatNumber } from '../../utils/formatting'
import { COLORS } from '../../config'

interface BarChartProps {
  label: string
  value: number
  range: [number, number]
  unit: string
}

/** Horizontal centre-zero bar, red when the value nears the range edges. */
export function BarChart({ label, value, range, unit }: BarChartProps) {
  const [min, max] = range
  const percentage = ((value - min) / (max - min)) * 100
  const color = Math.abs(value) > (max - min) * 0.8 ? COLORS.DANGER : COLORS.SUCCESS

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
  )
}
