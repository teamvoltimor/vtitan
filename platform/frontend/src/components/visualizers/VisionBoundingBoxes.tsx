import type { Detection2DArrayMsg } from '../../types'
import { formatNumber } from '../../utils/formatting'
import { VISION_CONFIG, COLORS } from '../../config'

/** Draws vision detection bounding boxes, coloured by class, from a Detection2DArray. */
export function VisionBoundingBoxes({ data }: { data: Detection2DArrayMsg }) {
  if (!data?.detections || data.detections.length === 0) {
    return <div className="specialized-viz">No detections</div>
  }

  return (
    <div className="specialized-viz vision-boxes">
      <svg
        width={VISION_CONFIG.CANVAS_WIDTH}
        height={VISION_CONFIG.CANVAS_HEIGHT}
        viewBox={VISION_CONFIG.VIEWBOX}
        style={{ background: '#111', borderRadius: '4px' }}
      >
        {data.detections.map((det, i) => {
          const hypothesis = det.results?.[0]?.hypothesis
          const classId = hypothesis?.class_id ?? 'unknown'
          const score = hypothesis?.score ?? 0

          const bbox = det.bbox
          const cx = bbox?.center?.position?.x ?? 0
          const cy = bbox?.center?.position?.y ?? 0
          const w = bbox?.size_x ?? 0
          const h = bbox?.size_y ?? 0

          const x = cx - w / 2
          const y = cy - h / 2

          const color = classId.includes('red')
            ? VISION_CONFIG.COLORS.RED
            : classId.includes('green')
              ? VISION_CONFIG.COLORS.GREEN
              : VISION_CONFIG.COLORS.BLUE

          return (
            <g key={i}>
              <rect
                x={x}
                y={y}
                width={w}
                height={h}
                fill="none"
                stroke={color}
                strokeWidth={VISION_CONFIG.BBOX_STROKE_WIDTH}
              />
              <rect
                x={x}
                y={y - 20}
                width={Math.max(VISION_CONFIG.MIN_LABEL_WIDTH, classId.length * VISION_CONFIG.LABEL_WIDTH_PER_CHAR)}
                height="20"
                fill={color}
              />
              <text x={x + 5} y={y - 5} fill={COLORS.WHITE} fontSize="14" fontWeight="bold">
                {classId} ({formatNumber(score * 100, { decimals: 0, unit: '%' })})
              </text>
            </g>
          )
        })}
      </svg>
    </div>
  )
}
