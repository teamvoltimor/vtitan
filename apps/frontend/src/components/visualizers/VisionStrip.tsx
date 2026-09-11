/**
 * src/components/visualizers/VisionStrip.tsx
 *
 * Live camera-detection strip rendered from the telemetry snapshot's own
 * `vision_detections` field (audit §7.2). Unlike the topic inspector's
 * VisionBoundingBoxes, this consumes the spec-native flat Detection model —
 * no ROS message shape, no vision-debug JPEG stream required. Rendered as an
 * unobtrusive overlay on the 3D canvas, top-right.
 */

import { COLORS, VISION_CONFIG } from '../../config';
import type { Detection } from '../../types';
import { formatNumber } from '../../utils/formatting';

function colorFor(class_name: string): string {
  if (class_name.includes('red')) return VISION_CONFIG.COLORS.RED;
  if (class_name.includes('green')) return VISION_CONFIG.COLORS.GREEN;
  return VISION_CONFIG.COLORS.BLUE;
}

/** Draws the latest frame's detection bounding boxes, coloured by class. */
export function VisionStrip({ detections }: { detections: readonly Detection[] }) {
  if (detections.length === 0) {
    return null;
  }

  return (
    <div
      className="vision-strip"
      role="img"
      aria-label={`${detections.length} vision detection(s)`}
    >
      <svg
        width={VISION_CONFIG.CANVAS_WIDTH}
        height={VISION_CONFIG.CANVAS_HEIGHT}
        viewBox={VISION_CONFIG.VIEWBOX}
        aria-hidden="true"
      >
        {detections.map((det) => {
          const color = colorFor(det.class_name);
          const labelWidth = Math.max(
            VISION_CONFIG.MIN_LABEL_WIDTH,
            det.class_name.length * VISION_CONFIG.LABEL_WIDTH_PER_CHAR
          );
          const key = `${det.class_name}-${det.bbox_x}-${det.bbox_y}-${det.bbox_w}-${det.bbox_h}`;
          return (
            <g key={key}>
              <rect
                x={det.bbox_x}
                y={det.bbox_y}
                width={det.bbox_w}
                height={det.bbox_h}
                fill="none"
                stroke={color}
                strokeWidth={VISION_CONFIG.BBOX_STROKE_WIDTH}
              />
              <rect
                x={det.bbox_x}
                y={det.bbox_y - 20}
                width={labelWidth}
                height="20"
                fill={color}
              />
              <text
                x={det.bbox_x + 5}
                y={det.bbox_y - 5}
                fill={COLORS.WHITE}
                fontSize="14"
                fontWeight="bold"
              >
                {det.class_name} ({formatNumber(det.confidence * 100, { decimals: 0, unit: '%' })})
              </text>
            </g>
          );
        })}
      </svg>
    </div>
  );
}
