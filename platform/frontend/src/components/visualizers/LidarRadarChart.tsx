import { useRef, useEffect } from 'react';
import type { LaserScanMsg } from '../../types';
import { LIDAR_CONFIG } from '../../config';

/** Top-down LiDAR radar chart with danger/warning/safe colouring. */
export function LidarRadarChart({ data, expanded }: { data: LaserScanMsg; expanded?: boolean }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const size = expanded
    ? LIDAR_CONFIG.RADAR_CHART.CANVAS_WIDTH * 2
    : LIDAR_CONFIG.RADAR_CHART.CANVAS_WIDTH;

  useEffect(() => {
    if (!canvasRef.current || !data?.ranges) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    const { width, height } = canvasRef.current;
    const centerX = width / 2;
    const centerY = height / 2;
    const maxRange = LIDAR_CONFIG.MAX_RANGE_METERS;
    const scale = Math.min(width, height) / 2 / maxRange;

    ctx.clearRect(0, 0, width, height);

    ctx.strokeStyle = LIDAR_CONFIG.RADAR_CHART.COLORS.GRID;
    ctx.lineWidth = 1;
    const gridStep = LIDAR_CONFIG.RADAR_CHART.GRID_STEP_METERS;
    for (let r = gridStep; r <= maxRange; r += gridStep) {
      ctx.beginPath();
      ctx.arc(centerX, centerY, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.strokeStyle = LIDAR_CONFIG.RADAR_CHART.COLORS.FORWARD_INDICATOR;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, 0);
    ctx.stroke();

    const angleMin = data.angle_min ?? 0;
    const angleIncrement = data.angle_increment ?? 0.01;
    const { DANGER, WARNING } = LIDAR_CONFIG.RADAR_CHART.DISTANCE_THRESHOLDS;
    const colors = LIDAR_CONFIG.RADAR_CHART.COLORS;

    data.ranges.forEach((range, index) => {
      if (range < (data.range_min || 0) || range > (data.range_max || maxRange)) return;

      const angle = angleMin + index * angleIncrement;
      const x = range * Math.cos(angle);
      const y = range * Math.sin(angle);

      const color = range < DANGER ? colors.DANGER : range < WARNING ? colors.WARNING : colors.SAFE;
      ctx.fillStyle = color;
      ctx.fillRect(
        centerX - y * scale - LIDAR_CONFIG.RADAR_CHART.DOT_OFFSET_PIXELS,
        centerY - x * scale - LIDAR_CONFIG.RADAR_CHART.DOT_OFFSET_PIXELS,
        LIDAR_CONFIG.RADAR_CHART.DOT_SIZE_PIXELS,
        LIDAR_CONFIG.RADAR_CHART.DOT_SIZE_PIXELS
      );
    });
  }, [data]);

  return (
    <div className="specialized-viz radar-viz">
      <canvas ref={canvasRef} width={size} height={size} />
    </div>
  );
}
