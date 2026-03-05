import { useCallback, useEffect, useRef } from 'react';
import type { AnnotationPoint, SegmentationPreviewShape } from '../state/appState';
import { useAppState } from '../state/appState';

interface ImageBounds {
  x: number;
  y: number;
  width: number;
  height: number;
}

export const useCanvasRender = (
  annotationPoints: AnnotationPoint[],
  segmentationPreview: SegmentationPreviewShape[],
  queuedPoints: AnnotationPoint[] = []
) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const { zoom, selectedGalleryItem, classColors } = useAppState();
  const imageBoundsRef = useRef<ImageBounds | null>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const dashOffsetRef = useRef(0);
  const animationFrame = useRef<number | null>(null);

  const drawAnnotations = useCallback(
    (ctx: CanvasRenderingContext2D, bounds: ImageBounds | null) => {
      if (!bounds) return;
      annotationPoints.forEach((point) => {
        const x = bounds.x + point.x * bounds.width;
        const y = bounds.y + point.y * bounds.height;
        const radius = 18;

        ctx.save();
        ctx.beginPath();
        ctx.fillStyle = `${point.color}33`;
        ctx.strokeStyle = point.color;
        ctx.lineWidth = point.pointType === 'positive' ? 2.5 : 1.5;
        ctx.globalAlpha = point.pointType === 'positive' ? 0.85 : 1;
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.stroke();

        if (point.pointType === 'negative') {
          ctx.beginPath();
          ctx.moveTo(x - radius * 0.7, y - radius * 0.7);
          ctx.lineTo(x + radius * 0.7, y + radius * 0.7);
          ctx.moveTo(x + radius * 0.7, y - radius * 0.7);
          ctx.lineTo(x - radius * 0.7, y + radius * 0.7);
          ctx.stroke();
        }
        ctx.restore();
      });
    },
    [annotationPoints]
  );

  const drawSegmentationPreview = useCallback(
    (ctx: CanvasRenderingContext2D, bounds: ImageBounds | null) => {
      if (!bounds || segmentationPreview.length === 0) return;
      segmentationPreview.forEach((shape) => {
        if (shape.points.length < 3) return;
        const color = classColors[shape.className] ?? '#94e2d5';
        ctx.save();
        ctx.beginPath();
        shape.points.forEach((point, index) => {
          const x = bounds.x + point.x * bounds.width;
          const y = bounds.y + point.y * bounds.height;
          if (index === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.closePath();
        ctx.fillStyle = `${color}2a`;
        ctx.strokeStyle = `${color}88`;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.lineDashOffset = dashOffsetRef.current;
        ctx.fill();
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      });
    },
    [classColors, segmentationPreview]
  );

  const drawQueuedPoints = useCallback(
    (ctx: CanvasRenderingContext2D, bounds: ImageBounds | null) => {
      if (!bounds || queuedPoints.length === 0) return;
      queuedPoints.forEach((point) => {
        const x = bounds.x + point.x * bounds.width;
        const y = bounds.y + point.y * bounds.height;
        const radius = 12;
        ctx.save();
        ctx.globalAlpha = 0.35;
        ctx.beginPath();
        ctx.fillStyle = `${point.color}33`;
        ctx.strokeStyle = point.color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      });
    },
    [queuedPoints]
  );

  const drawMockMask = useCallback(
    (ctx: CanvasRenderingContext2D, bounds: ImageBounds | null) => {
      if (!bounds || annotationPoints.length === 0) return;
      const grouped = annotationPoints.reduce<Record<string, AnnotationPoint[]>>((acc, point) => {
        const classKey = point.className || 'default';
        if (!acc[classKey]) acc[classKey] = [];
        acc[classKey].push(point);
        return acc;
      }, {});

      Object.entries(grouped).forEach(([className, points]) => {
        if (points.length < 2) return;
        const color = classColors[className] ?? '#ffffff';
        ctx.save();
        ctx.beginPath();
        points.forEach((point, index) => {
          const x = bounds.x + point.x * bounds.width;
          const y = bounds.y + point.y * bounds.height;
          if (index === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.closePath();
        ctx.fillStyle = `${color}2a`;
        ctx.strokeStyle = `${color}70`;
        ctx.lineWidth = 1;
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      });
    },
    [annotationPoints, classColors]
  );

  const ensureCanvasResolution = () => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return null;
    const dpr = window.devicePixelRatio || 1;
    const pixelWidth = Math.round(rect.width * dpr);
    const pixelHeight = Math.round(rect.height * dpr);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    return {
      rectWidth: rect.width,
      rectHeight: rect.height,
      canvasWidth: pixelWidth,
      canvasHeight: pixelHeight,
      dpr,
    };
  };

  const drawCanvas = useCallback(
    (ctx: CanvasRenderingContext2D, image: HTMLImageElement) => {
      const metrics = ensureCanvasResolution();
      if (!metrics) return;
      const { canvasWidth, canvasHeight, dpr } = metrics;
      const margin = 80 * dpr;
      const scale = Math.min(
        (canvasWidth - margin) / image.width,
        (canvasHeight - margin) / image.height
      );
      const drawWidth = image.width * scale * zoom;
      const drawHeight = image.height * scale * zoom;
      const x = (canvasWidth - drawWidth) / 2;
      const y = (canvasHeight - drawHeight) / 2;
      imageBoundsRef.current = { x, y, width: drawWidth, height: drawHeight };

      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, canvasWidth, canvasHeight);
      ctx.fillStyle = '#10101a';
      ctx.fillRect(0, 0, canvasWidth, canvasHeight);
      ctx.drawImage(image, x, y, drawWidth, drawHeight);
      drawSegmentationPreview(ctx, imageBoundsRef.current);
      drawMockMask(ctx, imageBoundsRef.current);
      drawAnnotations(ctx, imageBoundsRef.current);
      drawQueuedPoints(ctx, imageBoundsRef.current);
    },
    [zoom, drawSegmentationPreview, drawMockMask, drawAnnotations, drawQueuedPoints]
  );

  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    if (!selectedGalleryItem) {
      ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height);
      imageBoundsRef.current = null;
      imageRef.current = null;
      return;
    }

    const image = new Image();
    imageRef.current = image;
    image.src = selectedGalleryItem.src;
    image.onload = () => {
      if (!canvasRef.current) return;
      const ctxAfter = canvasRef.current.getContext('2d');
      if (!ctxAfter) return;
      drawCanvas(ctxAfter, image);
    };
  }, [selectedGalleryItem, drawCanvas]);

  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx || !imageRef.current) return;
    drawCanvas(ctx, imageRef.current);
  }, [annotationPoints, queuedPoints, zoom, selectedGalleryItem, classColors, segmentationPreview, drawCanvas]);

  useEffect(() => {
    if (segmentationPreview.length === 0) {
      if (animationFrame.current) {
        cancelAnimationFrame(animationFrame.current);
        animationFrame.current = null;
      }
      return;
    }

    const animate = () => {
      dashOffsetRef.current = (dashOffsetRef.current + 0.8) % 12;
      if (canvasRef.current && imageRef.current) {
        const ctx = canvasRef.current.getContext('2d');
        if (ctx) {
          drawCanvas(ctx, imageRef.current);
        }
      }
      animationFrame.current = requestAnimationFrame(animate);
    };

    animationFrame.current = requestAnimationFrame(animate);
    return () => {
      if (animationFrame.current) {
        cancelAnimationFrame(animationFrame.current);
        animationFrame.current = null;
      }
    };
  }, [segmentationPreview, annotationPoints, queuedPoints, zoom, selectedGalleryItem, classColors, drawCanvas]);

  return { canvasRef, imageBounds: imageBoundsRef };
};
