import { Box } from '@mui/material';
import { useEffect, useRef } from 'react';
import type { SegmentationShape } from '../api/client';

interface AnnotatedThumbnailProps {
  src: string;
  alt: string;
  annotations: SegmentationShape[];
  borderRadius?: string;
}

const COLORS = [
  '#ef5350',
  '#66bb6a',
  '#42a5f5',
  '#ffa726',
  '#ab47bc',
  '#ec407a',
  '#26a69a',
  '#ff7043',
];

const AnnotatedThumbnail = ({
  src,
  alt,
  annotations,
  borderRadius = '6px',
}: AnnotatedThumbnailProps) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement>(null);

  // biome-ignore lint/correctness/useExhaustiveDependencies: src is a re-run trigger for the ref-based img element, not read by name in the effect
  useEffect(() => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !img || annotations.length === 0) return;

    const render = () => {
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      const W = img.offsetWidth || img.clientWidth;
      const H = img.offsetHeight || img.clientHeight;
      if (W === 0 || H === 0) return;
      canvas.width = W;
      canvas.height = H;
      ctx.clearRect(0, 0, W, H);

      annotations.forEach((shape, idx) => {
        const color = COLORS[idx % COLORS.length];
        const pts = shape.points;
        if (pts.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(1, W / 200);
        ctx.beginPath();
        ctx.moveTo(parseFloat(pts[0].x) * W, parseFloat(pts[0].y) * H);
        for (let i = 1; i < pts.length; i++) {
          ctx.lineTo(parseFloat(pts[i].x) * W, parseFloat(pts[i].y) * H);
        }
        ctx.closePath();
        ctx.stroke();
      });
    };

    if (img.complete && img.naturalWidth > 0) {
      render();
    } else {
      img.onload = render;
    }
  }, [src, annotations]);

  return (
    <Box sx={{ position: 'relative', width: '100%', overflow: 'hidden', borderRadius }}>
      <img
        ref={imgRef}
        src={src}
        alt={alt}
        style={{ width: '100%', display: 'block', borderRadius }}
      />
      {annotations.length > 0 && (
        <canvas
          ref={canvasRef}
          style={{
            position: 'absolute',
            top: 0,
            left: 0,
            width: '100%',
            height: '100%',
            pointerEvents: 'none',
          }}
        />
      )}
    </Box>
  );
};

export default AnnotatedThumbnail;
