import ImageOutlinedIcon from '@mui/icons-material/ImageOutlined';
import { Box, Card, Stack, Typography } from '@mui/material';
import type { MouseEvent } from 'react';
import { useEffect, useRef, useState } from 'react';
import { useCanvasRender } from '../hooks/useCanvasRender';
import { type AnnotationPoint, useAppState } from '../state/appState';
import AnnotateInsights from './AnnotateInsights';
import { AnnotationActions } from './AnnotationActions';
import { StatusBar } from './StatusBar';
import TimelinePanel from './TimelinePanel';
import { ZoomToolbarSection } from './ZoomControl';

const AnnotateTab = () => {
  const {
    zoom,
    setZoom,
    pointType,
    setPointType,
    exportFormat,
    setExportFormat,
    maskLevel,
    setMaskLevel,
    classes,
    classColors,
    activeClass,
    setActiveClass,
    autoAnnotate,
    timeline,
    recordAction,
    stats,
    selectedGalleryItem,
    annotationMode,
    setAnnotationMode,
    annotationPoints,
    addAnnotationPoint,
    segmentationPreview,
    runSegmentationTest,
    segmentFromPoints,
    clearSegmentationPreview,
    clearAnnotationPoints,
    undoAnnotationPoint,
    segmentationStatus,
    segmentationMessage,
    saveAndNext,
    skipAndNext,
    goToPrev,
    acceptMask,
  } = useAppState();
  // Queue for clicks that arrive while an inference is in-flight (auto mode only).
  // Ref holds the authoritative queue; state mirrors it for canvas rendering.
  const clickQueue = useRef<AnnotationPoint[]>([]);
  const [queuedPoints, setQueuedPoints] = useState<AnnotationPoint[]>([]);
  const { canvasRef, imageBounds } = useCanvasRender(
    annotationPoints,
    segmentationPreview,
    queuedPoints
  );

  // Clear the queue whenever the user switches to a different image.
  // biome-ignore lint/correctness/useExhaustiveDependencies: selectedGalleryItem?.id is an intentional re-run trigger, not read inside the effect
  useEffect(() => {
    clickQueue.current = [];
    setQueuedPoints([]);
  }, [selectedGalleryItem?.id]);

  // When inference finishes, drain the next queued click.
  useEffect(() => {
    if (segmentationStatus === 'pending') return;
    const next = clickQueue.current.shift();
    if (next) {
      setQueuedPoints([...clickQueue.current]);
      addAnnotationPoint(next);
      void segmentFromPoints([next]);
    }
  }, [segmentationStatus, addAnnotationPoint, segmentFromPoints]);

  const classLabel = activeClass ?? classes[0] ?? 'No class';

  const handleCanvasClick = (event: MouseEvent<HTMLCanvasElement>) => {
    const bounds = imageBounds.current;
    if (!bounds) return;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const relativeX = (event.clientX - rect.left) * scaleX - bounds.x;
    const relativeY = (event.clientY - rect.top) * scaleY - bounds.y;
    if (relativeX < 0 || relativeY < 0 || relativeX > bounds.width || relativeY > bounds.height)
      return;
    const normalizedX = relativeX / bounds.width;
    const normalizedY = relativeY / bounds.height;
    const point = {
      x: normalizedX,
      y: normalizedY,
      pointType,
      className: classLabel,
      color: classColors[classLabel] ?? '#ffffff',
    };
    if (annotationMode === 'auto') {
      if (segmentationStatus === 'pending') {
        // Queue the click — no dot added so no phantom polygon is drawn.
        clickQueue.current.push(point);
        setQueuedPoints([...clickQueue.current]);
      } else {
        addAnnotationPoint(point);
        void segmentFromPoints([point]);
      }
    } else {
      addAnnotationPoint(point);
    }
  };

  return (
    <Stack spacing={2.5} sx={{ height: '100%' }}>
      <ZoomToolbarSection
        zoom={zoom}
        setZoom={setZoom}
        recordAction={recordAction}
        pointType={pointType}
        setPointType={setPointType}
        maskLevel={maskLevel}
        setMaskLevel={setMaskLevel}
        exportFormat={exportFormat}
        setExportFormat={setExportFormat}
        activeClass={activeClass}
        setActiveClass={setActiveClass}
        classes={classes}
        classColors={classColors}
      />

      <AnnotationActions
        annotationMode={annotationMode}
        setAnnotationMode={(mode) => {
          setAnnotationMode(mode);
          if (mode === 'auto') clearAnnotationPoints();
        }}
        annotationPointsCount={annotationPoints.length}
        segmentationStatus={segmentationStatus}
        selectedGalleryItem={!!selectedGalleryItem}
        segmentationPreviewLength={segmentationPreview.length}
        onAutoAnnotate={autoAnnotate}
        onRunSegmentation={runSegmentationTest}
        onClearMasks={clearSegmentationPreview}
        onAccept={acceptMask}
        onUndo={undoAnnotationPoint}
        onClear={clearAnnotationPoints}
        onPrev={goToPrev}
        onSkip={() => void skipAndNext()}
        onSaveNext={() => void saveAndNext()}
      />

      {/* Canvas */}
      <Card variant="outlined" sx={{ flex: 1, overflow: 'hidden', position: 'relative' }}>
        <Box
          sx={{
            height: '100%',
            overflow: 'auto',
            px: 1,
            py: 2,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }}
        >
          <Box
            component="canvas"
            ref={canvasRef}
            width={960}
            height={560}
            onClick={handleCanvasClick}
            sx={{
              backgroundColor: 'background.default',
              width: '100%',
              maxWidth: 960,
              cursor: 'crosshair',
            }}
          />
        </Box>

        {!selectedGalleryItem && (
          <Box
            sx={{
              position: 'absolute',
              inset: 0,
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              gap: 1,
              pointerEvents: 'none',
            }}
          >
            <ImageOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
            <Typography variant="body2" sx={{ color: 'text.disabled', fontWeight: 500 }}>
              No image selected
            </Typography>
            <Typography variant="caption" sx={{ color: 'text.disabled' }}>
              Browse your gallery to begin annotating
            </Typography>
          </Box>
        )}
      </Card>

      <StatusBar
        filename={selectedGalleryItem ? selectedGalleryItem.label : 'No image selected'}
        stats={stats}
        segmentationStatus={segmentationStatus}
        segmentationMessage={segmentationMessage}
      />

      <Stack direction={{ xs: 'column', md: 'row' }} spacing={3}>
        <AnnotateInsights />
        <TimelinePanel events={timeline} />
      </Stack>
    </Stack>
  );
};

export default AnnotateTab;
