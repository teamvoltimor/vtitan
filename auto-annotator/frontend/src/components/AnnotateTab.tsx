import ImageOutlinedIcon from '@mui/icons-material/ImageOutlined';
import type { SelectChangeEvent } from '@mui/material';
import {
  Box,
  Button,
  ButtonGroup,
  Card,
  Chip,
  CircularProgress,
  Divider,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Slider,
  Stack,
  ToggleButton,
  ToggleButtonGroup,
  Typography,
  useTheme,
} from '@mui/material';
import type { MouseEvent } from 'react';
import { useEffect, useMemo, useRef, useState } from 'react';
import { useCanvasRender } from '../hooks/useCanvasRender';
import { type AnnotationPoint, type PointType, useAppState } from '../state/appState';
import AnnotateInsights from './AnnotateInsights';
import TimelinePanel from './TimelinePanel';

const pointOptions: { label: string; value: 'positive' | 'negative' }[] = [
  { label: 'Positive', value: 'positive' },
  { label: 'Negative', value: 'negative' },
];

const exportOptions: { label: string; value: 'segmentation' | 'detection' }[] = [
  { label: 'Segmentation', value: 'segmentation' },
  { label: 'Detection', value: 'detection' },
];

const statsRows: { key: 'processed' | 'skipped' | 'labels'; label: string }[] = [
  { key: 'processed', label: 'Processed' },
  { key: 'skipped', label: 'Skipped' },
  { key: 'labels', label: 'Total' },
];

const AnnotateTab = () => {
  const theme = useTheme();
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

  const zoomLabel = useMemo(() => `${Math.round(zoom * 100)}%`, [zoom]);
  const classLabel = activeClass ?? classes[0] ?? 'No class';

  const segmentationStatusColor =
    segmentationStatus === 'ready'
      ? 'success'
      : segmentationStatus === 'pending'
        ? 'warning'
        : segmentationStatus === 'error'
          ? 'error'
          : 'default';

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
      {/* Toolbar — zoom + controls unified */}
      <Box
        sx={{
          display: 'flex',
          flexDirection: 'column',
          gap: 1.5,
          p: 1.5,
          borderRadius: '6px',
          border: `1px solid ${theme.palette.divider}`,
          bgcolor: 'background.paper',
        }}
      >
        <Stack gap={2} direction={{ xs: 'column', sm: 'row' }} alignItems="center">
          <Stack spacing={0.5} sx={{ flex: 1 }}>
            <Typography variant="caption" sx={{ color: 'text.secondary' }}>
              Zoom{' '}
              <Box
                component="span"
                sx={{
                  fontFamily: '"JetBrains Mono", monospace',
                  color: 'text.primary',
                  fontVariantNumeric: 'tabular-nums',
                }}
              >
                {zoomLabel}
              </Box>
            </Typography>
            <Slider
              value={zoom}
              onChange={(_, value) => setZoom(value as number)}
              onChangeCommitted={(_, value) =>
                recordAction(`Set zoom to ${Math.round((value as number) * 100)}%`)
              }
              min={0.5}
              max={2}
              step={0.05}
              marks={[
                { value: 0.5, label: '50%' },
                { value: 1, label: '100%' },
                { value: 1.5, label: '150%' },
              ]}
            />
          </Stack>
          <Button variant="outlined" size="small" sx={{ flexShrink: 0 }} onClick={() => setZoom(1)}>
            Reset
          </Button>
        </Stack>

        <Divider />

        <Stack direction={{ xs: 'column', md: 'row' }} gap={1.5} flexWrap="wrap">
          <FormControl sx={{ minWidth: 160 }} size="small">
            <InputLabel>Point type</InputLabel>
            <Select
              value={pointType}
              label="Point type"
              onChange={(event: SelectChangeEvent<PointType>) =>
                setPointType(event.target.value as PointType)
              }
            >
              {pointOptions.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 160 }} size="small">
            <InputLabel>Mask granularity</InputLabel>
            <Select
              value={maskLevel}
              label="Mask granularity"
              onChange={(event: SelectChangeEvent<string>) => setMaskLevel(event.target.value)}
            >
              <MenuItem value="Object (1)">Object (1)</MenuItem>
              <MenuItem value="Instance">Instance</MenuItem>
              <MenuItem value="Fine detail">Fine detail</MenuItem>
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 160 }} size="small">
            <InputLabel>Export format</InputLabel>
            <Select
              value={exportFormat}
              label="Export format"
              onChange={(event: SelectChangeEvent<'segmentation' | 'detection'>) =>
                setExportFormat(event.target.value as 'segmentation' | 'detection')
              }
            >
              {exportOptions.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 160 }} size="small">
            <InputLabel>Active class</InputLabel>
            <Select
              value={classLabel}
              label="Active class"
              onChange={(event: SelectChangeEvent<string>) => {
                setActiveClass(event.target.value);
                recordAction(`Switched to class ${event.target.value}`);
              }}
            >
              {classes.map((cls) => (
                <MenuItem key={cls} value={cls}>
                  <Stack direction="row" spacing={1} alignItems="center">
                    <Box
                      sx={{
                        width: 8,
                        height: 8,
                        borderRadius: '50%',
                        backgroundColor: classColors[cls] ?? 'transparent',
                        flexShrink: 0,
                      }}
                    />
                    <Typography variant="body2">{cls}</Typography>
                  </Stack>
                </MenuItem>
              ))}
            </Select>
          </FormControl>
        </Stack>
      </Box>

      {/* Action row — canvas ops left, navigation right */}
      <Stack direction="row" spacing={1} alignItems="center" flexWrap="wrap" useFlexGap>
        <Button variant="outlined" size="small" onClick={autoAnnotate}>
          Auto annotate
        </Button>
        <ToggleButtonGroup
          value={annotationMode}
          exclusive
          size="small"
          onChange={(_, value: 'auto' | 'manual' | null) => {
            if (value === null) return;
            setAnnotationMode(value);
            if (value === 'auto') clearAnnotationPoints();
          }}
          sx={{ height: 30 }}
        >
          <ToggleButton value="auto" sx={{ px: 1.5, fontSize: '0.8125rem', textTransform: 'none' }}>
            Auto
          </ToggleButton>
          <ToggleButton
            value="manual"
            sx={{ px: 1.5, fontSize: '0.8125rem', textTransform: 'none' }}
          >
            Manual
          </ToggleButton>
        </ToggleButtonGroup>
        {annotationMode === 'manual' && (
          <Button
            variant="outlined"
            size="small"
            onClick={runSegmentationTest}
            disabled={
              segmentationStatus === 'pending' ||
              annotationPoints.length === 0 ||
              !selectedGalleryItem
            }
            startIcon={
              segmentationStatus === 'pending' ? (
                <CircularProgress size={11} color="inherit" />
              ) : null
            }
          >
            {segmentationStatus === 'pending' ? 'Running…' : 'Generate mask'}
          </Button>
        )}
        <Button
          variant="outlined"
          size="small"
          onClick={clearSegmentationPreview}
          disabled={segmentationPreview.length === 0}
        >
          Clear masks
        </Button>
        <ButtonGroup size="small" sx={{ ml: 0.5 }}>
          <Button onClick={acceptMask}>Accept</Button>
          <Button
            onClick={undoAnnotationPoint}
            disabled={annotationMode !== 'manual' || annotationPoints.length === 0}
          >
            Undo
          </Button>
          <Button
            onClick={clearAnnotationPoints}
            disabled={annotationMode !== 'manual' || annotationPoints.length === 0}
          >
            Clear
          </Button>
        </ButtonGroup>

        <Box sx={{ flex: 1 }} />

        <Button variant="text" size="small" onClick={goToPrev} disabled={!selectedGalleryItem}>
          Previous
        </Button>
        <Button
          variant="text"
          size="small"
          onClick={() => void skipAndNext()}
          disabled={!selectedGalleryItem}
        >
          Skip
        </Button>
        <Button
          variant="contained"
          size="small"
          onClick={() => void saveAndNext()}
          disabled={!selectedGalleryItem || segmentationPreview.length === 0}
        >
          Save &amp; next
        </Button>
      </Stack>

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

        {/* Canvas empty state */}
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

      {/* Compact status row — filename · stats · SAM chip */}
      <Stack
        direction={{ xs: 'column', sm: 'row' }}
        alignItems={{ sm: 'center' }}
        spacing={2}
        useFlexGap
        flexWrap="wrap"
        sx={{ borderTop: `1px solid ${theme.palette.divider}`, pt: 1.5 }}
      >
        <Typography
          variant="caption"
          sx={{
            color: 'text.disabled',
            fontFamily: '"JetBrains Mono", monospace',
            flex: 1,
            minWidth: 0,
          }}
          noWrap
        >
          {selectedGalleryItem ? selectedGalleryItem.label : 'No image selected'}
        </Typography>

        <Stack direction="row" spacing={3} sx={{ flexShrink: 0 }}>
          {statsRows.map(({ key, label }) => (
            <Stack key={key} direction="row" alignItems="baseline" spacing={0.75}>
              <Typography variant="caption" color="text.disabled">
                {label}
              </Typography>
              <Typography
                variant="body2"
                sx={{
                  fontWeight: 600,
                  fontFamily: '"JetBrains Mono", monospace',
                  fontVariantNumeric: 'tabular-nums',
                }}
              >
                {stats[key]}
              </Typography>
            </Stack>
          ))}
        </Stack>

        <Stack direction="row" alignItems="center" spacing={1} sx={{ flexShrink: 0 }}>
          <Chip
            label={`SAM · ${segmentationStatus}`}
            size="small"
            color={
              segmentationStatusColor as
                | 'default'
                | 'primary'
                | 'secondary'
                | 'error'
                | 'info'
                | 'success'
                | 'warning'
            }
          />
          {segmentationMessage && (
            <Typography variant="caption" color="text.secondary" noWrap sx={{ maxWidth: 200 }}>
              {segmentationMessage}
            </Typography>
          )}
        </Stack>
      </Stack>

      <Stack direction={{ xs: 'column', md: 'row' }} spacing={3}>
        <AnnotateInsights />
        <TimelinePanel events={timeline} />
      </Stack>
    </Stack>
  );
};

export default AnnotateTab;
