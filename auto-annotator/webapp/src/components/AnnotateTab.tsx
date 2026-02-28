import {
  Box,
  Button,
  ButtonGroup,
  Card,
  Chip,
  Divider,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  SelectChangeEvent,
  Slider,
  Stack,
  Typography,
} from '@mui/material'
import { useMemo } from 'react'
import type { MouseEvent } from 'react'
import { useAppState } from '../state/appState'
import { useCanvasRender } from '../hooks/useCanvasRender'
import AnnotateInsights from './AnnotateInsights'
import TimelinePanel from './TimelinePanel'

const pointOptions: { label: string; value: 'positive' | 'negative' }[] = [
  { label: 'Positive', value: 'positive' },
  { label: 'Negative', value: 'negative' },
]

const exportOptions: { label: string; value: 'segmentation' | 'detection' }[] = [
  { label: 'Segmentation', value: 'segmentation' },
  { label: 'Detection', value: 'detection' },
]

const statsRows = ['processed', 'skipped', 'labels'] as const

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
    logEntries,
    stats,
    selectedGalleryItem,
    annotationPoints,
    addAnnotationPoint,
    segmentationPreview,
    runSegmentationTest,
    clearSegmentationPreview,
    clearAnnotationPoints,
    undoAnnotationPoint,
    segmentationStatus,
  } = useAppState()
  const { canvasRef, imageBounds } = useCanvasRender(annotationPoints, segmentationPreview)

  const zoomLabel = useMemo(() => `${Math.round(zoom * 100)}%`, [zoom])
  const classLabel = activeClass ?? classes[0] ?? 'No class'

  const statusChipLabel = selectedGalleryItem ? 'Drawing ready' : 'Awaiting selection'
  const segmentationStatusColor =
    segmentationStatus === 'ready'
      ? 'success'
      : segmentationStatus === 'pending'
      ? 'warning'
      : segmentationStatus === 'error'
      ? 'error'
      : 'default'

  const handleCanvasClick = (event: MouseEvent<HTMLCanvasElement>) => {
    const bounds = imageBounds.current
    if (!bounds) return
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    const relativeX = (event.clientX - rect.left) * scaleX - bounds.x
    const relativeY = (event.clientY - rect.top) * scaleY - bounds.y
    if (relativeX < 0 || relativeY < 0 || relativeX > bounds.width || relativeY > bounds.height) return
    const normalizedX = relativeX / bounds.width
    const normalizedY = relativeY / bounds.height
    addAnnotationPoint({
      x: normalizedX,
      y: normalizedY,
      pointType,
      className: classLabel,
      color: classColors[classLabel] ?? '#ffffff',
    })
  }

  return (
    <Stack spacing={3} sx={{ height: '100%' }}>
      <Stack gap={2} direction={{ xs: 'column', sm: 'row' }} alignItems="center">
        <Stack spacing={1} sx={{ flex: 1 }}>
          <Typography variant="body2" fontWeight={600} color="text.secondary">
            Zoom ({zoomLabel})
          </Typography>
          <Slider
            value={zoom}
            onChange={(_, value) => {
              setZoom(value as number)
              recordAction(`Set zoom to ${Math.round((value as number) * 100)}%`)
            }}
            min={0.5}
            max={2}
            step={0.05}
            marks={[{ value: 0.5, label: '50%' }, { value: 1, label: '100%' }, { value: 1.5, label: '150%' }]}
          />
        </Stack>
        <Button variant="outlined" sx={{ height: 40 }} onClick={() => setZoom(1)}>
          Reset zoom
        </Button>
      </Stack>
        <Stack direction={{ xs: 'column', md: 'row' }} gap={2} flexWrap="wrap">
          <FormControl sx={{ minWidth: 180 }}>
            <InputLabel>Point type</InputLabel>
          <Select
            value={pointType}
            label="Point type"
            onChange={(event: SelectChangeEvent<PointType>) => setPointType(event.target.value as PointType)}
          >
              {pointOptions.map((option) => (
                <MenuItem key={option.value} value={option.value}>
                  {option.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <FormControl sx={{ minWidth: 180 }}>
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
          <FormControl sx={{ minWidth: 180 }}>
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
          <FormControl sx={{ minWidth: 180 }}>
            <InputLabel>Active class</InputLabel>
            <Select
              value={classLabel}
              label="Active class"
            onChange={(event: SelectChangeEvent<string>) => {
              setActiveClass(event.target.value)
              recordAction(`Switched to class ${event.target.value}`)
            }}
            >
              {classes.map((cls) => (
                <MenuItem key={cls} value={cls}>
                  <Stack direction="row" spacing={1} alignItems="center">
                    <Box
                      sx={{
                        width: 10,
                        height: 10,
                        borderRadius: '50%',
                        backgroundColor: classColors[cls] ?? 'transparent',
                      }}
                    />
                    <Typography variant="body2">{cls}</Typography>
                  </Stack>
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <ButtonGroup sx={{ ml: 'auto' }}>
            <Button variant="contained" onClick={() => recordAction('Accepted mask')}>
              Accept mask
            </Button>
            <Button onClick={undoAnnotationPoint}>Undo</Button>
            <Button onClick={clearAnnotationPoints}>Clear</Button>
          </ButtonGroup>
        </Stack>
      <Stack direction={{ xs: 'column', md: 'row' }} flexWrap="wrap" spacing={2} alignItems="center">
        <Button variant="outlined" size="small" onClick={autoAnnotate}>
          Auto annotate
        </Button>
        <Button
          variant="outlined"
          size="small"
          onClick={runSegmentationTest}
          disabled={segmentationStatus === 'pending' || annotationPoints.length === 0 || !selectedGalleryItem}
        >
          Generate mask
        </Button>
        <Button variant="outlined" size="small" onClick={clearSegmentationPreview} disabled={segmentationPreview.length === 0}>
          Clear masks
        </Button>
        <Button variant="outlined" size="small">
          Skip
        </Button>
        <Button variant="outlined" size="small">
          Previous
        </Button>
        <Button variant="contained" size="small">
          Save & next
        </Button>
      </Stack>
      <Card
        variant="outlined"
        sx={{
          flex: 1,
          overflow: 'hidden',
          borderRadius: 0,
          bgcolor: 'background.paper',
          borderColor: 'divider',
          boxShadow: 'none',
        }}
      >
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
                borderRadius: 0,
                backgroundColor: 'background.default',
                width: '100%',
                maxWidth: 960,
                border: '1px solid rgba(255,255,255,0.08)',
                boxShadow: '0 12px 30px rgba(0, 0, 0, 0.55)',
                cursor: 'crosshair',
              }}
            />
          </Box>
        </Card>
      <Card variant="outlined" sx={{ borderRadius: 3, p: 3 }}>
          <Stack spacing={2}>
            <Stack direction="row" justifyContent="space-between" alignItems="center">
              <Stack spacing={0.5}>
                <Typography variant="subtitle2" fontWeight={600}>
                  Annotation status
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  {selectedGalleryItem ? selectedGalleryItem.label : 'No image selected'}
                </Typography>
              </Stack>
              <Chip label={statusChipLabel} color={selectedGalleryItem ? 'success' : 'default'} size="small" />
            </Stack>
            <Divider />
            <Stack direction="row" spacing={2} flexWrap="wrap" justifyContent="space-between">
              {statsRows.map((key) => (
                <Stack key={key} spacing={0.5}>
                  <Typography variant="caption" color="text.secondary">
                    {key}
                  </Typography>
                  <Typography variant="body1" fontWeight={600}>
                    {stats[key]}
                  </Typography>
                </Stack>
              ))}
            </Stack>
            <Stack direction="row" spacing={1} flexWrap="wrap">
              <Chip label={`Point ${pointType}`} size="small" />
              <Chip label={`Mask ${maskLevel}`} size="small" />
              <Chip label={`Export ${exportFormat}`} size="small" />
              <Chip label={`Class ${classLabel}`} size="small" />
            </Stack>
            <Stack direction="row" alignItems="center" spacing={1} flexWrap="wrap">
              <Chip
                label={`SAM ${segmentationStatus}`}
                size="small"
                color={segmentationStatusColor as 'default' | 'primary' | 'secondary' | 'error' | 'info' | 'success' | 'warning'}
              />
              <Typography variant="body2" color="text.secondary" sx={{ maxWidth: 360 }} noWrap>
                {segmentationMessage || 'SAM response will show here once you generate a mask.'}
              </Typography>
            </Stack>
            <Divider />
          <Stack spacing={1}>
            <Typography variant="caption" color="text.secondary">
              Recent log
            </Typography>
            {logEntries.length === 0 ? (
              <Typography variant="body2">No activity yet</Typography>
            ) : (
              logEntries.map((entry) => (
                <Typography key={entry} variant="body2" fontWeight={500} noWrap>
                  {entry}
                </Typography>
              ))
            )}
          </Stack>
        </Stack>
      </Card>
      <Stack direction={{ xs: 'column', md: 'row' }} spacing={3}>
        <AnnotateInsights />
        <TimelinePanel events={timeline} />
      </Stack>
    </Stack>
  )
}

export default AnnotateTab
