import {
  Box,
  Button,
  ButtonGroup,
  Card,
  Chip,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Slider,
  Stack,
  Typography,
} from '@mui/material'
import { useMemo } from 'react'
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
    activeClass,
    setActiveClass,
    autoAnnotate,
    timeline,
    recordAction,
    selectedGalleryItem,
  } = useAppState()
  const canvasRef = useCanvasRender()

  const zoomLabel = useMemo(() => `${Math.round(zoom * 100)}%`, [zoom])

  return (
    <Stack spacing={3} sx={{ height: '100%' }}>
      <Stack gap={2} direction={{ xs: 'column', sm: 'row' }} alignItems="flex-start">
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
        <FormControl sx={{ minWidth: 200 }}>
          <InputLabel>Point type</InputLabel>
          <Select value={pointType} label="Point type" onChange={(event) => setPointType(event.target.value as any)}>
            {pointOptions.map((option) => (
              <MenuItem key={option.value} value={option.value}>
                {option.label}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
        <FormControl sx={{ minWidth: 200 }}>
          <InputLabel>Mask granularity</InputLabel>
          <Select value={maskLevel} label="Mask granularity" onChange={(event) => setMaskLevel(event.target.value as string)}>
            <MenuItem value="Object (1)">Object (1)</MenuItem>
            <MenuItem value="Instance">Instance</MenuItem>
            <MenuItem value="Fine detail">Fine detail</MenuItem>
          </Select>
        </FormControl>
        <FormControl sx={{ minWidth: 200 }}>
          <InputLabel>Export format</InputLabel>
          <Select value={exportFormat} label="Export format" onChange={(event) => setExportFormat(event.target.value as any)}>
            {exportOptions.map((option) => (
              <MenuItem key={option.value} value={option.value}>
                {option.label}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
        <ButtonGroup sx={{ ml: 'auto' }}>
          <Button variant="contained" onClick={() => recordAction('Accepted mask')}>
            Accept mask
          </Button>
          <Button onClick={() => recordAction('Undid last change')}>Undo</Button>
          <Button onClick={() => recordAction('Cleared points')}>Clear</Button>
        </ButtonGroup>
      </Stack>
      <Stack direction="row" flexWrap="wrap" spacing={2}>
        <Button variant="outlined" size="small" onClick={autoAnnotate}>
          Auto annotate text
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
          borderRadius: 3,
          bgColor: 'background.paper',
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
            width={640}
            height={420}
            sx={{ borderRadius: 2, backgroundColor: 'background.default' }}
          />
        </Box>
      </Card>
      <Stack direction={{ xs: 'column', md: 'row' }} spacing={3}>
        <AnnotateInsights />
        <TimelinePanel events={timeline} />
      </Stack>
    </Stack>
  )
}

export default AnnotateTab
