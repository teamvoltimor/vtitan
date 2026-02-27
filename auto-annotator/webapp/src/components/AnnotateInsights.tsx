import { Card, Chip, Divider, Stack, Typography } from '@mui/material'
import { useAppState } from '../state/appState'
const detailRows = [
  { label: 'Point mode', getter: (state: string) => state },
  { label: 'Export format', getter: (state: string) => state.toUpperCase() },
  { label: 'Canvas zoom', getter: (state: number) => `${Math.round(state * 100)}%` },
]

const AnnotateInsights = () => {
  const { selectedGalleryItem, pointType, exportFormat, zoom, classes, classColors, logEntries } = useAppState()

  return (
    <Card variant="outlined" sx={{ borderRadius: 3, px: 3, py: 2, minHeight: 150 }}>
      <Stack spacing={1}>
        <Stack direction="row" justifyContent="space-between" alignItems="center">
          <Typography variant="subtitle2" fontWeight={600}>
            Session highlights
          </Typography>
          <Chip
            size="small"
            label={selectedGalleryItem ? 'Drawing ready' : 'Awaiting selection'}
            color={selectedGalleryItem ? 'success' : 'default'}
          />
        </Stack>
        <Divider />
        <Stack direction="row" spacing={1} flexWrap="wrap">
          <Chip label="Auto-validate" size="small" />
          <Chip label="Strict export" size="small" />
        </Stack>
        <Stack spacing={0.5}>
          <Typography variant="caption" color="text.secondary">
            Current focus
          </Typography>
          <Typography variant="body1" fontWeight={500}>
            {selectedGalleryItem ? selectedGalleryItem.label : 'Select a gallery image to begin'}
          </Typography>
        </Stack>
        <Stack direction="row" flexWrap="wrap" spacing={3}>
          {detailRows.map((row) => {
            const value =
              row.label === 'Point mode'
                ? pointType
                : row.label === 'Export format'
                ? exportFormat
                : row.label === 'Canvas zoom'
                ? `${Math.round(zoom * 100)}%`
                : ''
            return (
              <Stack key={row.label} spacing={0.5}>
                <Typography variant="caption" color="text.secondary">
                  {row.label}
                </Typography>
                <Typography variant="body2" fontWeight={600}>
                  {value}
                </Typography>
              </Stack>
            )
          })}
        </Stack>
        <Stack spacing={1}>
          <Typography variant="caption" color="text.secondary">
            Classes
          </Typography>
          <Stack direction="row" spacing={1} flexWrap="wrap">
            {classes.map((cls) => (
              <Chip
                key={cls}
                label={cls}
                size="small"
                sx={{ backgroundColor: classColors[cls] ?? 'rgba(255,255,255,0.1)', color: '#fff' }}
              />
            ))}
          </Stack>
        </Stack>
        <Stack spacing={0.5}>
          <Typography variant="caption" color="text.secondary">
            Recent log
          </Typography>
          {logEntries.map((entry) => (
            <Typography key={entry} variant="body2" fontWeight={500} noWrap>
              {entry}
            </Typography>
          ))}
        </Stack>
      </Stack>
    </Card>
  )
}

export default AnnotateInsights
