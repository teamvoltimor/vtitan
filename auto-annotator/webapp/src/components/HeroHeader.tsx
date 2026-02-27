import { Box, Button, Chip, Stack, Typography, useTheme } from '@mui/material'
import { useAppState } from '../state/appState'

const heroStats = (galleryLength: number, statsSummary: { processed: string; skipped: string; labels: string }) => [
  { label: 'Images ready', value: `${galleryLength}` },
  { label: 'Processed', value: statsSummary.processed },
  { label: 'Labels', value: statsSummary.labels },
]

type HeroHeaderProps = {
  onLaunchAnnotate?: () => void
}

const HeroHeader = ({ onLaunchAnnotate }: HeroHeaderProps) => {
  const theme = useTheme()
  const { stats, gallery } = useAppState()

  return (
    <Box
      sx={{
        borderRadius: 3,
        background: `linear-gradient(135deg, ${theme.palette.primary.main} 0%, ${theme.palette.secondary.main} 55%, rgba(255,255,255,0) 100%)`,
        px: { xs: 3, sm: 4 },
        py: { xs: 3, sm: 4 },
        color: '#fff',
        boxShadow: '0 25px 60px rgba(14, 14, 44, 0.55)',
      }}
    >
      <Stack direction={{ xs: 'column', md: 'row' }} alignItems="flex-start" justifyContent="space-between" gap={2}>
        <Stack spacing={1} maxWidth={460}>
          <Typography variant="overline" letterSpacing={1} fontWeight={600} color="rgba(255,255,255,0.8)">
            Auto Annotator
          </Typography>
          <Typography variant="h4" fontWeight={700} sx={{ lineHeight: 1.2 }}>
            Tight canvas controls, laser-focused gallery, and settings that feel like Linear.
          </Typography>
          <Typography variant="body2" color="rgba(255,255,255,0.85)">
            Curate datasets, dial in zoom, and publish exports without leaving the minimalist workflow.
          </Typography>
          <Stack direction="row" spacing={1} flexWrap="wrap">
            <Chip label="Catppuccin" size="small" color="secondary" />
            <Chip label="Zoom aware" size="small" color="secondary" />
            <Chip label="Scroll-friendly canvas" size="small" color="secondary" />
          </Stack>
        </Stack>
        <Stack direction="row" spacing={1} flexWrap="wrap" sx={{ width: { xs: '100%', md: 'auto' } }}>
          <Button variant="contained" onClick={onLaunchAnnotate} size="large" disableElevation>
            Launch Annotate
          </Button>
          <Button variant="outlined" size="large" sx={{ color: '#fff', borderColor: 'rgba(255,255,255,0.7)' }}>
            View Gallery
          </Button>
        </Stack>
      </Stack>
      <Stack direction={{ xs: 'column', md: 'row' }} spacing={2} mt={3}>
        {heroStats(gallery.length, stats).map((stat) => (
          <Box key={stat.label} sx={{ flex: 1, p: 2, borderRadius: 2, backgroundColor: 'rgba(0,0,0,0.2)' }}>
            <Typography variant="caption" color="rgba(255,255,255,0.8)">
              {stat.label}
            </Typography>
            <Typography variant="h5" fontWeight={700}>
              {stat.value}
            </Typography>
          </Box>
        ))}
      </Stack>
    </Box>
  )
}

export default HeroHeader
