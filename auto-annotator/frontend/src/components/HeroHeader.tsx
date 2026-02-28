import { Box, Button, Divider, Stack, Typography, useTheme } from '@mui/material';
import { useAppState } from '../state/appState';

const heroStats = (
  galleryLength: number,
  statsSummary: { processed: string; skipped: string; labels: string }
) => [
  { label: 'Images ready', value: `${galleryLength}` },
  { label: 'Processed', value: statsSummary.processed },
  { label: 'Labels', value: statsSummary.labels },
];

type HeroHeaderProps = { onLaunchAnnotate?: () => void };

const HeroHeader = ({ onLaunchAnnotate }: HeroHeaderProps) => {
  const theme = useTheme();
  const mode = theme.palette.mode;
  const { stats, gallery } = useAppState();

  return (
    <Box sx={{ mb: 5 }}>
      <Stack
        direction={{ xs: 'column', md: 'row' }}
        alignItems={{ xs: 'flex-start', md: 'flex-end' }}
        justifyContent="space-between"
        gap={2}
        mb={3}
      >
        <Stack spacing={1.25}>
          {/* Badge-style overline */}
          <Box
            sx={{
              display: 'inline-flex',
              alignItems: 'center',
              gap: 0.75,
              px: 1,
              py: 0.375,
              borderRadius: '4px',
              bgcolor: mode === 'dark' ? 'rgba(94,106,210,0.12)' : 'rgba(79,92,200,0.08)',
              border: `1px solid ${
                mode === 'dark' ? 'rgba(94,106,210,0.25)' : 'rgba(79,92,200,0.18)'
              }`,
              alignSelf: 'flex-start',
            }}
          >
            <Box
              sx={{
                width: 6,
                height: 6,
                borderRadius: '50%',
                bgcolor: theme.palette.primary.main,
                flexShrink: 0,
              }}
            />
            <Typography
              sx={{
                fontSize: '0.625rem',
                fontWeight: 600,
                letterSpacing: '0.07em',
                textTransform: 'uppercase',
                color: mode === 'dark' ? 'rgba(165,180,252,0.9)' : theme.palette.primary.main,
                lineHeight: 1,
              }}
            >
              Annotation workspace
            </Typography>
          </Box>

          <Typography variant="h4" sx={{ color: 'text.primary' }}>
            Label with precision.
          </Typography>
          <Typography variant="body2" sx={{ color: 'text.secondary', maxWidth: 380 }}>
            Curate datasets, configure zoom, and publish exports without leaving the workflow.
          </Typography>
        </Stack>

        <Stack direction="row" spacing={1} sx={{ flexShrink: 0 }}>
          <Button variant="contained" onClick={onLaunchAnnotate} size="small">
            Launch Annotate
          </Button>
          <Button variant="outlined" size="small">
            Browse Gallery
          </Button>
        </Stack>
      </Stack>

      <Stack
        direction="row"
        divider={<Divider orientation="vertical" flexItem />}
        sx={{
          borderTop: `1px solid ${theme.palette.divider}`,
          borderBottom: `1px solid ${theme.palette.divider}`,
          py: 2.5,
        }}
      >
        {heroStats(gallery.length, stats).map((stat) => (
          <Box key={stat.label} sx={{ px: 3, '&:first-of-type': { pl: 0 } }}>
            <Typography
              variant="h5"
              sx={{
                color: 'text.primary',
                fontFamily: '"JetBrains Mono", monospace',
                fontVariantNumeric: 'tabular-nums',
                lineHeight: 1.2,
              }}
            >
              {stat.value}
            </Typography>
            <Typography
              variant="caption"
              sx={{ color: 'text.disabled', display: 'block', mt: 0.5 }}
            >
              {stat.label}
            </Typography>
          </Box>
        ))}
      </Stack>
    </Box>
  );
};

export default HeroHeader;
