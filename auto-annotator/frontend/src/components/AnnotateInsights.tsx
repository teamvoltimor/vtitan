import { Box, Card, Chip, Divider, Stack, Typography, useTheme } from '@mui/material';
import { useAppState } from '../state/appState';

const AnnotateInsights = () => {
  const theme = useTheme();
  const mode = theme.palette.mode;
  const { selectedGalleryItem, pointType, exportFormat, zoom, classes, classColors, logEntries } =
    useAppState();

  const detailRows = [
    { label: 'Point mode', value: pointType },
    { label: 'Export', value: exportFormat.toUpperCase() },
    { label: 'Zoom', value: `${Math.round(zoom * 100)}%` },
  ];

  return (
    <Card variant="outlined" sx={{ px: 3, py: 2, flex: 1 }}>
      <Stack spacing={1.5}>
        <Stack direction="row" justifyContent="space-between" alignItems="center">
          <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
            Session highlights
          </Typography>
          <Chip
            size="small"
            label={selectedGalleryItem ? 'Drawing ready' : 'Awaiting'}
            color={selectedGalleryItem ? 'success' : 'default'}
          />
        </Stack>

        <Divider />

        <Stack spacing={0.5}>
          <Typography variant="caption" color="text.secondary">
            Current focus
          </Typography>
          <Typography variant="body2" sx={{ fontWeight: 500 }}>
            {selectedGalleryItem ? selectedGalleryItem.label : 'Select a gallery image to begin'}
          </Typography>
        </Stack>

        <Stack direction="row" spacing={3} flexWrap="wrap">
          {detailRows.map((row) => (
            <Stack key={row.label} spacing={0.25}>
              <Typography variant="caption" color="text.secondary">
                {row.label}
              </Typography>
              <Typography
                variant="body2"
                sx={{ fontWeight: 600, fontFamily: '"JetBrains Mono", monospace' }}
              >
                {row.value}
              </Typography>
            </Stack>
          ))}
        </Stack>

        <Stack spacing={1}>
          <Typography variant="caption" color="text.secondary">
            Classes
          </Typography>
          <Stack direction="row" spacing={0.75} flexWrap="wrap" useFlexGap>
            {classes.map((cls) => (
              <Box
                key={cls}
                sx={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: 0.75,
                  px: 1,
                  py: 0.375,
                  borderRadius: '4px',
                  border: `1px solid ${theme.palette.divider}`,
                  bgcolor: mode === 'dark' ? 'rgba(255,255,255,0.04)' : 'rgba(0,0,0,0.03)',
                }}
              >
                <Box
                  sx={{
                    width: 7,
                    height: 7,
                    borderRadius: '50%',
                    bgcolor: classColors[cls] ?? theme.palette.primary.main,
                    flexShrink: 0,
                  }}
                />
                <Typography variant="caption" sx={{ fontWeight: 500, color: 'text.primary' }}>
                  {cls}
                </Typography>
              </Box>
            ))}
          </Stack>
        </Stack>

        {logEntries.length > 0 && (
          <Stack spacing={0.5}>
            <Typography variant="caption" color="text.secondary">
              Recent log
            </Typography>
            {logEntries.map((entry) => (
              <Typography
                key={entry}
                variant="caption"
                sx={{
                  color: 'text.secondary',
                  fontFamily: '"JetBrains Mono", monospace',
                  display: 'block',
                }}
                noWrap
              >
                {entry}
              </Typography>
            ))}
          </Stack>
        )}
      </Stack>
    </Card>
  );
};

export default AnnotateInsights;
