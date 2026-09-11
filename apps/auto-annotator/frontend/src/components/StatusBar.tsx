import { Chip, Stack, Typography } from '@mui/material';

interface StatusBarProps {
  filename: string;
  stats: { processed: string; skipped: string; labels: string };
  segmentationStatus: string;
  segmentationMessage: string;
}

const statsRows: { key: 'processed' | 'skipped' | 'labels'; label: string }[] = [
  { key: 'processed', label: 'Processed' },
  { key: 'skipped', label: 'Skipped' },
  { key: 'labels', label: 'Total' },
];

export function StatusBar({
  filename,
  stats,
  segmentationStatus,
  segmentationMessage,
}: StatusBarProps) {
  const segmentationStatusColor =
    segmentationStatus === 'ready'
      ? 'success'
      : segmentationStatus === 'pending'
        ? 'warning'
        : segmentationStatus === 'error'
          ? 'error'
          : 'default';

  return (
    <Stack
      direction={{ xs: 'column', sm: 'row' }}
      alignItems={{ sm: 'center' }}
      spacing={2}
      useFlexGap
      flexWrap="wrap"
      sx={{ borderTop: (t) => `1px solid ${t.palette.divider}`, pt: 1.5 }}
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
        {filename}
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
  );
}
