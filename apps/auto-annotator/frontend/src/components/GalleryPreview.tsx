import { Box, Card, Divider, Stack, Typography } from '@mui/material';

export function GalleryPreview({
  selectedLabel,
  children,
}: {
  selectedLabel: string | null;
  children?: React.ReactNode;
}) {
  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack direction="row" alignItems="center" spacing={2} mb={1.5}>
        <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
          Preview
        </Typography>
        <Divider sx={{ flex: 1 }} />
        {selectedLabel && (
          <Typography variant="caption" color="text.disabled" noWrap sx={{ maxWidth: 200 }}>
            {selectedLabel}
          </Typography>
        )}
      </Stack>
      <Box
        sx={{
          position: 'relative',
          minHeight: 180,
          borderRadius: '6px',
          bgcolor: '#10101a',
          overflow: 'hidden',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        {children ?? (
          <Typography variant="caption" color="text.disabled">
            Select an image to preview
          </Typography>
        )}
      </Box>
    </Card>
  );
}
