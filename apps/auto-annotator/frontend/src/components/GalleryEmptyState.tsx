import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import { Stack, Typography } from '@mui/material';

export function GalleryEmptyState() {
  return (
    <Stack flex={1} alignItems="center" justifyContent="center" spacing={1} py={6}>
      <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
      <Typography variant="body2" sx={{ color: 'text.disabled', fontWeight: 500 }}>
        No images yet
      </Typography>
      <Typography variant="caption" sx={{ color: 'text.disabled' }}>
        Upload images or refresh to load from the backend
      </Typography>
    </Stack>
  );
}
