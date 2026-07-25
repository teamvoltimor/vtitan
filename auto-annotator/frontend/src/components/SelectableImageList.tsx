import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import { Box, Button, Checkbox, Chip, Stack, Typography } from '@mui/material';

export interface SelectableImageItem {
  id: number;
  label: string;
  src: string;
  format: string;
  status?: string;
  aug_count?: number;
}

export function SelectableImageList({
  items,
  selectedIds,
  running,
  onToggle,
  onSelectAll,
  onClearAll,
  totalLabel,
}: {
  items: SelectableImageItem[];
  selectedIds: Set<number>;
  running: boolean;
  onToggle: (id: number) => void;
  onSelectAll: () => void;
  onClearAll: () => void;
  totalLabel?: string;
}) {
  if (items.length === 0) {
    return (
      <Stack alignItems="center" py={6} spacing={1}>
        <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
        <Typography variant="body2" color="text.disabled">
          No annotated images. Annotate images first.
        </Typography>
      </Stack>
    );
  }

  return (
    <Stack spacing={1.5}>
      <Stack direction="row" justifyContent="space-between" alignItems="center">
        <Typography variant="subtitle2" fontWeight={600}>
          Annotated images
        </Typography>
        <Stack direction="row" spacing={1} alignItems="center">
          {totalLabel && (
            <Typography variant="caption" color="text.disabled">
              {totalLabel}
            </Typography>
          )}
          <Button size="small" variant="text" onClick={onSelectAll} disabled={running}>
            All
          </Button>
          <Button size="small" variant="text" onClick={onClearAll} disabled={running}>
            None
          </Button>
        </Stack>
      </Stack>

      <Box sx={{ maxHeight: 480, overflowY: 'auto' }}>
        <Stack spacing={0.5}>
          {items.map((item) => (
            <Box
              key={item.id}
              onClick={() => !running && onToggle(item.id)}
              sx={{
                display: 'flex',
                alignItems: 'center',
                gap: 1.5,
                px: 1.5,
                py: 0.75,
                borderRadius: '6px',
                cursor: running ? 'default' : 'pointer',
                border: `1px solid ${selectedIds.has(item.id) ? 'primary.main' : 'transparent'}`,
                bgcolor: selectedIds.has(item.id) ? 'rgba(94,106,210,0.08)' : 'transparent',
                transition: 'background-color 80ms ease',
              }}
            >
              <Checkbox
                checked={selectedIds.has(item.id)}
                onChange={() => !running && onToggle(item.id)}
                onClick={(e) => e.stopPropagation()}
                disabled={running}
                size="small"
              />
              <Box
                component="img"
                src={item.src}
                sx={{
                  width: 48,
                  height: 32,
                  objectFit: 'cover',
                  borderRadius: '3px',
                  flexShrink: 0,
                }}
              />
              <Stack flex={1} minWidth={0}>
                <Typography variant="body2" fontWeight={500} noWrap>
                  {item.label}
                </Typography>
                <Typography variant="caption" color="text.disabled">
                  {item.format.toUpperCase()}
                </Typography>
              </Stack>
              {item.aug_count !== undefined && item.aug_count > 0 && (
                <Chip
                  label={`+${item.aug_count} aug`}
                  size="small"
                  color="success"
                  variant="outlined"
                  sx={{ fontSize: '0.6rem' }}
                />
              )}
            </Box>
          ))}
        </Stack>
      </Box>
    </Stack>
  );
}
