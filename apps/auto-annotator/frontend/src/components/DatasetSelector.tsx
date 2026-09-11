import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import { Box, Button, Checkbox, Chip, Stack, Typography } from '@mui/material';

interface GroupItem {
  id: number;
  label: string;
  src: string;
  format: string;
  aug_count: number;
}

export function DatasetSelector({
  groups,
  selectedIds,
  running,
  onToggle,
  onSelectAll,
  onClearAll,
}: {
  groups: GroupItem[];
  selectedIds: Set<number>;
  running: boolean;
  onToggle: (id: number) => void;
  onSelectAll: () => void;
  onClearAll: () => void;
}) {
  const totalImages = groups
    .filter((g) => selectedIds.has(g.id))
    .reduce((acc, g) => acc + 1 + g.aug_count, 0);

  return (
    <Stack spacing={1.5}>
      <Stack direction="row" justifyContent="space-between" alignItems="center">
        <Typography variant="subtitle2" fontWeight={600}>
          Dataset — select groups
        </Typography>
        <Stack direction="row" spacing={1} alignItems="center">
          <Typography variant="caption" color="text.disabled">
            {groups.length} groups · {selectedIds.size} selected · {totalImages} images
          </Typography>
          <Button size="small" variant="text" onClick={onSelectAll} disabled={running}>
            All
          </Button>
          <Button size="small" variant="text" onClick={onClearAll} disabled={running}>
            None
          </Button>
        </Stack>
      </Stack>

      {groups.length === 0 ? (
        <Stack alignItems="center" py={6} spacing={1}>
          <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
          <Typography variant="body2" color="text.disabled">
            No annotated image groups. Annotate and augment images first.
          </Typography>
        </Stack>
      ) : (
        <Box sx={{ maxHeight: 420, overflowY: 'auto' }}>
          <Stack spacing={0.5}>
            {groups.map((g) => (
              <Box
                key={g.id}
                onClick={() => !running && onToggle(g.id)}
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: 1.5,
                  px: 1.5,
                  py: 0.75,
                  borderRadius: '6px',
                  cursor: running ? 'default' : 'pointer',
                  border: `1px solid ${selectedIds.has(g.id) ? 'primary.main' : 'transparent'}`,
                  bgcolor: selectedIds.has(g.id) ? 'rgba(94,106,210,0.08)' : 'transparent',
                  transition: 'background-color 80ms ease',
                }}
              >
                <Checkbox
                  checked={selectedIds.has(g.id)}
                  onChange={() => !running && onToggle(g.id)}
                  onClick={(e) => e.stopPropagation()}
                  disabled={running}
                  size="small"
                />
                <Box
                  component="img"
                  src={g.src}
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
                    {g.label}
                  </Typography>
                  <Typography variant="caption" color="text.disabled">
                    {g.format.toUpperCase()} · {1 + g.aug_count} images
                  </Typography>
                </Stack>
                {g.aug_count > 0 && (
                  <Chip
                    label={`+${g.aug_count}`}
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
      )}
    </Stack>
  );
}
