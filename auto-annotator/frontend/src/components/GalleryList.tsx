import { Box, Checkbox, Pagination, Stack, Typography, useTheme } from '@mui/material';
import type { GalleryItem } from '../state/appState';
import AnnotatedThumbnail from './AnnotatedThumbnail';

const statusDot: Record<string, string> = {
  done: '#36b37e',
  pending: '#e6a817',
  skipped: '#8a8a96',
};

export function GalleryList({
  items,
  selectedIds,
  selectedItem,
  currentPage,
  totalPages,
  onToggleSelect,
  onSelect,
  onPageChange,
}: {
  items: GalleryItem[];
  selectedIds: Set<number>;
  selectedItem: GalleryItem | null;
  currentPage: number;
  totalPages: number;
  onToggleSelect: (id: number) => void;
  onSelect: (item: GalleryItem) => void;
  onPageChange: (page: number) => void;
}) {
  const theme = useTheme();

  return (
    <Stack spacing={2}>
      <Stack spacing={0.5}>
        {items.map((item) => {
          return (
            <Box
              key={item.id}
              onClick={() => onSelect(item)}
              sx={{
                display: 'flex',
                alignItems: 'center',
                gap: 2,
                px: 1.5,
                py: 1,
                borderRadius: '6px',
                cursor: 'pointer',
                border: `1px solid ${
                  selectedIds.has(item.id) || selectedItem?.id === item.id
                    ? theme.palette.primary.main
                    : 'transparent'
                }`,
                bgcolor: selectedIds.has(item.id)
                  ? theme.palette.mode === 'dark'
                    ? 'rgba(94,106,210,0.15)'
                    : 'rgba(79,92,200,0.08)'
                  : selectedItem?.id === item.id
                    ? theme.palette.mode === 'dark'
                      ? 'rgba(94,106,210,0.08)'
                      : 'rgba(79,92,200,0.05)'
                    : 'transparent',
                transition: 'background-color 80ms ease, border-color 80ms ease',
                '&:hover': {
                  bgcolor:
                    theme.palette.mode === 'dark' ? 'rgba(255,255,255,0.04)' : 'rgba(0,0,0,0.03)',
                },
              }}
            >
              <Checkbox
                checked={selectedIds.has(item.id)}
                onChange={() => onToggleSelect(item.id)}
                onClick={(e) => e.stopPropagation()}
                size="small"
              />
              <Box
                sx={{
                  width: 56,
                  height: 36,
                  flexShrink: 0,
                  borderRadius: '4px',
                  overflow: 'hidden',
                }}
              >
                <AnnotatedThumbnail
                  src={item.thumbSrc || item.src}
                  alt={item.label}
                  annotations={item.annotations ?? []}
                  borderRadius="4px"
                />
              </Box>
              <Stack spacing={0.25} sx={{ flex: 1, minWidth: 0 }}>
                <Typography variant="body2" sx={{ fontWeight: 500 }} noWrap>
                  {item.label}
                </Typography>
                <Stack direction="row" alignItems="center" spacing={0.75}>
                  <Box
                    sx={{
                      width: 6,
                      height: 6,
                      borderRadius: '50%',
                      bgcolor: statusDot[item.status] ?? statusDot.pending,
                      flexShrink: 0,
                    }}
                  />
                  <Typography variant="caption" color="text.disabled">
                    {item.format ? `${item.format.toUpperCase()} · ` : ''}
                    {item.status}
                  </Typography>
                </Stack>
              </Stack>
              <Typography variant="caption" color="text.disabled" sx={{ flexShrink: 0 }}>
                {item.updated}
              </Typography>
            </Box>
          );
        })}
      </Stack>
      {totalPages > 1 && (
        <Stack alignItems="center" pt={1}>
          <Pagination
            count={totalPages}
            page={currentPage + 1}
            onChange={(_, page) => onPageChange(page - 1)}
            size="small"
          />
        </Stack>
      )}
    </Stack>
  );
}
