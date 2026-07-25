import { Box, Card, Checkbox, Pagination, Stack, Typography } from '@mui/material';
import type { GalleryItem } from '../state/appState';
import AnnotatedThumbnail from './AnnotatedThumbnail';

const statusDot: Record<string, string> = {
  done: '#36b37e',
  pending: '#e6a817',
  skipped: '#8a8a96',
};

export function GalleryGrid({
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
  return (
    <Stack spacing={2}>
      <Stack direction="row" flexWrap="wrap" gap={1.5}>
        {items.map((item) => (
          <Box
            key={item.id}
            sx={{
              width: { xs: '100%', sm: 'calc(50% - 12px)', md: 'calc(33% - 12px)' },
              position: 'relative',
            }}
          >
            <Checkbox
              checked={selectedIds.has(item.id)}
              onChange={() => onToggleSelect(item.id)}
              sx={{
                position: 'absolute',
                top: 8,
                left: 8,
                zIndex: 1,
                bgcolor: 'background.paper',
                borderRadius: '4px',
              }}
            />
            <Card
              variant="outlined"
              sx={{
                cursor: 'pointer',
                borderColor: selectedIds.has(item.id)
                  ? 'primary.main'
                  : selectedItem?.id === item.id
                    ? 'primary.main'
                    : 'divider',
                transition: 'border-color 100ms ease',
                opacity: selectedIds.has(item.id) ? 0.7 : 1,
              }}
              onClick={() => onSelect(item)}
            >
              <AnnotatedThumbnail
                src={item.thumbSrc || item.src}
                alt={item.label}
                annotations={item.annotations ?? []}
                borderRadius="6px 6px 0 0"
              />
              <Stack px={1.5} py={1} spacing={0.25}>
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
            </Card>
          </Box>
        ))}
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
