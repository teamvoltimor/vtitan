import DeleteIcon from '@mui/icons-material/Delete';
import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import GridViewIcon from '@mui/icons-material/GridView';
import ImageIcon from '@mui/icons-material/Image';
import ViewListIcon from '@mui/icons-material/ViewList';
import {
  Box,
  Button,
  Card,
  Checkbox,
  Divider,
  LinearProgress,
  Pagination,
  Stack,
  Typography,
  useTheme,
} from '@mui/material';
import { useCanvasRender } from '../hooks/useCanvasRender';
import { useAppState } from '../state/appState';
import AnnotatedThumbnail from './AnnotatedThumbnail';

const statusDot: Record<string, string> = {
  done: '#36b37e',
  pending: '#e6a817',
  skipped: '#8a8a96',
};

const BrowseTab = () => {
  const theme = useTheme();
  const {
    gallery,
    refreshGallery,
    importImages,
    viewMode,
    setViewMode,
    selectedGalleryItem,
    setSelectedGalleryItem,
    stats,
    segmentationPreview,
    selectedGalleryIds,
    toggleGallerySelection,
    currentPage,
    setCurrentPage,
    itemsPerPage,
    deleteSelectedImages,
  } = useAppState();
  const { canvasRef } = useCanvasRender([], segmentationPreview);

  const handleUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    importImages(event.target.files);
  };

  const progressValue = Math.min(
    (parseInt(stats.processed) / Math.max(gallery.length, 1)) * 100,
    100
  );

  const paginatedGallery = gallery.slice(
    currentPage * itemsPerPage,
    (currentPage + 1) * itemsPerPage
  );
  const totalPages = Math.ceil(gallery.length / itemsPerPage);

  return (
    <Stack spacing={2.5} sx={{ height: '100%' }}>
      {/* Toolbar */}
      <Card variant="outlined" sx={{ p: 2 }}>
        <Stack
          direction={{ xs: 'column', md: 'row' }}
          alignItems="center"
          justifyContent="space-between"
          spacing={2}
        >
          <Stack direction="row" spacing={1} alignItems="center">
            <Button
              variant="outlined"
              size="small"
              onClick={refreshGallery}
              startIcon={<ImageIcon />}
            >
              Refresh
            </Button>
            <Button variant="outlined" size="small" component="label" startIcon={<ImageIcon />}>
              Upload
              <input type="file" hidden multiple accept="image/*" onChange={handleUpload} />
            </Button>
            {selectedGalleryIds.size > 0 && (
              <Button
                variant="outlined"
                size="small"
                color="error"
                startIcon={<DeleteIcon />}
                onClick={deleteSelectedImages}
              >
                Delete ({selectedGalleryIds.size})
              </Button>
            )}
          </Stack>
          <Stack direction="row" spacing={0.5} alignItems="center">
            <Button
              startIcon={<ViewListIcon />}
              variant={viewMode === 'List' ? 'contained' : 'text'}
              onClick={() => setViewMode('List')}
              size="small"
            >
              List
            </Button>
            <Button
              startIcon={<GridViewIcon />}
              variant={viewMode === 'Grid' ? 'contained' : 'text'}
              onClick={() => setViewMode('Grid')}
              size="small"
            >
              Grid
            </Button>
          </Stack>
        </Stack>
      </Card>

      {/* Gallery */}
      <Card variant="outlined" sx={{ flex: 1, px: 2, py: 2 }}>
        <Stack spacing={2} sx={{ height: '100%' }}>
          <Stack direction="row" justifyContent="space-between" alignItems="center">
            <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
              Gallery
            </Typography>
            <Typography variant="caption" color="text.disabled">
              {stats.processed} done · {stats.skipped} skipped · {gallery.length} total
            </Typography>
          </Stack>

          <LinearProgress variant="determinate" value={progressValue} />

          {gallery.length === 0 ? (
            <Stack flex={1} alignItems="center" justifyContent="center" spacing={1} py={6}>
              <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
              <Typography variant="body2" sx={{ color: 'text.disabled', fontWeight: 500 }}>
                No images yet
              </Typography>
              <Typography variant="caption" sx={{ color: 'text.disabled' }}>
                Upload images or refresh to load from the backend
              </Typography>
            </Stack>
          ) : viewMode === 'Grid' ? (
            <Stack spacing={2}>
              <Stack direction="row" flexWrap="wrap" gap={1.5}>
                {paginatedGallery.map((item) => (
                  <Box
                    key={item.id}
                    sx={{
                      width: { xs: '100%', sm: 'calc(50% - 12px)', md: 'calc(33% - 12px)' },
                      position: 'relative',
                    }}
                  >
                    <Checkbox
                      checked={selectedGalleryIds.has(item.id)}
                      onChange={() => toggleGallerySelection(item.id)}
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
                        borderColor:
                          selectedGalleryIds.has(item.id)
                            ? 'primary.main'
                            : selectedGalleryItem?.id === item.id
                              ? 'primary.main'
                              : 'divider',
                        transition: 'border-color 100ms ease',
                        opacity: selectedGalleryIds.has(item.id) ? 0.7 : 1,
                      }}
                      onClick={() => setSelectedGalleryItem(item)}
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
                            {item.format ? `${item.format.toUpperCase()} · ` : ''}{item.status}
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
                    onChange={(_, page) => setCurrentPage(page - 1)}
                    size="small"
                  />
                </Stack>
              )}
            </Stack>
          ) : (
            <Stack spacing={2}>
              <Stack spacing={0.5}>
                {paginatedGallery.map((item) => (
                  <Box
                    key={item.id}
                    onClick={() => setSelectedGalleryItem(item)}
                    sx={{
                      display: 'flex',
                      alignItems: 'center',
                      gap: 2,
                      px: 1.5,
                      py: 1,
                      borderRadius: '6px',
                      cursor: 'pointer',
                      border: `1px solid ${
                        selectedGalleryIds.has(item.id) || selectedGalleryItem?.id === item.id
                          ? theme.palette.primary.main
                          : 'transparent'
                      }`,
                      bgcolor:
                        selectedGalleryIds.has(item.id)
                          ? theme.palette.mode === 'dark'
                            ? 'rgba(94,106,210,0.15)'
                            : 'rgba(79,92,200,0.08)'
                          : selectedGalleryItem?.id === item.id
                            ? theme.palette.mode === 'dark'
                              ? 'rgba(94,106,210,0.08)'
                              : 'rgba(79,92,200,0.05)'
                            : 'transparent',
                      transition: 'background-color 80ms ease, border-color 80ms ease',
                      '&:hover': {
                        bgcolor:
                          theme.palette.mode === 'dark'
                            ? 'rgba(255,255,255,0.04)'
                            : 'rgba(0,0,0,0.03)',
                      },
                    }}
                  >
                    <Checkbox
                      checked={selectedGalleryIds.has(item.id)}
                      onChange={() => toggleGallerySelection(item.id)}
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
                          {item.format ? `${item.format.toUpperCase()} · ` : ''}{item.status}
                        </Typography>
                      </Stack>
                    </Stack>
                    <Typography variant="caption" color="text.disabled" sx={{ flexShrink: 0 }}>
                      {item.updated}
                    </Typography>
                  </Box>
                ))}
              </Stack>
              {totalPages > 1 && (
                <Stack alignItems="center" pt={1}>
                  <Pagination
                    count={totalPages}
                    page={currentPage + 1}
                    onChange={(_, page) => setCurrentPage(page - 1)}
                    size="small"
                  />
                </Stack>
              )}
            </Stack>
          )}
        </Stack>
      </Card>

      {/* Preview */}
      <Card variant="outlined" sx={{ p: 2 }}>
        <Stack direction="row" alignItems="center" spacing={2} mb={1.5}>
          <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
            Preview
          </Typography>
          <Divider sx={{ flex: 1 }} />
          {selectedGalleryItem && (
            <Typography variant="caption" color="text.disabled" noWrap sx={{ maxWidth: 200 }}>
              {selectedGalleryItem.label}
              {selectedGalleryItem.format ? ` · ${selectedGalleryItem.format.toUpperCase()}` : ''}
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
          {selectedGalleryItem ? (
            <canvas
              ref={canvasRef}
              style={{ width: '100%', height: '100%', display: 'block', minHeight: 180 }}
            />
          ) : (
            <Typography variant="caption" color="text.disabled">
              Select an image to preview
            </Typography>
          )}
        </Box>
      </Card>
    </Stack>
  );
};

export default BrowseTab;
