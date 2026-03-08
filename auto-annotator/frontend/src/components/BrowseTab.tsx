import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import GridViewIcon from '@mui/icons-material/GridView';
import ImageIcon from '@mui/icons-material/Image';
import ViewListIcon from '@mui/icons-material/ViewList';
import {
  Box,
  Button,
  Card,
  Divider,
  LinearProgress,
  Stack,
  Typography,
  useTheme,
} from '@mui/material';
import { useAppState } from '../state/appState';

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
  } = useAppState();

  const handleUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    importImages(event.target.files);
  };

  const progressValue = Math.min(
    (parseInt(stats.processed) / Math.max(gallery.length, 1)) * 100,
    100
  );

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
            <Stack direction="row" flexWrap="wrap" gap={1.5}>
              {gallery.map((item) => (
                <Card
                  key={item.id}
                  variant="outlined"
                  sx={{
                    width: { xs: '100%', sm: 'calc(50% - 12px)', md: 'calc(33% - 12px)' },
                    cursor: 'pointer',
                    borderColor: selectedGalleryItem?.id === item.id ? 'primary.main' : 'divider',
                    transition: 'border-color 100ms ease',
                  }}
                  onClick={() => setSelectedGalleryItem(item)}
                >
                  <img
                    src={item.src}
                    alt={item.label}
                    style={{ width: '100%', display: 'block', borderRadius: '6px 6px 0 0' }}
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
              ))}
            </Stack>
          ) : (
            <Stack spacing={0.5}>
              {gallery.map((item) => (
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
                    border: `1px solid ${selectedGalleryItem?.id === item.id ? theme.palette.primary.main : 'transparent'}`,
                    bgcolor:
                      selectedGalleryItem?.id === item.id
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
                  <Box
                    sx={{
                      width: 56,
                      height: 36,
                      flexShrink: 0,
                      borderRadius: '4px',
                      overflow: 'hidden',
                    }}
                  >
                    <img
                      src={item.src}
                      alt={item.label}
                      style={{
                        width: '100%',
                        height: '100%',
                        objectFit: 'cover',
                        display: 'block',
                      }}
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
            </Typography>
          )}
        </Stack>
        <Box
          sx={{
            minHeight: 140,
            borderRadius: '6px',
            bgcolor: 'action.hover',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            overflow: 'hidden',
          }}
        >
          {selectedGalleryItem ? (
            <img
              src={selectedGalleryItem.src}
              alt={selectedGalleryItem.label}
              style={{ width: '100%', display: 'block', borderRadius: 6 }}
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
