import {
  Box,
  Button,
  Card,
  Divider,
  LinearProgress,
  Stack,
  Typography,
} from '@mui/material'
import ImageIcon from '@mui/icons-material/Image'
import ViewListIcon from '@mui/icons-material/ViewList'
import GridViewIcon from '@mui/icons-material/GridView'
import { useAppState } from '../state/appState'

const BrowseTab = () => {
  const {
    gallery,
    refreshGallery,
    importImages,
    viewMode,
    setViewMode,
    selectedGalleryItem,
    setSelectedGalleryItem,
    stats,
  } = useAppState()

  const handleUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    importImages(event.target.files)
  }

  return (
    <Stack spacing={3} sx={{ height: '100%' }}>
      <Card variant="outlined" sx={{ p: 2, borderRadius: 3 }}>
        <Stack direction={{ xs: 'column', md: 'row' }} alignItems="center" justifyContent="space-between" spacing={2}>
          <Stack direction="row" spacing={1} alignItems="center">
            <Button variant="contained" size="small" onClick={refreshGallery} startIcon={<ImageIcon />}>
              Refresh
            </Button>
            <Button variant="outlined" size="small" component="label" startIcon={<ImageIcon />}>
              Upload
              <input type="file" hidden multiple accept="image/*" onChange={handleUpload} />
            </Button>
          </Stack>
          <Stack direction="row" spacing={1} alignItems="center">
            <Button
              startIcon={<ViewListIcon />}
              variant={viewMode === 'List' ? 'contained' : 'outlined'}
              onClick={() => setViewMode('List')}
              size="small"
            >
              List
            </Button>
            <Button
              startIcon={<GridViewIcon />}
              variant={viewMode === 'Grid' ? 'contained' : 'outlined'}
              onClick={() => setViewMode('Grid')}
              size="small"
            >
              Grid
            </Button>
          </Stack>
        </Stack>
      </Card>
      <Card variant="outlined" sx={{ flex: 1, borderRadius: 3, px: 2, py: 2 }}>
        <Stack spacing={2} sx={{ height: '100%' }}>
          <Stack direction="row" justifyContent="space-between" alignItems="center">
            <Typography variant="subtitle2" fontWeight={600}>
              Gallery overview
            </Typography>
            <Typography variant="body2" color="text.secondary">
              {stats.processed} processed · {stats.skipped} skipped
            </Typography>
          </Stack>
          <Box sx={{ width: '100%' }}>
            <LinearProgress variant="determinate" value={Math.min((parseInt(stats.processed) / 20) * 100, 100)} />
          </Box>
          {viewMode === 'Grid' ? (
            <Stack direction="row" flexWrap="wrap" gap={2}>
              {gallery.map((item) => (
                <Card
                  key={item.id}
                  variant="outlined"
                  sx={{
                    width: { xs: '100%', sm: 'calc(50% - 16px)', md: 'calc(33% - 16px)' },
                    cursor: 'pointer',
                    borderColor: selectedGalleryItem?.id === item.id ? 'primary.main' : 'divider',
                    boxShadow: selectedGalleryItem?.id === item.id ? '0 10px 25px rgba(136, 180, 250, 0.25)' : 'none',
                  }}
                  onClick={() => setSelectedGalleryItem(item)}
                >
                  <img src={item.src} alt={item.label} style={{ width: '100%', borderRadius: 12 }} />
                  <Stack px={2} py={1} spacing={0.5}>
                    <Typography variant="subtitle2">{item.label}</Typography>
                    <Typography variant="caption" color="text.secondary">
                      {item.format.toUpperCase()} · {item.status}
                    </Typography>
                  </Stack>
                </Card>
              ))}
            </Stack>
          ) : (
            <Stack spacing={2}>
              {gallery.map((item) => (
                <Card
                  key={item.id}
                  variant="outlined"
                  sx={{ cursor: 'pointer', borderColor: selectedGalleryItem?.id === item.id ? 'primary.main' : 'divider' }}
                  onClick={() => setSelectedGalleryItem(item)}
                >
                  <Stack direction="row" spacing={2} alignItems="center" sx={{ p: 2 }}>
                    <Box sx={{ width: 110, height: 70 }}>
                      <img src={item.src} alt={item.label} style={{ width: '100%', height: '100%', objectFit: 'cover', borderRadius: 8 }} />
                    </Box>
                    <Stack spacing={0.5} sx={{ flex: 1 }}>
                      <Typography variant="subtitle2">{item.label}</Typography>
                      <Typography variant="caption" color="text.secondary">
                        {item.format.toUpperCase()} · {item.status}
                      </Typography>
                    </Stack>
                    <Typography variant="caption" color="text.secondary">
                      {item.updated}
                    </Typography>
                  </Stack>
                </Card>
              ))}
            </Stack>
          )}
        </Stack>
      </Card>
      <Card variant="outlined" sx={{ p: 2, borderRadius: 3 }}>
        <Stack direction="row" alignItems="center" spacing={2}>
          <Typography variant="subtitle2" fontWeight={600}>
            Preview
          </Typography>
          <Divider sx={{ flex: 1 }} />
        </Stack>
        <Box sx={{ mt: 2, minHeight: 160, borderRadius: 2, bgcolor: 'rgba(255,255,255,0.05)' }}>
          {selectedGalleryItem ? (
            <img src={selectedGalleryItem.src} alt={selectedGalleryItem.label} style={{ width: '100%', borderRadius: 12 }} />
          ) : (
            <Typography variant="body2" color="text.secondary" sx={{ p: 2 }}>
              Select an image to preview
            </Typography>
          )}
        </Box>
      </Card>
    </Stack>
  )
}

export default BrowseTab
