import { Card, LinearProgress, Stack, Typography } from '@mui/material';
import { useCanvasRender } from '../hooks/useCanvasRender';
import { useAppState } from '../state/appState';
import { GalleryEmptyState } from './GalleryEmptyState';
import { GalleryGrid } from './GalleryGrid';
import { GalleryList } from './GalleryList';
import { GalleryPreview } from './GalleryPreview';
import { GalleryToolbar } from './GalleryToolbar';

const BrowseTab = () => {
  const {
    gallery,
    refreshGallery,
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

  const progressValue = Math.min(
    (parseInt(stats.processed, 10) / Math.max(gallery.length, 1)) * 100,
    100
  );

  const paginatedGallery = gallery.slice(
    currentPage * itemsPerPage,
    (currentPage + 1) * itemsPerPage
  );
  const totalPages = Math.ceil(gallery.length / itemsPerPage);

  return (
    <Stack spacing={2.5} sx={{ height: '100%' }}>
      <GalleryToolbar
        selectedCount={selectedGalleryIds.size}
        viewMode={viewMode}
        setViewMode={setViewMode}
        onRefresh={refreshGallery}
        onUpload={importImages}
        onDeleteSelected={deleteSelectedImages}
      />

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
            <GalleryEmptyState />
          ) : viewMode === 'Grid' ? (
            <GalleryGrid
              items={paginatedGallery}
              selectedIds={selectedGalleryIds}
              selectedItem={selectedGalleryItem}
              currentPage={currentPage}
              totalPages={totalPages}
              onToggleSelect={toggleGallerySelection}
              onSelect={setSelectedGalleryItem}
              onPageChange={setCurrentPage}
            />
          ) : (
            <GalleryList
              items={paginatedGallery}
              selectedIds={selectedGalleryIds}
              selectedItem={selectedGalleryItem}
              currentPage={currentPage}
              totalPages={totalPages}
              onToggleSelect={toggleGallerySelection}
              onSelect={setSelectedGalleryItem}
              onPageChange={setCurrentPage}
            />
          )}
        </Stack>
      </Card>

      <GalleryPreview selectedLabel={selectedGalleryItem?.label ?? null}>
        {selectedGalleryItem ? (
          <canvas
            ref={canvasRef}
            style={{ width: '100%', height: '100%', display: 'block', minHeight: 180 }}
          />
        ) : null}
      </GalleryPreview>
    </Stack>
  );
};

export default BrowseTab;
