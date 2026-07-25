import DeleteIcon from '@mui/icons-material/Delete';
import GridViewIcon from '@mui/icons-material/GridView';
import ImageIcon from '@mui/icons-material/Image';
import ViewListIcon from '@mui/icons-material/ViewList';
import { Button, Card, Stack } from '@mui/material';

export function GalleryToolbar({
  selectedCount,
  viewMode,
  setViewMode,
  onRefresh,
  onUpload,
  onDeleteSelected,
}: {
  selectedCount: number;
  viewMode: 'List' | 'Grid';
  setViewMode: (mode: 'List' | 'Grid') => void;
  onRefresh: () => void;
  onUpload: (files: FileList | null) => void;
  onDeleteSelected: () => void;
}) {
  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack
        direction={{ xs: 'column', md: 'row' }}
        alignItems="center"
        justifyContent="space-between"
        spacing={2}
      >
        <Stack direction="row" spacing={1} alignItems="center">
          <Button variant="outlined" size="small" onClick={onRefresh} startIcon={<ImageIcon />}>
            Refresh
          </Button>
          <Button
            variant="outlined"
            size="small"
            component="label"
            startIcon={<ImageIcon />}
          >
            Upload
            <input type="file" hidden multiple accept="image/*" onChange={(e) => onUpload(e.target.files)} />
          </Button>
          {selectedCount > 0 && (
            <Button
              variant="outlined"
              size="small"
              color="error"
              startIcon={<DeleteIcon />}
              onClick={onDeleteSelected}
            >
              Delete ({selectedCount})
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
  );
}
