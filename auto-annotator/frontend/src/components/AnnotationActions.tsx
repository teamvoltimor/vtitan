import { Box, Button, ButtonGroup, CircularProgress, Stack, ToggleButton, ToggleButtonGroup } from '@mui/material';

interface AnnotationActionsProps {
  annotationMode: 'auto' | 'manual';
  setAnnotationMode: (mode: 'auto' | 'manual') => void;
  annotationPointsCount: number;
  segmentationStatus: string;
  selectedGalleryItem: boolean;
  segmentationPreviewLength: number;
  onAutoAnnotate: () => void;
  onRunSegmentation: () => void;
  onClearMasks: () => void;
  onAccept: () => void;
  onUndo: () => void;
  onClear: () => void;
  onPrev: () => void;
  onSkip: () => void;
  onSaveNext: () => void;
}

export function AnnotationActions({
  annotationMode,
  setAnnotationMode,
  annotationPointsCount,
  segmentationStatus,
  selectedGalleryItem,
  segmentationPreviewLength,
  onAutoAnnotate,
  onRunSegmentation,
  onClearMasks,
  onAccept,
  onUndo,
  onClear,
  onPrev,
  onSkip,
  onSaveNext,
}: AnnotationActionsProps) {
  const isPending = segmentationStatus === 'pending';

  return (
    <Stack direction="row" spacing={1} alignItems="center" flexWrap="wrap" useFlexGap>
      <Button variant="outlined" size="small" onClick={onAutoAnnotate}>
        Auto annotate
      </Button>
      <ToggleButtonGroup
        value={annotationMode}
        exclusive
        size="small"
        onChange={(_, value: 'auto' | 'manual' | null) => {
          if (value === null) return;
          setAnnotationMode(value);
          if (value === 'auto') onClear();
        }}
        sx={{ height: 30 }}
      >
        <ToggleButton value="auto" sx={{ px: 1.5, fontSize: '0.8125rem', textTransform: 'none' }}>
          Auto
        </ToggleButton>
        <ToggleButton
          value="manual"
          sx={{ px: 1.5, fontSize: '0.8125rem', textTransform: 'none' }}
        >
          Manual
        </ToggleButton>
      </ToggleButtonGroup>
      {annotationMode === 'manual' && (
        <Button
          variant="outlined"
          size="small"
          onClick={onRunSegmentation}
          disabled={isPending || annotationPointsCount === 0 || !selectedGalleryItem}
          startIcon={isPending ? <CircularProgress size={11} color="inherit" /> : null}
        >
          {isPending ? 'Running…' : 'Generate mask'}
        </Button>
      )}
      <Button
        variant="outlined"
        size="small"
        onClick={onClearMasks}
        disabled={segmentationPreviewLength === 0}
      >
        Clear masks
      </Button>
      <ButtonGroup size="small" sx={{ ml: 0.5 }}>
        <Button onClick={onAccept}>Accept</Button>
        <Button
          onClick={onUndo}
          disabled={annotationMode !== 'manual' || annotationPointsCount === 0}
        >
          Undo
        </Button>
        <Button
          onClick={onClear}
          disabled={annotationMode !== 'manual' || annotationPointsCount === 0}
        >
          Clear
        </Button>
      </ButtonGroup>

      <Box sx={{ flex: 1 }} />

      <Button variant="text" size="small" onClick={onPrev} disabled={!selectedGalleryItem}>
        Previous
      </Button>
      <Button variant="text" size="small" onClick={onSkip} disabled={!selectedGalleryItem}>
        Skip
      </Button>
      <Button
        variant="contained"
        size="small"
        onClick={onSaveNext}
        disabled={!selectedGalleryItem || segmentationPreviewLength === 0}
      >
        Save &amp; next
      </Button>
    </Stack>
  );
}
