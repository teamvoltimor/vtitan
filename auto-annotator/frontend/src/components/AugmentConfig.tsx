import AutoFixHighIcon from '@mui/icons-material/AutoFixHigh';
import { Button, Card, Slider, Stack, Typography } from '@mui/material';

export function AugmentConfig({
  numAugmentations,
  onNumChange,
  selectedCount,
  running,
  onStart,
}: {
  numAugmentations: number;
  onNumChange: (value: number) => void;
  selectedCount: number;
  running: boolean;
  onStart: () => void;
}) {
  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack spacing={2}>
        <Typography variant="subtitle2" fontWeight={600}>
          Augmentation settings
        </Typography>
        <Stack spacing={1}>
          <Stack direction="row" justifyContent="space-between">
            <Typography variant="body2">Augmentations per image</Typography>
            <Typography variant="body2" fontWeight={600}>
              {numAugmentations}
            </Typography>
          </Stack>
          <Slider
            value={numAugmentations}
            min={1}
            max={20}
            step={1}
            onChange={(_, v) => onNumChange(v as number)}
            disabled={running}
            size="small"
          />
          <Typography variant="caption" color="text.disabled">
            Transforms: random brightness/contrast, horizontal flip, shift/scale/rotate, random
            crop
          </Typography>
        </Stack>
        <Stack direction="row" spacing={1} alignItems="center">
          <Button
            variant="contained"
            startIcon={<AutoFixHighIcon />}
            onClick={onStart}
            disabled={running || selectedCount === 0}
            size="small"
          >
            {running
              ? 'Augmenting...'
              : `Augment ${selectedCount} group${selectedCount !== 1 ? 's' : ''}`}
          </Button>
          {selectedCount > 0 && !running && (
            <Typography variant="caption" color="text.disabled">
              → {selectedCount * numAugmentations} new images
            </Typography>
          )}
        </Stack>
      </Stack>
    </Card>
  );
}
