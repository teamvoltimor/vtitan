import ModelTrainingIcon from '@mui/icons-material/ModelTraining';
import { Button, Card, Chip, Stack, TextField, Typography } from '@mui/material';

const YOLO_MODELS = ['yolo11s.pt', 'yolo11n.pt', 'yolo11m.pt', 'yolo11l.pt', 'yolo11x.pt'];

export function TrainingConfig({
  modelName,
  onModelChange,
  epochs,
  onEpochsChange,
  batch,
  onBatchChange,
  imgsz,
  onImgszChange,
  running,
  totalImages,
  selectedGroupCount,
  onStart,
}: {
  modelName: string;
  onModelChange: (v: string) => void;
  epochs: number;
  onEpochsChange: (v: number) => void;
  batch: number;
  onBatchChange: (v: number) => void;
  imgsz: number;
  onImgszChange: (v: number) => void;
  running: boolean;
  totalImages: number;
  selectedGroupCount: number;
  onStart: () => void;
}) {
  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack spacing={2}>
        <Typography variant="subtitle2" fontWeight={600}>
          Training configuration
        </Typography>
        <Stack direction={{ xs: 'column', sm: 'row' }} spacing={2} flexWrap="wrap">
          <Stack spacing={0.5} sx={{ minWidth: 140 }}>
            <Typography variant="caption" color="text.disabled">
              Model
            </Typography>
            <Stack direction="row" spacing={0.5} flexWrap="wrap">
              {YOLO_MODELS.map((m) => (
                <Chip
                  key={m}
                  label={m}
                  size="small"
                  variant={modelName === m ? 'filled' : 'outlined'}
                  color={modelName === m ? 'primary' : 'default'}
                  onClick={() => !running && onModelChange(m)}
                  sx={{ cursor: 'pointer', fontSize: '0.65rem', mb: 0.5 }}
                />
              ))}
            </Stack>
          </Stack>
          <TextField
            label="Epochs"
            type="number"
            value={epochs}
            onChange={(e) => onEpochsChange(Math.max(1, parseInt(e.target.value, 10) || 1))}
            disabled={running}
            size="small"
            sx={{ width: 100 }}
            inputProps={{ min: 1, max: 1000 }}
          />
          <TextField
            label="Batch"
            type="number"
            value={batch}
            onChange={(e) => onBatchChange(Math.max(1, parseInt(e.target.value, 10) || 1))}
            disabled={running}
            size="small"
            sx={{ width: 100 }}
            inputProps={{ min: 1 }}
          />
          <TextField
            label="Image size"
            type="number"
            value={imgsz}
            onChange={(e) => onImgszChange(Math.max(32, parseInt(e.target.value, 10) || 640))}
            disabled={running}
            size="small"
            sx={{ width: 110 }}
            inputProps={{ min: 32, step: 32 }}
          />
        </Stack>
        <Stack direction="row" spacing={1} alignItems="center">
          <Button
            variant="contained"
            startIcon={<ModelTrainingIcon />}
            onClick={onStart}
            disabled={running || selectedGroupCount === 0}
            size="small"
          >
            {running ? 'Training...' : 'Start training'}
          </Button>
          {totalImages > 0 && !running && (
            <Typography variant="caption" color="text.disabled">
              {totalImages} images ({selectedGroupCount} groups)
            </Typography>
          )}
        </Stack>
      </Stack>
    </Card>
  );
}
