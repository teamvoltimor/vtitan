import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';
import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import ModelTrainingIcon from '@mui/icons-material/ModelTraining';
import {
  Alert,
  Box,
  Button,
  Card,
  Checkbox,
  Chip,
  Divider,
  LinearProgress,
  Stack,
  TextField,
  Typography,
} from '@mui/material';
import { useCallback, useEffect, useRef, useState } from 'react';
import {
  getGroupedGallery,
  startTrain,
  trainStreamUrl,
  type GroupedGalleryItem,
} from '../api/client';

const YOLO_MODELS = ['yolo11s.pt', 'yolo11n.pt', 'yolo11m.pt', 'yolo11l.pt', 'yolo11x.pt'];

interface TrainProgress {
  epoch: number;
  total: number;
  box_loss: number;
  cls_loss: number;
  map50: number;
}

// eslint-disable-next-line @typescript-eslint/no-unused-vars
const TrainTab = (_props: { onNavigate?: (tabIndex: number) => void }) => {
  const [groups, setGroups] = useState<GroupedGalleryItem[]>([]);
  const [selectedGroups, setSelectedGroups] = useState<Set<number>>(new Set());
  const [modelName, setModelName] = useState('yolo11s.pt');
  const [epochs, setEpochs] = useState(50);
  const [batch, setBatch] = useState(16);
  const [imgsz, setImgsz] = useState(640);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState<TrainProgress | null>(null);
  const [finished, setFinished] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [log, setLog] = useState<string[]>([]);
  const esRef = useRef<EventSource | null>(null);

  const loadGroups = useCallback(async () => {
    try {
      const items = await getGroupedGallery();
      setGroups(items.filter((g) => g.status === 'done'));
    } catch {
      setError('Failed to load groups');
    }
  }, []);

  useEffect(() => {
    void loadGroups();
    return () => esRef.current?.close();
  }, [loadGroups]);

  const toggleGroup = (id: number) =>
    setSelectedGroups((prev) => {
      const next = new Set(prev);
      next.has(id) ? next.delete(id) : next.add(id);
      return next;
    });

  const selectAll = () => setSelectedGroups(new Set(groups.map((g) => g.id)));
  const clearAll = () => setSelectedGroups(new Set());

  const totalImages = groups
    .filter((g) => selectedGroups.has(g.id))
    .reduce((acc, g) => acc + 1 + g.aug_count, 0);

  const handleTrain = async () => {
    if (running) return;
    setError(null);
    setFinished(false);
    setProgress(null);
    setLog([]);
    setRunning(true);

    const es = new EventSource(trainStreamUrl());
    esRef.current = es;

    es.onmessage = (e) => {
      try {
        const evt = JSON.parse(e.data);
        if (evt.heartbeat) return;
        if (evt.error) {
          setError(evt.error);
          setRunning(false);
          es.close();
          return;
        }
        if (evt.finished) {
          setFinished(true);
          setRunning(false);
          es.close();
          return;
        }
        if (evt.epoch !== undefined) {
          const p: TrainProgress = {
            epoch: evt.epoch,
            total: evt.total,
            box_loss: evt.box_loss ?? 0,
            cls_loss: evt.cls_loss ?? 0,
            map50: evt.map50 ?? 0,
          };
          setProgress(p);
          setLog((prev) =>
            [
              `Epoch ${p.epoch}/${p.total} — box ${p.box_loss.toFixed(4)} cls ${p.cls_loss.toFixed(4)} mAP50 ${p.map50.toFixed(4)}`,
              ...prev,
            ].slice(0, 50)
          );
        }
      } catch {}
    };

    es.onerror = () => {
      setError('Stream disconnected');
      setRunning(false);
      es.close();
    };

    try {
      await startTrain({ modelName, epochs, batch, imgsz });
    } catch (err) {
      setError((err as Error).message);
      setRunning(false);
      es.close();
    }
  };

  const progressPct = progress
    ? Math.round((progress.epoch / Math.max(progress.total, 1)) * 100)
    : 0;

  return (
    <Stack spacing={2.5}>
      {/* Model config */}
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
                    onClick={() => !running && setModelName(m)}
                    sx={{ cursor: 'pointer', fontSize: '0.65rem', mb: 0.5 }}
                  />
                ))}
              </Stack>
            </Stack>
            <TextField
              label="Epochs"
              type="number"
              value={epochs}
              onChange={(e) => setEpochs(Math.max(1, parseInt(e.target.value, 10) || 1))}
              disabled={running}
              size="small"
              sx={{ width: 100 }}
              inputProps={{ min: 1, max: 1000 }}
            />
            <TextField
              label="Batch"
              type="number"
              value={batch}
              onChange={(e) => setBatch(Math.max(1, parseInt(e.target.value, 10) || 1))}
              disabled={running}
              size="small"
              sx={{ width: 100 }}
              inputProps={{ min: 1 }}
            />
            <TextField
              label="Image size"
              type="number"
              value={imgsz}
              onChange={(e) => setImgsz(Math.max(32, parseInt(e.target.value, 10) || 640))}
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
              onClick={handleTrain}
              disabled={running}
              size="small"
            >
              {running
                ? `Training epoch ${progress?.epoch ?? 0}/${progress?.total ?? epochs}...`
                : 'Start training'}
            </Button>
            {totalImages > 0 && !running && (
              <Typography variant="caption" color="text.disabled">
                {totalImages} images ({selectedGroups.size} groups)
              </Typography>
            )}
          </Stack>
        </Stack>
      </Card>

      {/* Progress */}
      {(running || finished) && progress && (
        <Card variant="outlined" sx={{ p: 2 }}>
          <Stack spacing={1.5}>
            <Stack direction="row" justifyContent="space-between" alignItems="center">
              <Typography variant="subtitle2" fontWeight={600}>
                {finished ? 'Training complete' : 'Training'}
              </Typography>
              {finished && <CheckCircleOutlineIcon sx={{ color: 'success.main', fontSize: 18 }} />}
              <Typography variant="caption" color="text.disabled">
                Epoch {progress.epoch} / {progress.total}
              </Typography>
            </Stack>
            <LinearProgress
              variant="determinate"
              value={progressPct}
              color={finished ? 'success' : 'primary'}
            />
            <Stack direction="row" spacing={3}>
              <Stack spacing={0.25}>
                <Typography variant="caption" color="text.disabled">
                  box loss
                </Typography>
                <Typography variant="body2" fontWeight={600}>
                  {progress.box_loss.toFixed(4)}
                </Typography>
              </Stack>
              <Stack spacing={0.25}>
                <Typography variant="caption" color="text.disabled">
                  cls loss
                </Typography>
                <Typography variant="body2" fontWeight={600}>
                  {progress.cls_loss.toFixed(4)}
                </Typography>
              </Stack>
              <Stack spacing={0.25}>
                <Typography variant="caption" color="text.disabled">
                  mAP50
                </Typography>
                <Typography variant="body2" fontWeight={600}>
                  {progress.map50.toFixed(4)}
                </Typography>
              </Stack>
            </Stack>
            {log.length > 0 && (
              <>
                <Divider />
                <Box
                  sx={{
                    fontFamily: '"JetBrains Mono", monospace',
                    fontSize: '0.65rem',
                    color: 'text.disabled',
                    maxHeight: 120,
                    overflowY: 'auto',
                  }}
                >
                  {log.map((l, i) => (
                    // biome-ignore lint/suspicious/noArrayIndexKey: training log lines are plain strings with no stable id and may repeat
                    <div key={i}>{l}</div>
                  ))}
                </Box>
              </>
            )}
          </Stack>
        </Card>
      )}

      {error && <Alert severity="error">{error}</Alert>}

      {/* Group selector */}
      <Card variant="outlined" sx={{ p: 2 }}>
        <Stack spacing={1.5}>
          <Stack direction="row" justifyContent="space-between" alignItems="center">
            <Typography variant="subtitle2" fontWeight={600}>
              Dataset — select groups
            </Typography>
            <Stack direction="row" spacing={1} alignItems="center">
              <Typography variant="caption" color="text.disabled">
                {groups.length} groups · {selectedGroups.size} selected · {totalImages} images
              </Typography>
              <Button size="small" variant="text" onClick={selectAll} disabled={running}>
                All
              </Button>
              <Button size="small" variant="text" onClick={clearAll} disabled={running}>
                None
              </Button>
            </Stack>
          </Stack>

          {groups.length === 0 ? (
            <Stack alignItems="center" py={6} spacing={1}>
              <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
              <Typography variant="body2" color="text.disabled">
                No annotated image groups. Annotate and augment images first.
              </Typography>
            </Stack>
          ) : (
            <Box sx={{ maxHeight: 420, overflowY: 'auto' }}>
              <Stack spacing={0.5}>
                {groups.map((g) => (
                  <Box
                    key={g.id}
                    onClick={() => !running && toggleGroup(g.id)}
                    sx={{
                      display: 'flex',
                      alignItems: 'center',
                      gap: 1.5,
                      px: 1.5,
                      py: 0.75,
                      borderRadius: '6px',
                      cursor: running ? 'default' : 'pointer',
                      border: `1px solid ${selectedGroups.has(g.id) ? 'primary.main' : 'transparent'}`,
                      bgcolor: selectedGroups.has(g.id) ? 'rgba(94,106,210,0.08)' : 'transparent',
                      transition: 'background-color 80ms ease',
                    }}
                  >
                    <Checkbox
                      checked={selectedGroups.has(g.id)}
                      onChange={() => !running && toggleGroup(g.id)}
                      onClick={(e) => e.stopPropagation()}
                      disabled={running}
                      size="small"
                    />
                    <Box
                      component="img"
                      src={g.src}
                      sx={{
                        width: 48,
                        height: 32,
                        objectFit: 'cover',
                        borderRadius: '3px',
                        flexShrink: 0,
                      }}
                    />
                    <Stack flex={1} minWidth={0}>
                      <Typography variant="body2" fontWeight={500} noWrap>
                        {g.label}
                      </Typography>
                      <Typography variant="caption" color="text.disabled">
                        {g.format.toUpperCase()} · {1 + g.aug_count} images
                      </Typography>
                    </Stack>
                    {g.aug_count > 0 && (
                      <Chip
                        label={`+${g.aug_count}`}
                        size="small"
                        color="success"
                        variant="outlined"
                        sx={{ fontSize: '0.6rem' }}
                      />
                    )}
                  </Box>
                ))}
              </Stack>
            </Box>
          )}
        </Stack>
      </Card>
    </Stack>
  );
};

export default TrainTab;
