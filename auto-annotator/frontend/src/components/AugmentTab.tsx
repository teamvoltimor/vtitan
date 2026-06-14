import AutoFixHighIcon from '@mui/icons-material/AutoFixHigh';
import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';
import FolderOpenOutlinedIcon from '@mui/icons-material/FolderOpenOutlined';
import {
  Alert,
  Box,
  Button,
  Card,
  Checkbox,
  Chip,
  LinearProgress,
  Slider,
  Stack,
  Typography,
} from '@mui/material';
import { useEffect, useRef, useState } from 'react';
import {
  augmentStreamUrl,
  getGroupedGallery,
  startAugment,
  type GroupedGalleryItem,
} from '../api/client';

// eslint-disable-next-line @typescript-eslint/no-unused-vars
const AugmentTab = (_props: { onNavigate?: (tabIndex: number) => void }) => {
  const [groups, setGroups] = useState<GroupedGalleryItem[]>([]);
  const [selected, setSelected] = useState<Set<number>>(new Set());
  const [numAugmentations, setNumAugmentations] = useState(9);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState<{ done: number; total: number; step: string } | null>(null);
  const [finished, setFinished] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const esRef = useRef<EventSource | null>(null);

  const loadGroups = async () => {
    try {
      const items = await getGroupedGallery();
      setGroups(items.filter((g) => g.status === 'done'));
    } catch {
      setError('Failed to load images');
    }
  };

  useEffect(() => {
    void loadGroups();
    return () => esRef.current?.close();
  }, []);

  const toggleSelect = (id: number) =>
    setSelected((prev) => {
      const next = new Set(prev);
      next.has(id) ? next.delete(id) : next.add(id);
      return next;
    });

  const selectAll = () => setSelected(new Set(groups.map((g) => g.id)));
  const clearAll = () => setSelected(new Set());

  const handleStart = async () => {
    if (selected.size === 0 || running) return;
    setError(null);
    setFinished(false);
    setProgress({ done: 0, total: selected.size * numAugmentations, step: '' });
    setRunning(true);

    const es = new EventSource(augmentStreamUrl());
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
          void loadGroups();
          return;
        }
        setProgress({ done: evt.done ?? 0, total: evt.total ?? 1, step: evt.step ?? '' });
      } catch (parseErr) {
        console.error('SSE parse error', parseErr);
      }
    };

    es.onerror = () => {
      setError('Stream disconnected');
      setRunning(false);
      es.close();
    };

    try {
      await startAugment(Array.from(selected), numAugmentations);
    } catch (err) {
      setError((err as Error).message);
      setRunning(false);
      es.close();
    }
  };

  const doneImages = groups.filter((g) => g.status === 'done');
  const progressPct = progress ? Math.round((progress.done / Math.max(progress.total, 1)) * 100) : 0;

  return (
    <Stack spacing={2.5}>
      {/* Config card */}
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
              onChange={(_, v) => setNumAugmentations(v as number)}
              disabled={running}
              size="small"
            />
            <Typography variant="caption" color="text.disabled">
              Transforms: random brightness/contrast, horizontal flip, shift/scale/rotate, random crop
            </Typography>
          </Stack>
          <Stack direction="row" spacing={1} alignItems="center">
            <Button
              variant="contained"
              startIcon={<AutoFixHighIcon />}
              onClick={handleStart}
              disabled={running || selected.size === 0}
              size="small"
            >
              {running ? 'Augmenting...' : `Augment ${selected.size} group${selected.size !== 1 ? 's' : ''}`}
            </Button>
            {selected.size > 0 && !running && (
              <Typography variant="caption" color="text.disabled">
                → {selected.size * numAugmentations} new images
              </Typography>
            )}
          </Stack>
        </Stack>
      </Card>

      {/* Progress */}
      {(running || finished) && (
        <Card variant="outlined" sx={{ p: 2 }}>
          <Stack spacing={1}>
            <Stack direction="row" justifyContent="space-between" alignItems="center">
              <Typography variant="subtitle2" fontWeight={600}>
                {finished ? 'Done' : 'Progress'}
              </Typography>
              {finished && <CheckCircleOutlineIcon sx={{ color: 'success.main', fontSize: 18 }} />}
              <Typography variant="caption" color="text.disabled">
                {progress?.done ?? 0} / {progress?.total ?? 0}
              </Typography>
            </Stack>
            <LinearProgress variant="determinate" value={progressPct} color={finished ? 'success' : 'primary'} />
            {progress?.step && (
              <Typography variant="caption" color="text.disabled" noWrap>
                {progress.step}
              </Typography>
            )}
          </Stack>
        </Card>
      )}

      {error && <Alert severity="error">{error}</Alert>}

      {/* Image group list */}
      <Card variant="outlined" sx={{ p: 2 }}>
        <Stack spacing={1.5}>
          <Stack direction="row" justifyContent="space-between" alignItems="center">
            <Typography variant="subtitle2" fontWeight={600}>
              Annotated images
            </Typography>
            <Stack direction="row" spacing={1} alignItems="center">
              <Typography variant="caption" color="text.disabled">
                {doneImages.length} groups · {selected.size} selected
              </Typography>
              <Button size="small" variant="text" onClick={selectAll} disabled={running}>
                All
              </Button>
              <Button size="small" variant="text" onClick={clearAll} disabled={running}>
                None
              </Button>
            </Stack>
          </Stack>

          {doneImages.length === 0 ? (
            <Stack alignItems="center" py={6} spacing={1}>
              <FolderOpenOutlinedIcon sx={{ fontSize: 36, color: 'text.disabled' }} />
              <Typography variant="body2" color="text.disabled">
                No annotated images. Annotate images first.
              </Typography>
            </Stack>
          ) : (
            <Box sx={{ maxHeight: 480, overflowY: 'auto' }}>
              <Stack spacing={0.5}>
                {doneImages.map((item) => (
                  <Box
                    key={item.id}
                    onClick={() => !running && toggleSelect(item.id)}
                    sx={{
                      display: 'flex',
                      alignItems: 'center',
                      gap: 1.5,
                      px: 1.5,
                      py: 0.75,
                      borderRadius: '6px',
                      cursor: running ? 'default' : 'pointer',
                      border: `1px solid ${selected.has(item.id) ? 'primary.main' : 'transparent'}`,
                      bgcolor: selected.has(item.id) ? 'rgba(94,106,210,0.08)' : 'transparent',
                      transition: 'background-color 80ms ease',
                    }}
                  >
                    <Checkbox
                      checked={selected.has(item.id)}
                      onChange={() => !running && toggleSelect(item.id)}
                      onClick={(e) => e.stopPropagation()}
                      disabled={running}
                      size="small"
                    />
                    <Box
                      component="img"
                      src={item.src}
                      sx={{ width: 48, height: 32, objectFit: 'cover', borderRadius: '3px', flexShrink: 0 }}
                    />
                    <Stack flex={1} minWidth={0}>
                      <Typography variant="body2" fontWeight={500} noWrap>
                        {item.label}
                      </Typography>
                      <Typography variant="caption" color="text.disabled">
                        {item.format.toUpperCase()}
                      </Typography>
                    </Stack>
                    {item.aug_count > 0 && (
                      <Chip label={`+${item.aug_count} aug`} size="small" color="success" variant="outlined" sx={{ fontSize: '0.6rem' }} />
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

export default AugmentTab;
