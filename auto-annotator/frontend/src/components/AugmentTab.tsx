import { Alert, Card, Stack } from '@mui/material';
import { useCallback, useEffect, useRef, useState } from 'react';
import {
  augmentStreamUrl,
  getGroupedGallery,
  startAugment,
  type GroupedGalleryItem,
} from '../api/client';
import { AugmentConfig } from './AugmentConfig';
import { AugmentProgress } from './AugmentProgress';
import { SelectableImageList } from './SelectableImageList';

// eslint-disable-next-line @typescript-eslint/no-unused-vars
const AugmentTab = (_props: { onNavigate?: (tabIndex: number) => void }) => {
  const [groups, setGroups] = useState<GroupedGalleryItem[]>([]);
  const [selected, setSelected] = useState<Set<number>>(new Set());
  const [numAugmentations, setNumAugmentations] = useState(9);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState<{ done: number; total: number; step: string } | null>(
    null
  );
  const [finished, setFinished] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const esRef = useRef<EventSource | null>(null);

  const loadGroups = useCallback(async () => {
    try {
      const items = await getGroupedGallery();
      setGroups(items.filter((g) => g.status === 'done'));
    } catch {
      setError('Failed to load images');
    }
  }, []);

  useEffect(() => {
    void loadGroups();
    return () => esRef.current?.close();
  }, [loadGroups]);

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

  return (
    <Stack spacing={2.5}>
      <AugmentConfig
        numAugmentations={numAugmentations}
        onNumChange={setNumAugmentations}
        selectedCount={selected.size}
        running={running}
        onStart={handleStart}
      />

      <AugmentProgress
        running={running}
        finished={finished}
        done={progress?.done ?? 0}
        total={progress?.total ?? 0}
        step={progress?.step ?? ''}
      />

      {error && <Alert severity="error">{error}</Alert>}

      <Card variant="outlined" sx={{ p: 2 }}>
        <SelectableImageList
          items={doneImages}
          selectedIds={selected}
          running={running}
          onToggle={toggleSelect}
          onSelectAll={selectAll}
          onClearAll={clearAll}
          totalLabel={`${doneImages.length} groups · ${selected.size} selected`}
        />
      </Card>
    </Stack>
  );
};

export default AugmentTab;
