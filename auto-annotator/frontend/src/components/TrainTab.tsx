import { Alert, Card, Stack } from '@mui/material';
import { useCallback, useEffect, useRef, useState } from 'react';
import {
  getGroupedGallery,
  startTrain,
  trainStreamUrl,
  type GroupedGalleryItem,
} from '../api/client';
import { DatasetSelector } from './DatasetSelector';
import { TrainingConfig } from './TrainingConfig';
import { TrainingProgress } from './TrainingProgress';

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

  return (
    <Stack spacing={2.5}>
      <TrainingConfig
        modelName={modelName}
        onModelChange={setModelName}
        epochs={epochs}
        onEpochsChange={setEpochs}
        batch={batch}
        onBatchChange={setBatch}
        imgsz={imgsz}
        onImgszChange={setImgsz}
        running={running}
        totalImages={totalImages}
        selectedGroupCount={selectedGroups.size}
        onStart={handleTrain}
      />

      <TrainingProgress
        running={running}
        finished={finished}
        epoch={progress?.epoch ?? 0}
        total={progress?.total ?? 0}
        boxLoss={progress?.box_loss ?? 0}
        clsLoss={progress?.cls_loss ?? 0}
        map50={progress?.map50 ?? 0}
        logs={log}
      />

      {error && <Alert severity="error">{error}</Alert>}

      <Card variant="outlined" sx={{ p: 2 }}>
        <DatasetSelector
          groups={groups}
          selectedIds={selectedGroups}
          running={running}
          onToggle={toggleGroup}
          onSelectAll={selectAll}
          onClearAll={clearAll}
        />
      </Card>
    </Stack>
  );
};

export default TrainTab;
