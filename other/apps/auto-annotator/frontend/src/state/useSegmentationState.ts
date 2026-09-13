import { useCallback, useRef, useState } from 'react';
import {
  skipImage as apiSkipImage,
  type GalleryResponse,
  type SegmentationPoint,
  type SegmentationShape,
  saveAnnotations,
  segmentImage,
} from '../api/client';
import type {
  AnnotationMode,
  AnnotationPoint,
  ExportFormat,
  GalleryItem,
  ModelOption,
  SegmentationPreviewShape,
  SegmentationStatus,
} from './appState.types';

/**
 * Owns model selection, the manual-annotation point buffer, and the
 * segmentation preview/status pipeline, including the save/skip-and-advance
 * flow that moves the user to the next pending gallery image.
 *
 * Gallery mutation is delegated back to the caller via `handleGalleryResponse`
 * and `setSelectedGalleryItem` so this slice never owns gallery state itself.
 */
export const useSegmentationState = (
  recordAction: (label: string) => void,
  selectedGalleryItem: GalleryItem | null,
  setSelectedGalleryItem: (item: GalleryItem | null) => void,
  exportFormat: ExportFormat,
  handleGalleryResponse: (response: GalleryResponse) => void
) => {
  const [models, setModels] = useState<ModelOption[]>([]);
  const [selectedModel, setSelectedModel] = useState('');
  const [modelStatus, setModelStatus] = useState('No model loaded');
  const [annotationMode, setAnnotationMode] = useState<AnnotationMode>('auto');
  const [annotationPoints, setAnnotationPoints] = useState<AnnotationPoint[]>([]);
  const [segmentationPreview, setSegmentationPreview] = useState<SegmentationPreviewShape[]>([]);
  const [segmentationStatus, setSegmentationStatus] = useState<SegmentationStatus>('idle');
  const [segmentationMessage, setSegmentationMessage] = useState('');
  const pendingSegmentationRequests = useRef(0);

  const resetSegmentation = useCallback(() => {
    setSegmentationPreview([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
  }, []);

  const startSegmentationRequest = useCallback(() => {
    pendingSegmentationRequests.current += 1;
    setSegmentationStatus('pending');
    setSegmentationMessage('Running SAM ...');
  }, []);

  const completeSegmentationRequest = useCallback((state: SegmentationStatus, message: string) => {
    pendingSegmentationRequests.current = Math.max(pendingSegmentationRequests.current - 1, 0);
    setSegmentationMessage(message);
    if (pendingSegmentationRequests.current === 0) {
      setSegmentationStatus(state);
    }
  }, []);

  const loadModel = useCallback(
    (modelId: string) => {
      setSelectedModel(modelId);
      setModelStatus(`Loaded ${modelId}`);
      recordAction(`Model ${modelId} loaded`);
    },
    [recordAction]
  );

  const autoAnnotate = useCallback(() => {
    recordAction('Auto-annotated current image (stub)');
  }, [recordAction]);

  const addAnnotationPoint = useCallback((point: AnnotationPoint) => {
    setAnnotationPoints((prev) => [...prev, point]);
  }, []);

  const clearAnnotationPoints = useCallback(() => {
    setAnnotationPoints([]);
    recordAction('Cleared point buffer');
  }, [recordAction]);

  const undoAnnotationPoint = useCallback(() => {
    setAnnotationPoints((prev) => prev.slice(0, -1));
    recordAction('Removed last point');
  }, [recordAction]);

  const runSegmentation = useCallback(
    async (points: AnnotationPoint[]) => {
      if (!selectedGalleryItem || points.length === 0) return;
      startSegmentationRequest();
      try {
        const payloadPoints: SegmentationPoint[] = points.map(({ x, y, pointType, className }) => ({
          x: x.toFixed(6),
          y: y.toFixed(6),
          pointType,
          className,
        }));
        const response = await segmentImage(selectedGalleryItem.id, payloadPoints);
        if (response.shapes.length > 0) {
          setSegmentationPreview((prev) => [
            ...prev,
            ...response.shapes.map((s) => ({
              ...s,
              points: s.points.map((p) => ({ x: parseFloat(p.x), y: parseFloat(p.y) })),
            })),
          ]);
          setAnnotationPoints([]);
        }
        recordAction('Segmentation inference completed');
        completeSegmentationRequest(response.state, response.message);
      } catch (error) {
        completeSegmentationRequest('error', (error as Error).message);
      }
    },
    [completeSegmentationRequest, recordAction, selectedGalleryItem, startSegmentationRequest]
  );

  const runSegmentationTest = useCallback(async () => {
    if (annotationPoints.length === 0) {
      setSegmentationStatus('error');
      setSegmentationMessage('Select an image and place at least one point');
      return;
    }
    await runSegmentation(annotationPoints);
  }, [annotationPoints, runSegmentation]);

  const segmentFromPoints = useCallback(
    (points: AnnotationPoint[]) => runSegmentation(points),
    [runSegmentation]
  );

  const clearSegmentationPreview = useCallback(() => {
    setSegmentationPreview([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
    recordAction('Cleared segmentation preview');
  }, [recordAction]);

  const advanceNext = useCallback(
    (response: GalleryResponse, currentId: number, actionLabel: string) => {
      handleGalleryResponse(response);
      setSegmentationPreview([]);
      setAnnotationPoints([]);
      setSegmentationStatus('idle');
      setSegmentationMessage('');
      const nextItem =
        response.items.find((item) => item.status === 'pending' && item.id > currentId) ??
        response.items.find((item) => item.status === 'pending');
      setSelectedGalleryItem(nextItem ?? null);
      recordAction(actionLabel);
    },
    [handleGalleryResponse, recordAction, setSelectedGalleryItem]
  );

  const saveAndNext = useCallback(async () => {
    if (!selectedGalleryItem || segmentationPreview.length === 0) return;
    const currentId = selectedGalleryItem.id;
    try {
      const apiShapes: SegmentationShape[] = segmentationPreview.map((s) => ({
        id: s.id,
        className: s.className,
        points: s.points.map((p) => ({ x: p.x.toFixed(6), y: p.y.toFixed(6) })),
      }));
      const response = await saveAnnotations(selectedGalleryItem.id, exportFormat, apiShapes);
      advanceNext(response, currentId, `Saved ${selectedGalleryItem.label}`);
    } catch (error) {
      recordAction(`Save failed: ${(error as Error).message}`);
    }
  }, [selectedGalleryItem, segmentationPreview, exportFormat, advanceNext, recordAction]);

  const skipAndNext = useCallback(async () => {
    if (!selectedGalleryItem) return;
    const currentId = selectedGalleryItem.id;
    try {
      const response = await apiSkipImage(selectedGalleryItem.id);
      advanceNext(response, currentId, `Skipped ${selectedGalleryItem.label}`);
    } catch (error) {
      recordAction(`Skip failed: ${(error as Error).message}`);
    }
  }, [selectedGalleryItem, advanceNext, recordAction]);

  const acceptMask = useCallback(() => {
    setAnnotationPoints([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
    recordAction('Accepted mask');
  }, [recordAction]);

  return {
    models,
    setModels,
    selectedModel,
    setSelectedModel,
    modelStatus,
    annotationMode,
    setAnnotationMode,
    annotationPoints,
    addAnnotationPoint,
    clearAnnotationPoints,
    undoAnnotationPoint,
    segmentationPreview,
    setSegmentationPreview,
    segmentationStatus,
    segmentationMessage,
    resetSegmentation,
    loadModel,
    autoAnnotate,
    runSegmentationTest,
    segmentFromPoints,
    clearSegmentationPreview,
    saveAndNext,
    skipAndNext,
    acceptMask,
  };
};
