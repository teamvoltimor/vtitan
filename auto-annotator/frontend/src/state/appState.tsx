import { createContext, useCallback, useContext, useEffect, useMemo, useRef, useState } from 'react';

export type PointType = 'positive' | 'negative';
export type ExportFormat = 'segmentation' | 'detection';
export type ViewMode = 'List' | 'Grid';
export type OutlineMode = 'Class color' | 'Neutral';
export type AnnotationMode = 'auto' | 'manual';

import {
  type GalleryItem as ApiGalleryItem,
  type GalleryResponse,
  getGallery,
  importGalleryImages,
  saveAnnotations,
  skipImage as apiSkipImage,
  type SegmentationPoint,
  segmentImage,
  getClasses,
  upsertClass,
  getModels,
  type ClassItem,
  type ModelItem,
} from '../api/client';

/* eslint-disable react-refresh/only-export-components */
export type GalleryItem = ApiGalleryItem;

export interface TimelineEvent {
  time: string;
  label: string;
}

export interface ModelOption {
  id: string;
  label: string;
}

export type SegmentationStatus = 'idle' | 'pending' | 'ready' | 'error';

export type AnnotationPoint = {
  x: number;
  y: number;
  pointType: PointType;
  className: string;
  color: string;
};

export interface SegmentationPreviewShape {
  id: string;
  className: string;
  points: { x: number; y: number }[];
}

interface AppStateContextValue {
  zoom: number;
  setZoom: (value: number) => void;
  pointType: PointType;
  setPointType: (value: PointType) => void;
  exportFormat: ExportFormat;
  setExportFormat: (value: ExportFormat) => void;
  maskLevel: string;
  setMaskLevel: (value: string) => void;
  activeClass: string | null;
  setActiveClass: (value: string | null) => void;
  classes: string[];
  classColors: Record<string, string>;
  addClass: (name: string, color: string) => Promise<void>;
  updateClassColor: (name: string, color: string) => Promise<void>;
  logEntries: string[];
  pushLog: (entry: string) => void;
  stats: { processed: string; skipped: string; labels: string };
  setStats: (stats: AppStateContextValue['stats']) => void;
  gallery: GalleryItem[];
  refreshGallery: () => void;
  importImages: (files: FileList | null) => void;
  selectedGalleryItem: GalleryItem | null;
  setSelectedGalleryItem: (item: GalleryItem | null) => void;
  viewMode: ViewMode;
  setViewMode: (mode: ViewMode) => void;
  outlineMode: OutlineMode;
  setOutlineMode: (mode: OutlineMode) => void;
  timeline: TimelineEvent[];
  pushTimeline: (event: TimelineEvent) => void;
  recordAction: (label: string) => void;
  models: ModelOption[];
  selectedModel: string;
  modelStatus: string;
  loadModel: (modelId: string) => void;
  autoAnnotate: () => void;
  annotationMode: AnnotationMode;
  setAnnotationMode: (mode: AnnotationMode) => void;
  annotationPoints: AnnotationPoint[];
  addAnnotationPoint: (point: AnnotationPoint) => void;
  clearAnnotationPoints: () => void;
  undoAnnotationPoint: () => void;
  segmentationPreview: SegmentationPreviewShape[];
  segmentationStatus: SegmentationStatus;
  segmentationMessage: string;
  runSegmentationTest: () => Promise<void>;
  segmentFromPoints: (points: AnnotationPoint[]) => Promise<void>;
  clearSegmentationPreview: () => void;
  saveAndNext: () => Promise<void>;
  skipAndNext: () => Promise<void>;
  goToPrev: () => void;
  acceptMask: () => void;
}

const AppStateContext = createContext<AppStateContextValue | null>(null);

export const AppProvider = ({ children }: { children: React.ReactNode }) => {
  const [zoom, setZoom] = useState(1);
  const [pointType, setPointType] = useState<PointType>('positive');
  const [exportFormat, setExportFormat] = useState<ExportFormat>('segmentation');
  const [maskLevel, setMaskLevel] = useState('Object (1)');
  const [activeClass, setActiveClass] = useState<string | null>(null);
  const [classes, setClasses] = useState<string[]>([]);
  const [classColors, setClassColors] = useState<Record<string, string>>({});
  const [logEntries, setLogEntries] = useState<string[]>(['Ready.']);
  const [stats, setStats] = useState({ processed: '0', skipped: '0', labels: '0' });
  const [gallery, setGallery] = useState<GalleryItem[]>([]);
  const [selectedGalleryItem, setSelectedGalleryItem] = useState<GalleryItem | null>(null);
  const [viewMode, setViewMode] = useState<ViewMode>('List');
  const [outlineMode, setOutlineMode] = useState<OutlineMode>('Class color');
  const [timeline, setTimeline] = useState<TimelineEvent[]>([]);
  const [models, setModels] = useState<ModelOption[]>([]);
  const [selectedModel, setSelectedModel] = useState('');
  const [modelStatus, setModelStatus] = useState('No model loaded');
  const [annotationMode, setAnnotationMode] = useState<AnnotationMode>('auto');
  const [annotationPoints, setAnnotationPoints] = useState<AnnotationPoint[]>([]);
  const [segmentationPreview, setSegmentationPreview] = useState<SegmentationPreviewShape[]>([]);
  const [segmentationStatus, setSegmentationStatus] = useState<SegmentationStatus>('idle');
  const [segmentationMessage, setSegmentationMessage] = useState('');
  const pendingSegmentationRequests = useRef(0);

  const pushLog = useCallback((entry: string) => {
    setLogEntries((prev) => [entry, ...prev].slice(0, 5));
  }, []);

  const pushTimeline = useCallback((event: TimelineEvent) => {
    setTimeline((prev) => [event, ...prev].slice(0, 6));
  }, []);

  const recordAction = useCallback(
    (label: string) => {
      const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
      pushLog(`${time} · ${label}`);
      pushTimeline({ time, label });
    },
    [pushLog, pushTimeline]
  );

  const startSegmentationRequest = useCallback(() => {
    pendingSegmentationRequests.current += 1;
    setSegmentationStatus('pending');
    setSegmentationMessage('Running SAM ...');
  }, []);

  const completeSegmentationRequest = useCallback(
    (state: SegmentationStatus, message: string) => {
      pendingSegmentationRequests.current = Math.max(pendingSegmentationRequests.current - 1, 0);
      setSegmentationMessage(message);
      if (pendingSegmentationRequests.current === 0) {
        setSegmentationStatus(state);
      }
    },
    []
  );

  const _applyClasses = useCallback((items: ClassItem[]) => {
    setClasses(items.map((c) => c.name));
    setClassColors(
      Object.fromEntries(items.map((c) => [c.name, c.color]))
    );
  }, []);

  const addClass = useCallback(
    async (name: string, color: string) => {
      const items = await upsertClass(name, color);
      _applyClasses(items);
      setActiveClass(name);
      recordAction(`Created class ${name}`);
    },
    [_applyClasses, recordAction],
  );

  const updateClassColor = useCallback(
    async (name: string, color: string) => {
      const items = await upsertClass(name, color);
      _applyClasses(items);
      recordAction(`Updated color for ${name}`);
    },
    [_applyClasses, recordAction],
  );

  const _handleGalleryResponse = useCallback((response: GalleryResponse) => {
    setGallery(response.items);
    setStats({
      processed: response.stats.done.toString(),
      skipped: response.stats.skipped.toString(),
      labels: response.stats.total.toString(),
    });
    setSelectedGalleryItem((current) => current ?? response.items[0] ?? null);
  }, []);

  const refreshGallery = useCallback(async () => {
    setSegmentationPreview([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
    const response = await getGallery();
    _handleGalleryResponse(response);
    recordAction('Gallery refreshed');
  }, [recordAction, _handleGalleryResponse]);

  const importImages = useCallback(
    async (files: FileList | null) => {
      if (!files || files.length === 0) {
        return;
      }
      try {
        const response = await importGalleryImages(files);
        _handleGalleryResponse(response);
        recordAction(`Imported ${files.length} image${files.length > 1 ? 's' : ''}`);
      } catch (error) {
        recordAction(`Import failed: ${(error as Error).message}`);
      }
    },
    [recordAction, _handleGalleryResponse]
  );

  useEffect(() => {
    const load = async () => {
      try {
        await refreshGallery();
        const [classItems, modelItems] = await Promise.all([getClasses(), getModels()]);
        _applyClasses(classItems);
        const opts = modelItems.map((m: ModelItem) => ({ id: m.id, label: m.label }));
        setModels(opts);
        if (opts.length > 0) setSelectedModel(opts[0].id);
      } catch (error) {
        recordAction(`Startup failed: ${(error as Error).message}`);
      }
    };
    void load();
  }, [refreshGallery]);

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

  const addAnnotationPoint = useCallback(
    (point: AnnotationPoint) => {
      setAnnotationPoints((prev) => [...prev, point]);
    },
    []
  );

  const clearAnnotationPoints = useCallback(() => {
    setAnnotationPoints([]);
    recordAction('Cleared point buffer');
  }, [recordAction]);

  const undoAnnotationPoint = useCallback(() => {
    setAnnotationPoints((prev) => prev.slice(0, -1));
    recordAction('Removed last point');
  }, [recordAction]);

  const runSegmentationTest = useCallback(async () => {
    if (!selectedGalleryItem || annotationPoints.length === 0) {
      setSegmentationStatus('error');
      setSegmentationMessage('Select an image and place at least one point');
      return;
    }
    startSegmentationRequest();
    try {
      const payloadPoints: SegmentationPoint[] = annotationPoints.map(
        ({ x, y, pointType, className }) => ({
          x,
          y,
          pointType,
          className,
        })
      );
      const response = await segmentImage(selectedGalleryItem.id, payloadPoints);
      if (response.shapes.length > 0) {
        setSegmentationPreview((prev) => [...prev, ...response.shapes]);
        setAnnotationPoints([]);
      }
      recordAction('Segmentation inference completed');
      completeSegmentationRequest(response.state, response.message);
    } catch (error) {
      completeSegmentationRequest('error', (error as Error).message);
    }
  }, [annotationPoints, completeSegmentationRequest, recordAction, selectedGalleryItem, startSegmentationRequest]);

  const segmentFromPoints = useCallback(
    async (points: AnnotationPoint[]) => {
      if (!selectedGalleryItem || points.length === 0) return;
      startSegmentationRequest();
      try {
        const payloadPoints: SegmentationPoint[] = points.map(({ x, y, pointType, className }) => ({
          x,
          y,
          pointType,
          className,
        }));
        const response = await segmentImage(selectedGalleryItem.id, payloadPoints);
        if (response.shapes.length > 0) {
          setSegmentationPreview((prev) => [...prev, ...response.shapes]);
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

  const clearSegmentationPreview = useCallback(() => {
    setSegmentationPreview([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
    recordAction('Cleared segmentation preview');
  }, [recordAction]);

  const _advanceNext = useCallback(
    (response: GalleryResponse, currentId: number, actionLabel: string) => {
      _handleGalleryResponse(response);
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
    [_handleGalleryResponse, recordAction],
  );

  const saveAndNext = useCallback(async () => {
    if (!selectedGalleryItem || segmentationPreview.length === 0) return;
    const currentId = selectedGalleryItem.id;
    try {
      const response = await saveAnnotations(selectedGalleryItem.id, exportFormat, segmentationPreview);
      _advanceNext(response, currentId, `Saved ${selectedGalleryItem.label}`);
    } catch (error) {
      recordAction(`Save failed: ${(error as Error).message}`);
    }
  }, [selectedGalleryItem, segmentationPreview, exportFormat, _advanceNext, recordAction]);

  const skipAndNext = useCallback(async () => {
    if (!selectedGalleryItem) return;
    const currentId = selectedGalleryItem.id;
    try {
      const response = await apiSkipImage(selectedGalleryItem.id);
      _advanceNext(response, currentId, `Skipped ${selectedGalleryItem.label}`);
    } catch (error) {
      recordAction(`Skip failed: ${(error as Error).message}`);
    }
  }, [selectedGalleryItem, _advanceNext, recordAction]);

  const goToPrev = useCallback(() => {
    if (!selectedGalleryItem) return;
    const prev = [...gallery].reverse().find((item) => item.id < selectedGalleryItem.id);
    if (prev) setSelectedGalleryItem(prev);
  }, [selectedGalleryItem, gallery]);

  const acceptMask = useCallback(() => {
    setAnnotationPoints([]);
    setSegmentationStatus('idle');
    setSegmentationMessage('');
    recordAction('Accepted mask');
  }, [recordAction]);

  const value = useMemo(
    () => ({
      zoom,
      setZoom,
      pointType,
      setPointType,
      exportFormat,
      setExportFormat,
      maskLevel,
      setMaskLevel,
      activeClass,
      setActiveClass,
      classes,
      classColors,
      addClass,
      updateClassColor,
      logEntries,
      pushLog,
      stats,
      setStats,
      gallery,
      refreshGallery,
      importImages,
      selectedGalleryItem,
      setSelectedGalleryItem,
      viewMode,
      setViewMode,
      outlineMode,
      setOutlineMode,
      timeline,
      pushTimeline,
      recordAction,
      models,
      selectedModel,
      modelStatus,
      loadModel,
      autoAnnotate,
      annotationMode,
      setAnnotationMode,
      annotationPoints,
      addAnnotationPoint,
      clearAnnotationPoints,
      undoAnnotationPoint,
      segmentationPreview,
      segmentationStatus,
      segmentationMessage,
      runSegmentationTest,
      segmentFromPoints,
      clearSegmentationPreview,
      saveAndNext,
      skipAndNext,
      goToPrev,
      acceptMask,
    }),
    [
      zoom,
      pointType,
      exportFormat,
      maskLevel,
      activeClass,
      classes,
      classColors,
      logEntries,
      stats,
      gallery,
      selectedGalleryItem,
      viewMode,
      outlineMode,
      timeline,
      models,
      selectedModel,
      modelStatus,
      annotationMode,
      annotationPoints,
      segmentationPreview,
      segmentationStatus,
      segmentationMessage,
      addClass,
      updateClassColor,
      pushLog,
      refreshGallery,
      importImages,
      pushTimeline,
      recordAction,
      loadModel,
      autoAnnotate,
      addAnnotationPoint,
      clearAnnotationPoints,
      undoAnnotationPoint,
      runSegmentationTest,
      segmentFromPoints,
      clearSegmentationPreview,
      saveAndNext,
      skipAndNext,
      goToPrev,
      acceptMask,
      _advanceNext,
    ]
  );

  return <AppStateContext.Provider value={value}>{children}</AppStateContext.Provider>;
};

export const useAppState = () => {
  const stateContext = useContext(AppStateContext);
  if (!stateContext) {
    throw new Error('useAppState must be used within AppProvider');
  }
  return stateContext;
};
