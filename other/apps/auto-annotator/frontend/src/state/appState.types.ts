import type { GalleryItem as ApiGalleryItem } from '../api/client';

export type PointType = 'positive' | 'negative';
export type ExportFormat = 'segmentation' | 'detection';
export type ViewMode = 'List' | 'Grid';
export type OutlineMode = 'Class color' | 'Neutral';
export type AnnotationMode = 'auto' | 'manual';

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

export interface AppStateContextValue {
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
  selectedGalleryIds: Set<number>;
  toggleGallerySelection: (id: number) => void;
  clearGallerySelection: () => void;
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
  currentPage: number;
  setCurrentPage: (page: number) => void;
  itemsPerPage: number;
  deleteSelectedImages: () => Promise<void>;
}
