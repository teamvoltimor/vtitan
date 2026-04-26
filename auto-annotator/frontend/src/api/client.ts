import { resolveBackendUrl } from '../../config/backend';

const API_BASE_URL = resolveBackendUrl(import.meta.env.VITE_API_BASE_URL);

const handleResponse = async <T>(response: Response): Promise<T> => {
  if (!response.ok) {
    const body = await response.text();
    throw new Error(body || response.statusText);
  }
  return response.json();
};

const postJSON = <T>(path: string, body: unknown): Promise<T> =>
  fetch(`${API_BASE_URL}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  }).then((r) => handleResponse<T>(r));

export interface GalleryItem {
  id: number;
  label: string;
  src: string;
  format: string;
  status: string;
  updated: string;
  annotations: SegmentationShape[];
}

interface GalleryStats {
  pending: number;
  done: number;
  skipped: number;
  total: number;
  pct: number;
}

export interface GalleryResponse {
  items: GalleryItem[];
  stats: GalleryStats;
}

export type SegmentationPoint = {
  x: number;
  y: number;
  pointType: 'positive' | 'negative';
  className: string;
};

export interface SegmentationShape {
  id: string;
  className: string;
  points: { x: number; y: number }[];
}

export interface SegmentationResponse {
  state: 'idle' | 'pending' | 'ready' | 'error';
  message: string;
  shapes: SegmentationShape[];
}

export const getGallery = async (): Promise<GalleryResponse> => {
  const response = await fetch(`${API_BASE_URL}/gallery`);
  return handleResponse<GalleryResponse>(response);
};

export const importGalleryImages = async (files: FileList): Promise<GalleryResponse> => {
  const form = new FormData();
  Array.from(files).forEach((file) => form.append('files', file));
  const response = await fetch(`${API_BASE_URL}/gallery/import`, {
    method: 'POST',
    body: form,
  });
  return handleResponse<GalleryResponse>(response);
};

export const segmentImage = (imageId: number, points: SegmentationPoint[]): Promise<SegmentationResponse> =>
  postJSON('/segment', { imageId, points });

export const saveAnnotations = (
  imageId: number,
  exportFormat: 'segmentation' | 'detection',
  shapes: SegmentationShape[],
): Promise<GalleryResponse> => postJSON('/save', { imageId, exportFormat, shapes });

export const skipImage = (imageId: number): Promise<GalleryResponse> =>
  postJSON('/skip', { imageId });

export interface ClassItem {
  id: number;
  name: string;
  color: string;
}

export interface ModelItem {
  id: string;
  label: string;
}

export const getClasses = (): Promise<ClassItem[]> =>
  fetch(`${API_BASE_URL}/classes`).then((r) => handleResponse<ClassItem[]>(r));

export const upsertClass = (name: string, color: string): Promise<ClassItem[]> =>
  postJSON('/classes', { name, color });

export const getModels = (): Promise<ModelItem[]> =>
  fetch(`${API_BASE_URL}/models`).then((r) => handleResponse<ModelItem[]>(r));

export const getAnnotations = (imageId: number): Promise<SegmentationShape[]> =>
  fetch(`${API_BASE_URL}/annotations/${imageId}`).then((r) => handleResponse<SegmentationShape[]>(r));

export const deleteImages = (imageIds: number[]): Promise<GalleryResponse> =>
  postJSON('/delete', { imageIds });

export interface GroupedGalleryItem {
  id: number;
  label: string;
  src: string;
  format: string;
  status: string;
  updated_at: string;
  aug_count: number;
}

export interface JobStatusResponse {
  running: boolean;
  message: string;
}

export const getGroupedGallery = (): Promise<GroupedGalleryItem[]> =>
  fetch(`${API_BASE_URL}/gallery/grouped`).then((r) => handleResponse<GroupedGalleryItem[]>(r));

export const startAugment = (imageIds: number[], numAugmentations: number): Promise<JobStatusResponse> =>
  postJSON('/augment/start', { imageIds, numAugmentations });

export const getAugmentStatus = (): Promise<JobStatusResponse> =>
  fetch(`${API_BASE_URL}/augment/status`).then((r) => handleResponse<JobStatusResponse>(r));

export const startTrain = (params: {
  modelName: string;
  epochs: number;
  batch: number;
  imgsz: number;
}): Promise<JobStatusResponse> => postJSON('/train/start', params);

export const getTrainStatus = (): Promise<JobStatusResponse> =>
  fetch(`${API_BASE_URL}/train/status`).then((r) => handleResponse<JobStatusResponse>(r));

export const augmentStreamUrl = (): string => `${API_BASE_URL}/augment/stream`;
export const trainStreamUrl = (): string => `${API_BASE_URL}/train/stream`;
