import { resolveBackendUrl } from '../../config/backend';
import type { components } from './types.generated';

const API_BASE_URL = resolveBackendUrl(import.meta.env.VITE_API_BASE_URL);

// Every domain router is mounted under the versioned /api/v1 prefix on the backend.
const API = `${API_BASE_URL}/api/v1`;

const handleResponse = async <T>(response: Response): Promise<T> => {
  if (!response.ok) {
    const body = await response.text();
    throw new Error(body || response.statusText);
  }
  return response.json();
};

const postJSON = <T>(path: string, body: unknown): Promise<T> =>
  fetch(`${API}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  }).then((r) => handleResponse<T>(r));

// Generated type aliases — single source of truth is api/openapi.yaml
type Schemas = components['schemas'];

export type GalleryItem = Schemas['GalleryItem'];
export type GalleryResponse = Schemas['GalleryResponse'];
export type SegmentationPoint = Schemas['SegmentationPoint'];
export type SegmentationShape = Schemas['Shape'];
export type SegmentationResponse = Schemas['SegmentationResponse'];
export type ClassItem = Schemas['ClassItem'];
export type ModelItem = Schemas['ModelItem'];
export type GroupedGalleryItem = Schemas['ParentImageItem'];
export type JobStatusResponse = Schemas['JobStatusResponse'];

export const getGallery = async (): Promise<GalleryResponse> => {
  const response = await fetch(`${API}/gallery`);
  return handleResponse<GalleryResponse>(response);
};

export const importGalleryImages = async (files: FileList): Promise<GalleryResponse> => {
  const form = new FormData();
  Array.from(files).forEach((file) => form.append('files', file));
  const response = await fetch(`${API}/gallery/import`, {
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
): Promise<GalleryResponse> => postJSON('/annotations/save', { imageId, exportFormat, shapes });

export const skipImage = (imageId: number): Promise<GalleryResponse> =>
  postJSON('/annotations/skip', { imageId });

export const getClasses = (): Promise<ClassItem[]> =>
  fetch(`${API}/classes`).then((r) => handleResponse<ClassItem[]>(r));

export const upsertClass = (name: string, color: string): Promise<ClassItem[]> =>
  postJSON('/classes', { name, color });

export const getModels = (): Promise<ModelItem[]> =>
  fetch(`${API}/models`).then((r) => handleResponse<ModelItem[]>(r));

export const getAnnotations = (imageId: number): Promise<SegmentationShape[]> =>
  fetch(`${API}/annotations/${imageId}`).then((r) => handleResponse<SegmentationShape[]>(r));

export const deleteImages = (imageIds: number[]): Promise<GalleryResponse> =>
  postJSON('/images/delete', { imageIds });

export const getGroupedGallery = (): Promise<GroupedGalleryItem[]> =>
  fetch(`${API}/gallery/grouped`).then((r) => handleResponse<GroupedGalleryItem[]>(r));

export const startAugment = (imageIds: number[], numAugmentations: number): Promise<JobStatusResponse> =>
  postJSON('/augment/start', { imageIds, numAugmentations });

export const getAugmentStatus = (): Promise<JobStatusResponse> =>
  fetch(`${API}/augment/status`).then((r) => handleResponse<JobStatusResponse>(r));

export const startTrain = (params: {
  modelName: string;
  epochs: number;
  batch: number;
  imgsz: number;
}): Promise<JobStatusResponse> => postJSON('/train/start', params);

export const getTrainStatus = (): Promise<JobStatusResponse> =>
  fetch(`${API}/train/status`).then((r) => handleResponse<JobStatusResponse>(r));

export const augmentStreamUrl = (): string => `${API}/augment/stream`;
export const trainStreamUrl = (): string => `${API}/train/stream`;
