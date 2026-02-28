const API_BASE_URL = import.meta.env.VITE_API_BASE_URL ?? 'http://localhost:8000'

const handleResponse = async <T>(response: Response): Promise<T> => {
  if (!response.ok) {
    const body = await response.text()
    throw new Error(body || response.statusText)
  }
  return response.json()
}

export interface GalleryItem {
  id: number
  label: string
  src: string
  format: string
  status: string
  updated: string
}

interface GalleryStats {
  pending: number
  done: number
  skipped: number
  total: number
  pct: number
}

export interface GalleryResponse {
  items: GalleryItem[]
  stats: GalleryStats
}

export type SegmentationPoint = {
  x: number
  y: number
  pointType: 'positive' | 'negative'
  className: string
}

export interface SegmentationShape {
  id: string
  className: string
  points: { x: number; y: number }[]
}

export interface SegmentationResponse {
  state: 'idle' | 'pending' | 'ready' | 'error'
  message: string
  shapes: SegmentationShape[]
}

export const getGallery = async (): Promise<GalleryResponse> => {
  const response = await fetch(`${API_BASE_URL}/gallery`)
  return handleResponse<GalleryResponse>(response)
}

export const importGalleryImages = async (files: FileList): Promise<GalleryResponse> => {
  const form = new FormData()
  Array.from(files).forEach((file) => form.append('files', file))
  const response = await fetch(`${API_BASE_URL}/gallery/import`, {
    method: 'POST',
    body: form,
  })
  return handleResponse<GalleryResponse>(response)
}

export const segmentImage = async (imageId: number, points: SegmentationPoint[]): Promise<SegmentationResponse> => {
  const response = await fetch(`${API_BASE_URL}/segment`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ imageId, points }),
  })
  return handleResponse<SegmentationResponse>(response)
}
