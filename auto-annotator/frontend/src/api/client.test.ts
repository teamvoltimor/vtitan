import { afterEach, describe, expect, it, vi } from 'vitest';
import {
  augmentStreamUrl,
  deleteImages,
  getAnnotations,
  getAugmentStatus,
  getClasses,
  getGallery,
  getGroupedGallery,
  getModels,
  getTrainStatus,
  importGalleryImages,
  saveAnnotations,
  segmentImage,
  skipImage,
  startAugment,
  startTrain,
  trainStreamUrl,
  upsertClass,
} from './client';

// VITE_API_BASE_URL is undefined in the test environment, so resolveBackendUrl
// falls through to DEFAULT_BACKEND_URL ('http://localhost:8000').
const BASE = 'http://localhost:8000/api/v1';

function mockFetch(data: unknown, ok = true) {
  return vi.fn().mockResolvedValue({
    ok,
    status: ok ? 200 : 400,
    statusText: ok ? 'OK' : 'Bad Request',
    json: () => Promise.resolve(data),
    text: () => Promise.resolve(ok ? '' : String(data)),
  });
}

afterEach(() => vi.restoreAllMocks());

describe('GET endpoints', () => {
  it('getGallery → GET /gallery', async () => {
    const fetch = mockFetch({ items: [], stats: {} });
    vi.stubGlobal('fetch', fetch);
    await getGallery();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/gallery`);
  });

  it('getGroupedGallery → GET /gallery/grouped', async () => {
    const fetch = mockFetch([]);
    vi.stubGlobal('fetch', fetch);
    await getGroupedGallery();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/gallery/grouped`);
  });

  it('getClasses → GET /classes', async () => {
    const fetch = mockFetch([]);
    vi.stubGlobal('fetch', fetch);
    await getClasses();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/classes`);
  });

  it('getModels → GET /models', async () => {
    const fetch = mockFetch([]);
    vi.stubGlobal('fetch', fetch);
    await getModels();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/models`);
  });

  it('getAnnotations → GET /annotations/:id', async () => {
    const fetch = mockFetch([]);
    vi.stubGlobal('fetch', fetch);
    await getAnnotations(42);
    expect(fetch).toHaveBeenCalledWith(`${BASE}/annotations/42`);
  });

  it('getAugmentStatus → GET /augment/status', async () => {
    const fetch = mockFetch({ running: false, message: '' });
    vi.stubGlobal('fetch', fetch);
    await getAugmentStatus();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/augment/status`);
  });

  it('getTrainStatus → GET /train/status', async () => {
    const fetch = mockFetch({ running: false, message: '' });
    vi.stubGlobal('fetch', fetch);
    await getTrainStatus();
    expect(fetch).toHaveBeenCalledWith(`${BASE}/train/status`);
  });
});

describe('POST /annotations/* endpoints', () => {
  it('saveAnnotations → POST /annotations/save with imageId, exportFormat, shapes', async () => {
    const fetch = mockFetch({ items: [], stats: {} });
    vi.stubGlobal('fetch', fetch);
    await saveAnnotations(1, 'detection', []);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/annotations/save`,
      expect.objectContaining({
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ imageId: 1, exportFormat: 'detection', shapes: [] }),
      }),
    );
  });

  it('skipImage → POST /annotations/skip with imageId', async () => {
    const fetch = mockFetch({ items: [], stats: {} });
    vi.stubGlobal('fetch', fetch);
    await skipImage(7);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/annotations/skip`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ imageId: 7 }),
      }),
    );
  });
});

describe('POST /segment', () => {
  it('segmentImage → POST /segment with imageId and points', async () => {
    const fetch = mockFetch({ state: 'ready', message: '', shapes: [] });
    vi.stubGlobal('fetch', fetch);
    const points = [{ x: '10', y: '20', pointType: 'positive' as const, className: 'cat' }];
    await segmentImage(3, points);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/segment`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ imageId: 3, points }),
      }),
    );
  });
});

describe('POST /classes', () => {
  it('upsertClass → POST /classes with name and color', async () => {
    const fetch = mockFetch([]);
    vi.stubGlobal('fetch', fetch);
    await upsertClass('dog', '#ff0000');
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/classes`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ name: 'dog', color: '#ff0000' }),
      }),
    );
  });
});

describe('POST /images/delete', () => {
  it('deleteImages → POST /images/delete with imageIds array', async () => {
    const fetch = mockFetch({ items: [], stats: {} });
    vi.stubGlobal('fetch', fetch);
    await deleteImages([1, 2, 3]);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/images/delete`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ imageIds: [1, 2, 3] }),
      }),
    );
  });
});

describe('POST /gallery/import', () => {
  it('importGalleryImages → POST /gallery/import with FormData body', async () => {
    const fetch = mockFetch({ items: [], stats: {} });
    vi.stubGlobal('fetch', fetch);
    const file = new File(['content'], 'test.jpg', { type: 'image/jpeg' });
    // Minimal FileList-compatible iterable (Array.from uses Symbol.iterator).
    const files = {
      0: file,
      length: 1,
      item: (_i: number) => file,
      [Symbol.iterator]: function* () {
        yield file;
      },
    } as unknown as FileList;
    await importGalleryImages(files);
    const [url, opts] = fetch.mock.calls[0] as [string, RequestInit];
    expect(url).toBe(`${BASE}/gallery/import`);
    expect(opts.method).toBe('POST');
    expect(opts.body).toBeInstanceOf(FormData);
  });
});

describe('POST /augment/start and /train/start', () => {
  it('startAugment → POST /augment/start with imageIds and numAugmentations', async () => {
    const fetch = mockFetch({ running: true, message: '' });
    vi.stubGlobal('fetch', fetch);
    await startAugment([4, 5], 10);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/augment/start`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ imageIds: [4, 5], numAugmentations: 10 }),
      }),
    );
  });

  it('startTrain → POST /train/start with training params', async () => {
    const fetch = mockFetch({ running: true, message: '' });
    vi.stubGlobal('fetch', fetch);
    const params = { modelName: 'yolo11n', epochs: 50, batch: 16, imgsz: 640 };
    await startTrain(params);
    expect(fetch).toHaveBeenCalledWith(
      `${BASE}/train/start`,
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify(params),
      }),
    );
  });
});

describe('SSE URL helpers', () => {
  it('augmentStreamUrl → /augment/stream', () => {
    expect(augmentStreamUrl()).toBe(`${BASE}/augment/stream`);
  });

  it('trainStreamUrl → /train/stream', () => {
    expect(trainStreamUrl()).toBe(`${BASE}/train/stream`);
  });
});
