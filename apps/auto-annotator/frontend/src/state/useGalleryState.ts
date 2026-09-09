import { useCallback, useState } from 'react';
import {
  deleteImages as apiDeleteImages,
  type GalleryResponse,
  getGallery,
  importGalleryImages,
} from '../api/client';
import type { GalleryItem, OutlineMode, ViewMode } from './appState.types';

/**
 * Owns the gallery list, selection, pagination, and view/outline mode, plus
 * the API-backed actions that mutate the gallery (refresh, import, delete).
 *
 * `handleGalleryResponse` is exposed because the segmentation slice's
 * save/skip-and-advance flow also needs to apply a fresh `GalleryResponse`
 * after mutating the current image.
 */
export const useGalleryState = (
  recordAction: (label: string) => void,
  setStats: (stats: { processed: string; skipped: string; labels: string }) => void
) => {
  const [gallery, setGallery] = useState<GalleryItem[]>([]);
  const [selectedGalleryItem, setSelectedGalleryItem] = useState<GalleryItem | null>(null);
  const [selectedGalleryIds, setSelectedGalleryIds] = useState<Set<number>>(new Set());
  const [viewMode, setViewMode] = useState<ViewMode>('List');
  const [outlineMode, setOutlineMode] = useState<OutlineMode>('Class color');
  const [currentPage, setCurrentPage] = useState(0);
  const itemsPerPage = 12;

  const handleGalleryResponse = useCallback(
    (response: GalleryResponse) => {
      setGallery(response.items);
      setStats({
        processed: response.stats.done.toString(),
        skipped: response.stats.skipped.toString(),
        labels: response.stats.total.toString(),
      });
      setSelectedGalleryItem((current) => current ?? response.items[0] ?? null);
    },
    [setStats]
  );

  const refreshGalleryData = useCallback(async () => {
    const response = await getGallery();
    handleGalleryResponse(response);
    recordAction('Gallery refreshed');
  }, [recordAction, handleGalleryResponse]);

  const importImages = useCallback(
    async (files: FileList | null) => {
      if (!files || files.length === 0) {
        return;
      }
      try {
        const response = await importGalleryImages(files);
        handleGalleryResponse(response);
        recordAction(`Imported ${files.length} image${files.length > 1 ? 's' : ''}`);
      } catch (error) {
        recordAction(`Import failed: ${(error as Error).message}`);
      }
    },
    [recordAction, handleGalleryResponse]
  );

  const toggleGallerySelection = useCallback((id: number) => {
    setSelectedGalleryIds((prev) => {
      const next = new Set(prev);
      if (next.has(id)) {
        next.delete(id);
      } else {
        next.add(id);
      }
      return next;
    });
  }, []);

  const clearGallerySelection = useCallback(() => {
    setSelectedGalleryIds(new Set());
  }, []);

  const deleteSelectedImages = useCallback(async () => {
    const ids = Array.from(selectedGalleryIds);
    if (ids.length === 0) return;
    try {
      const response = await apiDeleteImages(ids);
      handleGalleryResponse(response);
      clearGallerySelection();
      recordAction(`Deleted ${ids.length} image${ids.length > 1 ? 's' : ''}`);
    } catch (error) {
      recordAction(`Delete failed: ${(error as Error).message}`);
    }
  }, [selectedGalleryIds, handleGalleryResponse, recordAction, clearGallerySelection]);

  const goToPrev = useCallback(() => {
    if (!selectedGalleryItem) return;
    const prev = [...gallery].reverse().find((item) => item.id < selectedGalleryItem.id);
    if (prev) setSelectedGalleryItem(prev);
  }, [selectedGalleryItem, gallery]);

  return {
    gallery,
    selectedGalleryItem,
    setSelectedGalleryItem,
    selectedGalleryIds,
    viewMode,
    setViewMode,
    outlineMode,
    setOutlineMode,
    currentPage,
    setCurrentPage,
    itemsPerPage,
    toggleGallerySelection,
    clearGallerySelection,
    goToPrev,
    handleGalleryResponse,
    refreshGalleryData,
    importImages,
    deleteSelectedImages,
  };
};
