import { useCallback, useEffect, useMemo } from 'react';
import { getAnnotations, getClasses, getModels, type ModelItem } from '../api/client';
import { AppStateContext, useAppState } from './appStateContext';
import { useActivityLog } from './useActivityLog';
import { useClassState } from './useClassState';
import { useGalleryState } from './useGalleryState';
import { useSegmentationState } from './useSegmentationState';
import { useUiSettings } from './useUiSettings';

/* eslint-disable react-refresh/only-export-components */
export type {
  AnnotationMode,
  AnnotationPoint,
  ExportFormat,
  GalleryItem,
  ModelOption,
  OutlineMode,
  PointType,
  SegmentationPreviewShape,
  SegmentationStatus,
  TimelineEvent,
  ViewMode,
} from './appState.types';
export { useAppState };

export const AppProvider = ({ children }: { children: React.ReactNode }) => {
  const uiSettings = useUiSettings();
  const activityLog = useActivityLog();
  const classState = useClassState(activityLog.recordAction);
  const galleryState = useGalleryState(activityLog.recordAction, activityLog.setStats);
  const segmentationState = useSegmentationState(
    activityLog.recordAction,
    galleryState.selectedGalleryItem,
    galleryState.setSelectedGalleryItem,
    uiSettings.exportFormat,
    galleryState.handleGalleryResponse
  );

  const refreshGallery = useCallback(async () => {
    segmentationState.resetSegmentation();
    await galleryState.refreshGalleryData();
  }, [segmentationState.resetSegmentation, galleryState.refreshGalleryData]);

  useEffect(() => {
    const load = async () => {
      try {
        await refreshGallery();
        const [classItems, modelItems] = await Promise.all([getClasses(), getModels()]);
        classState.applyClasses(classItems);
        const opts = modelItems.map((m: ModelItem) => ({ id: m.id, label: m.label }));
        segmentationState.setModels(opts);
        if (opts.length > 0) segmentationState.setSelectedModel(opts[0].id);
      } catch (error) {
        activityLog.recordAction(`Startup failed: ${(error as Error).message}`);
      }
    };
    void load();
  }, [
    refreshGallery,
    classState.applyClasses,
    activityLog.recordAction,
    segmentationState.setModels,
    segmentationState.setSelectedModel,
  ]);

  useEffect(() => {
    const item = galleryState.selectedGalleryItem;
    if (item?.status !== 'done') {
      segmentationState.setSegmentationPreview([]);
      return;
    }
    getAnnotations(item.id)
      .then((shapes) =>
        segmentationState.setSegmentationPreview(
          shapes.map((s) => ({
            id: s.id,
            className: s.className,
            points: s.points.map((p) => ({ x: parseFloat(p.x), y: parseFloat(p.y) })),
          }))
        )
      )
      .catch((err: unknown) => {
        segmentationState.setSegmentationPreview([]);
        activityLog.recordAction(`Failed to load annotations: ${(err as Error).message}`);
      });
  }, [
    galleryState.selectedGalleryItem,
    segmentationState.setSegmentationPreview,
    activityLog.recordAction,
  ]);

  const value = useMemo(
    () => ({
      zoom: uiSettings.zoom,
      setZoom: uiSettings.setZoom,
      pointType: uiSettings.pointType,
      setPointType: uiSettings.setPointType,
      exportFormat: uiSettings.exportFormat,
      setExportFormat: uiSettings.setExportFormat,
      maskLevel: uiSettings.maskLevel,
      setMaskLevel: uiSettings.setMaskLevel,
      activeClass: classState.activeClass,
      setActiveClass: classState.setActiveClass,
      classes: classState.classes,
      classColors: classState.classColors,
      addClass: classState.addClass,
      updateClassColor: classState.updateClassColor,
      logEntries: activityLog.logEntries,
      pushLog: activityLog.pushLog,
      stats: activityLog.stats,
      setStats: activityLog.setStats,
      gallery: galleryState.gallery,
      refreshGallery,
      importImages: galleryState.importImages,
      selectedGalleryItem: galleryState.selectedGalleryItem,
      setSelectedGalleryItem: galleryState.setSelectedGalleryItem,
      selectedGalleryIds: galleryState.selectedGalleryIds,
      toggleGallerySelection: galleryState.toggleGallerySelection,
      clearGallerySelection: galleryState.clearGallerySelection,
      viewMode: galleryState.viewMode,
      setViewMode: galleryState.setViewMode,
      outlineMode: galleryState.outlineMode,
      setOutlineMode: galleryState.setOutlineMode,
      timeline: activityLog.timeline,
      pushTimeline: activityLog.pushTimeline,
      recordAction: activityLog.recordAction,
      models: segmentationState.models,
      selectedModel: segmentationState.selectedModel,
      modelStatus: segmentationState.modelStatus,
      loadModel: segmentationState.loadModel,
      autoAnnotate: segmentationState.autoAnnotate,
      annotationMode: segmentationState.annotationMode,
      setAnnotationMode: segmentationState.setAnnotationMode,
      annotationPoints: segmentationState.annotationPoints,
      addAnnotationPoint: segmentationState.addAnnotationPoint,
      clearAnnotationPoints: segmentationState.clearAnnotationPoints,
      undoAnnotationPoint: segmentationState.undoAnnotationPoint,
      segmentationPreview: segmentationState.segmentationPreview,
      segmentationStatus: segmentationState.segmentationStatus,
      segmentationMessage: segmentationState.segmentationMessage,
      runSegmentationTest: segmentationState.runSegmentationTest,
      segmentFromPoints: segmentationState.segmentFromPoints,
      clearSegmentationPreview: segmentationState.clearSegmentationPreview,
      saveAndNext: segmentationState.saveAndNext,
      skipAndNext: segmentationState.skipAndNext,
      goToPrev: galleryState.goToPrev,
      acceptMask: segmentationState.acceptMask,
      currentPage: galleryState.currentPage,
      setCurrentPage: galleryState.setCurrentPage,
      itemsPerPage: galleryState.itemsPerPage,
      deleteSelectedImages: galleryState.deleteSelectedImages,
    }),
    [
      uiSettings.zoom,
      uiSettings.setZoom,
      uiSettings.pointType,
      uiSettings.setPointType,
      uiSettings.exportFormat,
      uiSettings.setExportFormat,
      uiSettings.maskLevel,
      uiSettings.setMaskLevel,
      classState.activeClass,
      classState.setActiveClass,
      classState.classes,
      classState.classColors,
      classState.addClass,
      classState.updateClassColor,
      activityLog.logEntries,
      activityLog.pushLog,
      activityLog.stats,
      activityLog.setStats,
      activityLog.timeline,
      activityLog.pushTimeline,
      activityLog.recordAction,
      galleryState.gallery,
      refreshGallery,
      galleryState.importImages,
      galleryState.selectedGalleryItem,
      galleryState.setSelectedGalleryItem,
      galleryState.selectedGalleryIds,
      galleryState.toggleGallerySelection,
      galleryState.clearGallerySelection,
      galleryState.viewMode,
      galleryState.setViewMode,
      galleryState.outlineMode,
      galleryState.setOutlineMode,
      galleryState.goToPrev,
      galleryState.currentPage,
      galleryState.setCurrentPage,
      galleryState.itemsPerPage,
      galleryState.deleteSelectedImages,
      segmentationState.models,
      segmentationState.selectedModel,
      segmentationState.modelStatus,
      segmentationState.loadModel,
      segmentationState.autoAnnotate,
      segmentationState.annotationMode,
      segmentationState.setAnnotationMode,
      segmentationState.annotationPoints,
      segmentationState.addAnnotationPoint,
      segmentationState.clearAnnotationPoints,
      segmentationState.undoAnnotationPoint,
      segmentationState.segmentationPreview,
      segmentationState.segmentationStatus,
      segmentationState.segmentationMessage,
      segmentationState.runSegmentationTest,
      segmentationState.segmentFromPoints,
      segmentationState.clearSegmentationPreview,
      segmentationState.saveAndNext,
      segmentationState.skipAndNext,
      segmentationState.acceptMask,
    ]
  );

  return <AppStateContext.Provider value={value}>{children}</AppStateContext.Provider>;
};
