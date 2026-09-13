import { useState } from 'react';
import type { ExportFormat, PointType } from './appState.types';

/**
 * Simple, self-contained UI preference state (zoom, point type, export
 * format, mask level) with no dependencies on other state slices.
 */
export const useUiSettings = () => {
  const [zoom, setZoom] = useState(1);
  const [pointType, setPointType] = useState<PointType>('positive');
  const [exportFormat, setExportFormat] = useState<ExportFormat>('segmentation');
  const [maskLevel, setMaskLevel] = useState('Object (1)');

  return {
    zoom,
    setZoom,
    pointType,
    setPointType,
    exportFormat,
    setExportFormat,
    maskLevel,
    setMaskLevel,
  };
};
