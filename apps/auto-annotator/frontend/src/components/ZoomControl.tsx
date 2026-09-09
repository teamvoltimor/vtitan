import type { SelectChangeEvent } from '@mui/material';
import {
  Box,
  Button,
  Divider,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Slider,
  Stack,
  Typography,
} from '@mui/material';
import { useMemo } from 'react';
import type { ExportFormat, PointType } from '../state/appState';

const pointOptions: { label: string; value: PointType }[] = [
  { label: 'Positive', value: 'positive' },
  { label: 'Negative', value: 'negative' },
];

const exportOptions: { label: string; value: ExportFormat }[] = [
  { label: 'Segmentation', value: 'segmentation' },
  { label: 'Detection', value: 'detection' },
];

export function ZoomControl({
  zoom,
  setZoom,
  recordAction,
}: {
  zoom: number;
  setZoom: (value: number) => void;
  recordAction: (label: string) => void;
}) {
  const zoomLabel = useMemo(() => `${Math.round(zoom * 100)}%`, [zoom]);

  return (
    <Stack gap={2} direction={{ xs: 'column', sm: 'row' }} alignItems="center">
      <Stack spacing={0.5} sx={{ flex: 1 }}>
        <Typography variant="caption" sx={{ color: 'text.secondary' }}>
          Zoom{' '}
          <Box
            component="span"
            sx={{
              fontFamily: '"JetBrains Mono", monospace',
              color: 'text.primary',
              fontVariantNumeric: 'tabular-nums',
            }}
          >
            {zoomLabel}
          </Box>
        </Typography>
        <Slider
          value={zoom}
          onChange={(_, value) => setZoom(value as number)}
          onChangeCommitted={(_, value) =>
            recordAction(`Set zoom to ${Math.round((value as number) * 100)}%`)
          }
          min={0.5}
          max={2}
          step={0.05}
          marks={[
            { value: 0.5, label: '50%' },
            { value: 1, label: '100%' },
            { value: 1.5, label: '150%' },
          ]}
        />
      </Stack>
      <Button variant="outlined" size="small" sx={{ flexShrink: 0 }} onClick={() => setZoom(1)}>
        Reset
      </Button>
    </Stack>
  );
}

export function AnnotationToolbar({
  pointType,
  setPointType,
  maskLevel,
  setMaskLevel,
  exportFormat,
  setExportFormat,
  activeClass,
  setActiveClass,
  classes,
  classColors,
  recordAction,
}: {
  pointType: PointType;
  setPointType: (value: PointType) => void;
  maskLevel: string;
  setMaskLevel: (value: string) => void;
  exportFormat: ExportFormat;
  setExportFormat: (value: ExportFormat) => void;
  activeClass: string | null;
  setActiveClass: (value: string | null) => void;
  classes: string[];
  classColors: Record<string, string>;
  recordAction: (label: string) => void;
}) {
  const classLabel = activeClass ?? classes[0] ?? 'No class';

  return (
    <Stack direction={{ xs: 'column', md: 'row' }} gap={1.5} flexWrap="wrap">
      <FormControl sx={{ minWidth: 160 }} size="small">
        <InputLabel>Point type</InputLabel>
        <Select
          value={pointType}
          label="Point type"
          onChange={(event: SelectChangeEvent<PointType>) => setPointType(event.target.value)}
        >
          {pointOptions.map((option) => (
            <MenuItem key={option.value} value={option.value}>
              {option.label}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
      <FormControl sx={{ minWidth: 160 }} size="small">
        <InputLabel>Mask granularity</InputLabel>
        <Select
          value={maskLevel}
          label="Mask granularity"
          onChange={(event: SelectChangeEvent<string>) => setMaskLevel(event.target.value)}
        >
          <MenuItem value="Object (1)">Object (1)</MenuItem>
          <MenuItem value="Instance">Instance</MenuItem>
          <MenuItem value="Fine detail">Fine detail</MenuItem>
        </Select>
      </FormControl>
      <FormControl sx={{ minWidth: 160 }} size="small">
        <InputLabel>Export format</InputLabel>
        <Select
          value={exportFormat}
          label="Export format"
          onChange={(event: SelectChangeEvent<ExportFormat>) => setExportFormat(event.target.value)}
        >
          {exportOptions.map((option) => (
            <MenuItem key={option.value} value={option.value}>
              {option.label}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
      <FormControl sx={{ minWidth: 160 }} size="small">
        <InputLabel>Active class</InputLabel>
        <Select
          value={classLabel}
          label="Active class"
          onChange={(event: SelectChangeEvent<string>) => {
            setActiveClass(event.target.value);
            recordAction(`Switched to class ${event.target.value}`);
          }}
        >
          {classes.map((cls) => (
            <MenuItem key={cls} value={cls}>
              <Stack direction="row" spacing={1} alignItems="center">
                <Box
                  sx={{
                    width: 8,
                    height: 8,
                    borderRadius: '50%',
                    backgroundColor: classColors[cls] ?? 'transparent',
                    flexShrink: 0,
                  }}
                />
                <Typography variant="body2">{cls}</Typography>
              </Stack>
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </Stack>
  );
}

export function ZoomToolbarSection(props: {
  zoom: number;
  setZoom: (value: number) => void;
  recordAction: (label: string) => void;
  pointType: PointType;
  setPointType: (value: PointType) => void;
  maskLevel: string;
  setMaskLevel: (value: string) => void;
  exportFormat: ExportFormat;
  setExportFormat: (value: ExportFormat) => void;
  activeClass: string | null;
  setActiveClass: (value: string | null) => void;
  classes: string[];
  classColors: Record<string, string>;
}) {
  return (
    <Box
      sx={{
        display: 'flex',
        flexDirection: 'column',
        gap: 1.5,
        p: 1.5,
        borderRadius: '6px',
        border: (t: { palette: { divider: string } }) => `1px solid ${t.palette.divider}`,
        bgcolor: 'background.paper',
      }}
    >
      <ZoomControl zoom={props.zoom} setZoom={props.setZoom} recordAction={props.recordAction} />
      <Divider />
      <AnnotationToolbar
        pointType={props.pointType}
        setPointType={props.setPointType}
        maskLevel={props.maskLevel}
        setMaskLevel={props.setMaskLevel}
        exportFormat={props.exportFormat}
        setExportFormat={props.setExportFormat}
        activeClass={props.activeClass}
        setActiveClass={props.setActiveClass}
        classes={props.classes}
        classColors={props.classColors}
        recordAction={props.recordAction}
      />
    </Box>
  );
}
