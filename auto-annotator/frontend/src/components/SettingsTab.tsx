import {
  Box,
  Button,
  Card,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Stack,
  Switch,
  Typography,
  useTheme,
} from '@mui/material';
import type { OutlineMode } from '../state/appState';
import { useAppState } from '../state/appState';

const SettingsTab = () => {
  const theme = useTheme();
  const {
    models,
    selectedModel,
    modelStatus,
    loadModel,
    classes,
    classColors,
    addClass,
    updateClassColor,
    outlineMode,
    setOutlineMode,
  } = useAppState();

  return (
    <Stack spacing={2.5} sx={{ height: '100%' }}>
      {/* Models */}
      <Card variant="outlined" sx={{ p: 3 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
            SAM Model
          </Typography>
          <FormControl sx={{ minWidth: 240 }} size="small">
            <InputLabel>Model</InputLabel>
            <Select
              value={selectedModel}
              label="Model"
              onChange={(event) => loadModel(event.target.value as string)}
            >
              {models.map((model) => (
                <MenuItem key={model.id} value={model.id}>
                  {model.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <Typography variant="body2" color="text.secondary">
            {modelStatus}
          </Typography>
          <Stack direction="row" alignItems="center" spacing={1.5}>
            <Switch defaultChecked size="small" />
            <Typography variant="body2" color="text.secondary">
              Auto-load on startup
            </Typography>
          </Stack>
        </Stack>
      </Card>

      {/* Classes */}
      <Card variant="outlined" sx={{ p: 3 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
            Classes
          </Typography>
          <Stack direction="row" spacing={1} flexWrap="wrap" useFlexGap>
            {classes.map((cls) => (
              <Box
                key={cls}
                sx={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: 1,
                  px: 1.5,
                  py: 0.75,
                  borderRadius: '6px',
                  border: `1px solid ${theme.palette.divider}`,
                  bgcolor: 'background.default',
                }}
              >
                <Box
                  sx={{
                    width: 8,
                    height: 8,
                    borderRadius: '50%',
                    bgcolor: classColors[cls] ?? theme.palette.primary.main,
                    flexShrink: 0,
                  }}
                />
                <Typography variant="body2" sx={{ fontWeight: 500, color: 'text.primary' }}>
                  {cls}
                </Typography>
              </Box>
            ))}
          </Stack>
          <Stack direction="row" spacing={1}>
            <Button size="small" variant="outlined" onClick={() => addClass('New class')}>
              Add class
            </Button>
            <Button
              size="small"
              variant="outlined"
              onClick={() => updateClassColor(classes[0] ?? 'Foreground', '#f38ba8')}
            >
              Update color
            </Button>
          </Stack>
        </Stack>
      </Card>

      {/* Display */}
      <Card variant="outlined" sx={{ p: 3, flex: 1 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
            Display
          </Typography>
          <FormControl sx={{ minWidth: 240 }} size="small">
            <InputLabel>Outline mode</InputLabel>
            <Select
              value={outlineMode}
              label="Outline mode"
              onChange={(event) => setOutlineMode(event.target.value as OutlineMode)}
            >
              <MenuItem value="Class color">Class color</MenuItem>
              <MenuItem value="Neutral">Neutral</MenuItem>
            </Select>
          </FormControl>
        </Stack>
      </Card>
    </Stack>
  );
};

export default SettingsTab;
