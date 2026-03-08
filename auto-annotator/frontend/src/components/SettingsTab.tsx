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
  TextField,
  Typography,
  useTheme,
} from '@mui/material';
import { useState } from 'react';
import type { OutlineMode } from '../state/appState';
import { useAppState } from '../state/appState';

const SettingsTab = () => {
  const theme = useTheme();
  const [newClassName, setNewClassName] = useState('');
  const [newClassColor, setNewClassColor] = useState('#fe9664');
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
            <Switch defaultChecked />
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

          {/* Existing classes */}
          <Stack spacing={1}>
            {classes.map((cls) => (
              <Stack key={cls} direction="row" alignItems="center" spacing={1.5}>
                <Box
                  component="input"
                  type="color"
                  value={classColors[cls] ?? '#ffffff'}
                  onChange={(e: React.ChangeEvent<HTMLInputElement>) =>
                    void updateClassColor(cls, e.target.value)
                  }
                  sx={{ width: 28, height: 28, border: 'none', p: 0, cursor: 'pointer', borderRadius: '4px', bgcolor: 'transparent' }}
                />
                <Typography variant="body2" sx={{ fontWeight: 500, flex: 1 }}>
                  {cls}
                </Typography>
              </Stack>
            ))}
          </Stack>

          {/* Add new class */}
          <Stack direction="row" spacing={1} alignItems="center">
            <Box
              component="input"
              type="color"
              value={newClassColor}
              onChange={(e: React.ChangeEvent<HTMLInputElement>) => setNewClassColor(e.target.value)}
              sx={{ width: 28, height: 28, border: 'none', p: 0, cursor: 'pointer', borderRadius: '4px', bgcolor: 'transparent' }}
            />
            <TextField
              size="small"
              placeholder="Class name"
              value={newClassName}
              onChange={(e) => setNewClassName(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && newClassName.trim()) {
                  void addClass(newClassName.trim(), newClassColor);
                  setNewClassName('');
                }
              }}
              sx={{ flex: 1 }}
            />
            <Button
              size="small"
              variant="outlined"
              disabled={!newClassName.trim()}
              onClick={() => {
                void addClass(newClassName.trim(), newClassColor);
                setNewClassName('');
              }}
            >
              Add
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
