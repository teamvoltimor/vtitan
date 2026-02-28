import {
  Button,
  Card,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  Stack,
  Switch,
  Typography,
} from '@mui/material'
import { useAppState } from '../state/appState'
import type { OutlineMode } from '../state/appState'

const SettingsTab = () => {
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
  } = useAppState()

  return (
    <Stack spacing={3} sx={{ height: '100%' }}>
      <Card variant="outlined" sx={{ p: 3, borderRadius: 3 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" fontWeight={600}>
            Models
          </Typography>
          <FormControl sx={{ minWidth: 240 }}>
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
          <Switch defaultChecked />
        </Stack>
      </Card>
      <Card variant="outlined" sx={{ p: 3, borderRadius: 3 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" fontWeight={600}>
            Classes
          </Typography>
          <Stack direction="row" spacing={1} flexWrap="wrap">
            {classes.map((cls) => (
              <Button key={cls} size="small" sx={{ backgroundColor: classColors[cls], color: '#000' }}>
                {cls}
              </Button>
            ))}
          </Stack>
          <Stack direction="row" spacing={1}>
            <Button size="small" onClick={() => addClass('New class')}>
              Add class
            </Button>
            <Button size="small" onClick={() => updateClassColor(classes[0] ?? 'Foreground', '#f38ba8')}>
              Update color
            </Button>
          </Stack>
        </Stack>
      </Card>
      <Card variant="outlined" sx={{ p: 3, borderRadius: 3, flex: 1 }}>
        <Stack spacing={2}>
          <Typography variant="subtitle2" fontWeight={600}>
            Display
          </Typography>
          <FormControl sx={{ minWidth: 240 }}>
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
  )
}

export default SettingsTab
