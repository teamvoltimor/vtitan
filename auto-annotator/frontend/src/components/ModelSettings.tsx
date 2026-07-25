import { Card, FormControl, InputLabel, MenuItem, Select, Stack, Switch, Typography } from '@mui/material';

export function ModelSettings({
  models,
  selectedModel,
  modelStatus,
  onModelChange,
}: {
  models: { id: string; label: string }[];
  selectedModel: string;
  modelStatus: string;
  onModelChange: (id: string) => void;
}) {
  return (
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
            onChange={(event) => onModelChange(event.target.value as string)}
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
  );
}
