import { Card, FormControl, InputLabel, MenuItem, Select, Stack, Typography } from '@mui/material';
import type { OutlineMode } from '../state/appState';

export function DisplaySettings({
  outlineMode,
  onOutlineModeChange,
}: {
  outlineMode: OutlineMode;
  onOutlineModeChange: (mode: OutlineMode) => void;
}) {
  return (
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
            onChange={(event) => onOutlineModeChange(event.target.value as OutlineMode)}
          >
            <MenuItem value="Class color">Class color</MenuItem>
            <MenuItem value="Neutral">Neutral</MenuItem>
          </Select>
        </FormControl>
      </Stack>
    </Card>
  );
}
