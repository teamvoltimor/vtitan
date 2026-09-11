import { Box, Button, Card, Stack, TextField, Typography } from '@mui/material';
import { useState } from 'react';

export function ClassSettings({
  classes,
  classColors,
  onUpdateColor,
  onAddClass,
}: {
  classes: string[];
  classColors: Record<string, string>;
  onUpdateColor: (cls: string, color: string) => void;
  onAddClass: (name: string, color: string) => void;
}) {
  const [newClassName, setNewClassName] = useState('');
  const [newClassColor, setNewClassColor] = useState('#fe9664');

  return (
    <Card variant="outlined" sx={{ p: 3 }}>
      <Stack spacing={2}>
        <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
          Classes
        </Typography>

        <Stack spacing={1}>
          {classes.map((cls) => (
            <Stack key={cls} direction="row" alignItems="center" spacing={1.5}>
              <Box
                component="input"
                type="color"
                value={classColors[cls] ?? '#ffffff'}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) =>
                  onUpdateColor(cls, e.target.value)
                }
                sx={{
                  width: 28,
                  height: 28,
                  border: 'none',
                  p: 0,
                  cursor: 'pointer',
                  borderRadius: '4px',
                  bgcolor: 'transparent',
                }}
              />
              <Typography variant="body2" sx={{ fontWeight: 500, flex: 1 }}>
                {cls}
              </Typography>
            </Stack>
          ))}
        </Stack>

        <Stack direction="row" spacing={1} alignItems="center">
          <Box
            component="input"
            type="color"
            value={newClassColor}
            onChange={(e: React.ChangeEvent<HTMLInputElement>) => setNewClassColor(e.target.value)}
            sx={{
              width: 28,
              height: 28,
              border: 'none',
              p: 0,
              cursor: 'pointer',
              borderRadius: '4px',
              bgcolor: 'transparent',
            }}
          />
          <TextField
            size="small"
            placeholder="Class name"
            value={newClassName}
            onChange={(e) => setNewClassName(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter' && newClassName.trim()) {
                onAddClass(newClassName.trim(), newClassColor);
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
              onAddClass(newClassName.trim(), newClassColor);
              setNewClassName('');
            }}
          >
            Add
          </Button>
        </Stack>
      </Stack>
    </Card>
  );
}
