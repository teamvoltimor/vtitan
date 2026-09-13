import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';
import { Box, Card, Divider, LinearProgress, Stack, Typography } from '@mui/material';

export function TrainingProgress({
  running,
  finished,
  epoch,
  total,
  boxLoss,
  clsLoss,
  map50,
  logs,
}: {
  running: boolean;
  finished: boolean;
  epoch: number;
  total: number;
  boxLoss: number;
  clsLoss: number;
  map50: number;
  logs: string[];
}) {
  if (!running && !finished) return null;

  const progressPct = total > 0 ? Math.round((epoch / total) * 100) : 0;

  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack spacing={1.5}>
        <Stack direction="row" justifyContent="space-between" alignItems="center">
          <Typography variant="subtitle2" fontWeight={600}>
            {finished ? 'Training complete' : 'Training'}
          </Typography>
          {finished && <CheckCircleOutlineIcon sx={{ color: 'success.main', fontSize: 18 }} />}
          <Typography variant="caption" color="text.disabled">
            Epoch {epoch} / {total}
          </Typography>
        </Stack>
        <LinearProgress
          variant="determinate"
          value={progressPct}
          color={finished ? 'success' : 'primary'}
        />
        <Stack direction="row" spacing={3}>
          <Stack spacing={0.25}>
            <Typography variant="caption" color="text.disabled">
              box loss
            </Typography>
            <Typography variant="body2" fontWeight={600}>
              {boxLoss.toFixed(4)}
            </Typography>
          </Stack>
          <Stack spacing={0.25}>
            <Typography variant="caption" color="text.disabled">
              cls loss
            </Typography>
            <Typography variant="body2" fontWeight={600}>
              {clsLoss.toFixed(4)}
            </Typography>
          </Stack>
          <Stack spacing={0.25}>
            <Typography variant="caption" color="text.disabled">
              mAP50
            </Typography>
            <Typography variant="body2" fontWeight={600}>
              {map50.toFixed(4)}
            </Typography>
          </Stack>
        </Stack>
        {logs.length > 0 && (
          <>
            <Divider />
            <Box
              sx={{
                fontFamily: '"JetBrains Mono", monospace',
                fontSize: '0.65rem',
                color: 'text.disabled',
                maxHeight: 120,
                overflowY: 'auto',
              }}
            >
              {logs.map((l, i) => (
                // biome-ignore lint/suspicious/noArrayIndexKey: training log lines are plain strings with no stable id and may repeat
                <div key={i}>{l}</div>
              ))}
            </Box>
          </>
        )}
      </Stack>
    </Card>
  );
}
