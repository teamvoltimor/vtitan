import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';
import { Card, LinearProgress, Stack, Typography } from '@mui/material';

export function AugmentProgress({
  running,
  finished,
  done,
  total,
  step,
}: {
  running: boolean;
  finished: boolean;
  done: number;
  total: number;
  step: string;
}) {
  if (!running && !finished) return null;

  const progressPct = total > 0 ? Math.round((done / total) * 100) : 0;

  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack spacing={1}>
        <Stack direction="row" justifyContent="space-between" alignItems="center">
          <Typography variant="subtitle2" fontWeight={600}>
            {finished ? 'Done' : 'Progress'}
          </Typography>
          {finished && <CheckCircleOutlineIcon sx={{ color: 'success.main', fontSize: 18 }} />}
          <Typography variant="caption" color="text.disabled">
            {done} / {total}
          </Typography>
        </Stack>
        <LinearProgress
          variant="determinate"
          value={progressPct}
          color={finished ? 'success' : 'primary'}
        />
        {step && (
          <Typography variant="caption" color="text.disabled" noWrap>
            {step}
          </Typography>
        )}
      </Stack>
    </Card>
  );
}
