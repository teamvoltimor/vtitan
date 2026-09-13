import AccessTimeIcon from '@mui/icons-material/AccessTime';
import Timeline from '@mui/lab/Timeline';
import TimelineConnector from '@mui/lab/TimelineConnector';
import TimelineContent from '@mui/lab/TimelineContent';
import TimelineDot from '@mui/lab/TimelineDot';
import TimelineItem from '@mui/lab/TimelineItem';
import TimelineSeparator from '@mui/lab/TimelineSeparator';
import { Box, Divider, Stack, Typography, useTheme } from '@mui/material';
import type { TimelineEvent } from '../state/appState';

const TimelinePanel = ({ events }: { events: TimelineEvent[] }) => {
  const theme = useTheme();

  return (
    <Stack
      spacing={1}
      sx={{
        p: 2,
        borderRadius: '6px',
        border: `1px solid ${theme.palette.divider}`,
        flex: 1,
      }}
    >
      <Stack direction="row" justifyContent="space-between" alignItems="center">
        <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
          Recent actions
        </Typography>
        <AccessTimeIcon fontSize="small" color="disabled" />
      </Stack>
      <Divider />

      {events.length === 0 ? (
        <Box sx={{ py: 3, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <Typography variant="caption" color="text.disabled">
            No actions recorded yet
          </Typography>
        </Box>
      ) : (
        <Timeline sx={{ p: 0, m: 0 }}>
          {events.map((event, index) => (
            <TimelineItem key={event.time} sx={{ minHeight: 56, '&::before': { display: 'none' } }}>
              <TimelineSeparator>
                <TimelineDot
                  sx={{ bgcolor: theme.palette.primary.main, boxShadow: 'none', m: 0 }}
                />
                {index < events.length - 1 && (
                  <TimelineConnector sx={{ bgcolor: theme.palette.divider }} />
                )}
              </TimelineSeparator>
              <TimelineContent sx={{ py: 0, px: 1.5, pb: index < events.length - 1 ? 1.5 : 0 }}>
                <Typography variant="body2" sx={{ fontWeight: 500 }}>
                  {event.label}
                </Typography>
                <Typography
                  variant="caption"
                  color="text.disabled"
                  sx={{ fontFamily: '"JetBrains Mono", monospace' }}
                >
                  {event.time}
                </Typography>
              </TimelineContent>
            </TimelineItem>
          ))}
        </Timeline>
      )}
    </Stack>
  );
};

export default TimelinePanel;
