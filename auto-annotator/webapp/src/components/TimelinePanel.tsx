import { Divider, Stack, Typography, useTheme } from '@mui/material'
import AccessTimeIcon from '@mui/icons-material/AccessTime'
import Timeline from '@mui/lab/Timeline'
import TimelineItem from '@mui/lab/TimelineItem'
import TimelineSeparator from '@mui/lab/TimelineSeparator'
import TimelineConnector from '@mui/lab/TimelineConnector'
import TimelineContent from '@mui/lab/TimelineContent'
import TimelineDot from '@mui/lab/TimelineDot'

import type { TimelineEvent } from '../state/appState'

const TimelinePanel = ({ events }: { events: TimelineEvent[] }) => {
  const theme = useTheme()

  return (
    <Stack spacing={1} sx={{ p: 2, borderRadius: 3, border: `1px solid ${theme.palette.divider}` }}>
      <Stack direction="row" justifyContent="space-between" alignItems="center">
        <Typography variant="subtitle2" fontWeight={600}>
          Recent actions
        </Typography>
        <AccessTimeIcon fontSize="small" color="disabled" />
      </Stack>
      <Divider />
      <Timeline sx={{ p: 0 }}>
        {events.map((event, index) => (
          <TimelineItem key={event.time} sx={{ minHeight: 72 }}>
            <TimelineSeparator>
              <TimelineDot sx={{ bgcolor: theme.palette.primary.main }} />
              {index < events.length - 1 && <TimelineConnector sx={{ bgcolor: theme.palette.divider }} />}
            </TimelineSeparator>
            <TimelineContent>
              <Typography variant="body2" fontWeight={600}>
                {event.label}
              </Typography>
              <Typography variant="caption" color="text.secondary">
                {event.time}
              </Typography>
            </TimelineContent>
          </TimelineItem>
        ))}
      </Timeline>
    </Stack>
  )
}

export default TimelinePanel
