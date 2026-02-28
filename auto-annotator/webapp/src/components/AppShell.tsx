import {
  Box,
  Button,
  Container,
  Divider,
  IconButton,
  Paper,
  Stack,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material'
import { useState } from 'react'
import AnnotateTab from './AnnotateTab'
import BrowseTab from './BrowseTab'
import HeroHeader from './HeroHeader'
import SettingsTab from './SettingsTab'
import type { TabContentProps } from './types'
import BrushIcon from '@mui/icons-material/Brush'
import GridViewIcon from '@mui/icons-material/GridView'
import SettingsSuggestIcon from '@mui/icons-material/SettingsSuggest'
import TerminalIcon from '@mui/icons-material/Terminal'
import BoltIcon from '@mui/icons-material/Bolt'

type TabConfig = {
  label: string
  Component: React.ComponentType<TabContentProps>
  icon: React.ReactNode
  helper: string
}

const tabRows: TabConfig[] = [
  { label: 'Annotate', Component: AnnotateTab, icon: <BrushIcon fontSize="small" />, helper: 'Label with precision' },
  { label: 'Browse', Component: BrowseTab, icon: <GridViewIcon fontSize="small" />, helper: 'Inspect and queue' },
  { label: 'Settings', Component: SettingsTab, icon: <SettingsSuggestIcon fontSize="small" />, helper: 'Calibrate ops' },
]

const AppShell = () => {
  const theme = useTheme()
  const isDesktop = useMediaQuery(theme.breakpoints.up('lg'))
  const [active, setActive] = useState(0)

  return (
    <Box
      sx={{
        minHeight: '100vh',
        bgcolor: '#04050c',
        backgroundImage: 'linear-gradient(150deg, rgba(255,255,255,0.03), transparent 60%)',
        px: { xs: 1, sm: 2, lg: 4 },
        py: { xs: 1, sm: 2, md: 4 },
      }}
    >
      <Container maxWidth="xl" sx={{ width: '100%', display: 'flex', flexDirection: 'column', gap: 2 }}>
        <HeroHeader onLaunchAnnotate={() => setActive(0)} />
        <Paper
          elevation={10}
          sx={{
            borderRadius: 0,
            border: `1px solid rgba(255,255,255,0.08)`,
            p: { xs: 2, md: 3 },
            bgcolor: 'rgba(8, 9, 18, 0.92)',
            boxShadow: '0 20px 60px rgba(0, 0, 0, 0.55)',
          }}
        >
          <Stack direction={{ xs: 'column', lg: 'row' }} spacing={3} alignItems="flex-start">
            {isDesktop && (
              <Paper
                elevation={6}
                sx={{
                  minWidth: 220,
                  borderRadius: 3,
                  border: `1px solid rgba(255,255,255,0.06)`,
                  bgcolor: 'rgba(11, 11, 32, 0.9)',
                }}
              >
                <Stack spacing={1} sx={{ p: 2 }}>
                  <Typography variant="caption" color="text.secondary">
                    Command palette
                  </Typography>
                  {tabRows.map((tab, index) => (
                    <Button
                      key={tab.label}
                      startIcon={tab.icon}
                      variant={active === index ? 'contained' : 'outlined'}
                      color={active === index ? 'secondary' : 'inherit'}
                      onClick={() => setActive(index)}
                      sx={{
                        justifyContent: 'flex-start',
                        borderRadius: 2,
                        textTransform: 'none',
                        fontWeight: 600,
                        py: 1.25,
                      }}
                    >
                      <Stack direction="column" alignItems="flex-start" spacing={0.15}>
                        <Typography variant="body2">{tab.label}</Typography>
                        <Typography variant="caption" color="text.secondary">
                          {tab.helper}
                        </Typography>
                      </Stack>
                    </Button>
                  ))}
                </Stack>
                <Divider sx={{ my: 1 }} />
                <Stack direction="row" justifyContent="space-between" sx={{ p: 2 }}>
                  <IconButton size="small" sx={{ border: '1px solid rgba(255,255,255,0.1)' }}>
                    <TerminalIcon fontSize="small" />
                  </IconButton>
                  <IconButton size="small" sx={{ border: '1px solid rgba(255,255,255,0.1)' }}>
                    <BoltIcon fontSize="small" />
                  </IconButton>
                </Stack>
              </Paper>
            )}
            <Box sx={{ flex: 1 }}>
              {!isDesktop && (
                <Stack direction="row" spacing={1} justifyContent="center">
                  {tabRows.map((tab, index) => (
                    <Button
                      key={tab.label}
                      size="small"
                      variant={active === index ? 'contained' : 'outlined'}
                      color={active === index ? 'secondary' : 'inherit'}
                      onClick={() => setActive(index)}
                      sx={{ borderRadius: 3, textTransform: 'none', minWidth: 100 }}
                    >
                      {tab.icon}
                      <Typography variant="body2" sx={{ ml: 0.5 }}>
                        {tab.label}
                      </Typography>
                    </Button>
                  ))}
                </Stack>
              )}
              <Box sx={{ mt: isDesktop ? 0 : 2 }}>
                {tabRows.map((tab, index) => (
                  <Box key={tab.label} hidden={active !== index} sx={{ height: '100%' }}>
                    {active === index && <tab.Component onNavigate={(to) => setActive(to)} />}
                  </Box>
                ))}
              </Box>
            </Box>
          </Stack>
        </Paper>
      </Container>
    </Box>
  )
}

export default AppShell
