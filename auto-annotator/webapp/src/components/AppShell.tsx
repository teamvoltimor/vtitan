import { Box, Tab, Tabs, useMediaQuery, useTheme } from '@mui/material'
import { useState } from 'react'
import AnnotateTab from './AnnotateTab'
import BrowseTab from './BrowseTab'
import HeroHeader from './HeroHeader'
import SettingsTab from './SettingsTab'
import type { TabContentProps } from './types'

type TabConfig = {
  label: string
  Component: React.ComponentType<TabContentProps>
}

const tabRows: TabConfig[] = [
  { label: 'Annotate', Component: AnnotateTab },
  { label: 'Browse', Component: BrowseTab },
  { label: 'Settings', Component: SettingsTab },
]

const AppShell = () => {
  const theme = useTheme()
  const isDesktop = useMediaQuery(theme.breakpoints.up('sm'))
  const [active, setActive] = useState(0)

  return (
    <Box
      sx={{
        minHeight: '100vh',
        bgcolor: 'background.default',
        px: { xs: 2, sm: 3 },
        py: { xs: 3, md: 4 },
        display: 'flex',
        flexDirection: 'column',
        gap: 3,
      }}
    >
      <HeroHeader onLaunchAnnotate={() => setActive(0)} />
      <Tabs
        value={active}
        onChange={(_, value) => setActive(value)}
        variant={isDesktop ? 'standard' : 'fullWidth'}
        centered={false}
        sx={{
          borderBottom: 1,
          borderColor: 'divider',
          '& .MuiTabs-indicator': { height: 3, borderRadius: 999 },
        }}
      >
        {tabRows.map((tab) => (
          <Tab
            key={tab.label}
            label={tab.label}
            sx={{
              textTransform: 'none',
              fontWeight: 600,
              fontSize: '0.95rem',
            }}
          />
        ))}
      </Tabs>
      <Box sx={{ flex: 1 }}>
        {tabRows.map((tab, index) => (
          <Box key={tab.label} hidden={active !== index} sx={{ height: '100%' }}>
            {active === index && <tab.Component onNavigate={(to) => setActive(to)} />}
          </Box>
        ))}
      </Box>
    </Box>
  )
}

export default AppShell
