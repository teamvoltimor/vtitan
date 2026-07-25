import AutoFixHighIcon from '@mui/icons-material/AutoFixHigh';
import BrushIcon from '@mui/icons-material/Brush';
import GridViewIcon from '@mui/icons-material/GridView';
import ModelTrainingIcon from '@mui/icons-material/ModelTraining';
import SettingsSuggestIcon from '@mui/icons-material/SettingsSuggest';
import { Box, useMediaQuery, useTheme } from '@mui/material';
import { useEffect, useState } from 'react';
import { useAppState } from '../state/appState';
import AnnotateTab from './AnnotateTab';
import AugmentTab from './AugmentTab';
import BrowseTab from './BrowseTab';
import HeroHeader from './HeroHeader';
import SettingsTab from './SettingsTab';
import TrainTab from './TrainTab';
import { AppHeader } from './AppHeader';
import { MobileNav } from './MobileNav';
import { SidebarNav } from './SidebarNav';
import type { TabConfig } from './SidebarNav';

const tabRows: TabConfig[] = [
  {
    label: 'Annotate',
    Component: AnnotateTab,
    icon: <BrushIcon sx={{ fontSize: 14 }} />,
    shortcut: 'A',
  },
  {
    label: 'Browse',
    Component: BrowseTab,
    icon: <GridViewIcon sx={{ fontSize: 14 }} />,
    shortcut: 'B',
  },
  {
    label: 'Augment',
    Component: AugmentTab,
    icon: <AutoFixHighIcon sx={{ fontSize: 14 }} />,
    shortcut: 'U',
  },
  {
    label: 'Train',
    Component: TrainTab,
    icon: <ModelTrainingIcon sx={{ fontSize: 14 }} />,
    shortcut: 'T',
  },
  {
    label: 'Settings',
    Component: SettingsTab,
    icon: <SettingsSuggestIcon sx={{ fontSize: 14 }} />,
    shortcut: 'S',
  },
];

type AppShellProps = { onToggleTheme: () => void };

const AppShell = ({ onToggleTheme }: AppShellProps) => {
  const theme = useTheme();
  const isDesktop = useMediaQuery(theme.breakpoints.up('lg'));
  const [activeTabIndex, setActiveTabIndex] = useState(0);
  const mode = theme.palette.mode;
  const { modelStatus } = useAppState();

  useEffect(() => {
    const shortcuts: Record<string, number> = { a: 0, b: 1, u: 2, t: 3, s: 4 };
    const handleKeyDown = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || (e.target as HTMLElement).isContentEditable)
        return;
      const index = shortcuts[e.key.toLowerCase()];
      if (index !== undefined) setActiveTabIndex(index);
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  return (
    <Box
      sx={{
        minHeight: '100vh',
        bgcolor: 'background.default',
        display: 'flex',
        flexDirection: 'column',
        backgroundImage:
          mode === 'dark'
            ? [
                'radial-gradient(ellipse 70% 240px at 55% 0px, rgba(94,106,210,0.07), transparent)',
                'radial-gradient(circle, rgba(255,255,255,0.022) 1px, transparent 1px)',
              ].join(', ')
            : [
                'radial-gradient(ellipse 70% 240px at 55% 0px, rgba(79,92,200,0.04), transparent)',
                'radial-gradient(circle, rgba(0,0,0,0.038) 1px, transparent 1px)',
              ].join(', '),
        backgroundSize: 'auto, 20px 20px',
      }}
    >
      <AppHeader onToggleTheme={onToggleTheme} />

      <Box sx={{ flex: 1, display: 'flex', overflow: 'hidden' }}>
        {isDesktop && (
          <SidebarNav
            tabRows={tabRows}
            activeTabIndex={activeTabIndex}
            setActiveTabIndex={setActiveTabIndex}
            modelStatus={modelStatus}
          />
        )}

        <Box sx={{ flex: 1, overflow: 'auto', display: 'flex', flexDirection: 'column' }}>
          <Box
            sx={{
              px: { xs: 3, sm: 4, md: 5 },
              py: { xs: 3, sm: 4 },
              maxWidth: 1120,
              width: '100%',
            }}
          >
            {!isDesktop && (
              <MobileNav
                tabRows={tabRows}
                activeTabIndex={activeTabIndex}
                setActiveTabIndex={setActiveTabIndex}
              />
            )}

            <HeroHeader onLaunchAnnotate={() => setActiveTabIndex(0)} />

            <Box sx={{ mt: 4 }}>
              {tabRows.map((tab, index) =>
                activeTabIndex === index ? (
                  <tab.Component key={tab.label} onNavigate={(to) => setActiveTabIndex(to)} />
                ) : null
              )}
            </Box>
          </Box>
        </Box>
      </Box>
    </Box>
  );
};

export default AppShell;
