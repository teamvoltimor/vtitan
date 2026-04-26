import AutoFixHighIcon from '@mui/icons-material/AutoFixHigh';
import BrushIcon from '@mui/icons-material/Brush';
import DarkModeIcon from '@mui/icons-material/DarkMode';
import GridViewIcon from '@mui/icons-material/GridView';
import LightModeIcon from '@mui/icons-material/LightMode';
import ModelTrainingIcon from '@mui/icons-material/ModelTraining';
import SettingsSuggestIcon from '@mui/icons-material/SettingsSuggest';
import {
  Box,
  Button,
  IconButton,
  Stack,
  Tooltip,
  Typography,
  useMediaQuery,
  useTheme,
} from '@mui/material';
import { useEffect, useState } from 'react';
import { useAppState } from '../state/appState';
import AnnotateTab from './AnnotateTab';
import AugmentTab from './AugmentTab';
import BrowseTab from './BrowseTab';
import HeroHeader from './HeroHeader';
import SettingsTab from './SettingsTab';
import TrainTab from './TrainTab';
import type { TabContentProps } from './types';

type TabConfig = {
  label: string;
  Component: React.ComponentType<TabContentProps>;
  icon: React.ReactNode;
  shortcut: string;
};

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
      if (tag === 'INPUT' || tag === 'TEXTAREA' || (e.target as HTMLElement).isContentEditable) return;
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
      {/* Top bar — frosted glass */}
      <Box
        component="header"
        sx={{
          height: 44,
          borderBottom: `1px solid ${theme.palette.divider}`,
          display: 'flex',
          alignItems: 'center',
          px: 2.5,
          position: 'sticky',
          top: 0,
          zIndex: 100,
          bgcolor: mode === 'dark' ? 'rgba(13,13,16,0.82)' : 'rgba(255,255,255,0.82)',
          backdropFilter: 'blur(14px)',
          WebkitBackdropFilter: 'blur(14px)',
        }}
      >
        <Stack direction="row" alignItems="center" spacing={1} sx={{ flex: 1 }}>
          <Box
            sx={{
              width: 18,
              height: 18,
              borderRadius: '4px',
              background: `linear-gradient(145deg, ${theme.palette.primary.main}, ${mode === 'dark' ? '#3d4ab0' : '#2a35a0'})`,
              flexShrink: 0,
            }}
          />
          <Typography
            variant="body2"
            sx={{
              fontWeight: 600,
              color: 'text.primary',
              fontFamily: '"JetBrains Mono", monospace',
              fontSize: '0.75rem',
              letterSpacing: '-0.01em',
            }}
          >
            klevor
            <Box
              component="span"
              sx={{
                color: 'text.disabled',
                fontFamily: 'inherit',
                fontSize: 'inherit',
                fontWeight: 400,
              }}
            >
              {' '}
              / annotate
            </Box>
          </Typography>
        </Stack>

        <Tooltip title={mode === 'dark' ? 'Light mode' : 'Dark mode'} placement="bottom-end">
          <IconButton size="small" onClick={onToggleTheme}>
            {mode === 'dark' ? (
              <LightModeIcon sx={{ fontSize: 15 }} />
            ) : (
              <DarkModeIcon sx={{ fontSize: 15 }} />
            )}
          </IconButton>
        </Tooltip>
      </Box>

      {/* Body */}
      <Box sx={{ flex: 1, display: 'flex', overflow: 'hidden' }}>
        {/* Sidebar */}
        {isDesktop && (
          <Box
            component="nav"
            sx={{
              width: 192,
              borderRight: `1px solid ${theme.palette.divider}`,
              flexShrink: 0,
              display: 'flex',
              flexDirection: 'column',
              py: 2,
              bgcolor: mode === 'dark' ? 'rgba(13,13,16,0.6)' : 'rgba(255,255,255,0.7)',
              backdropFilter: 'blur(8px)',
              WebkitBackdropFilter: 'blur(8px)',
            }}
          >
            <Typography
              variant="caption"
              sx={{
                color: 'text.disabled',
                fontWeight: 600,
                letterSpacing: '0.07em',
                textTransform: 'uppercase',
                px: 2.5,
                mb: 0.75,
                display: 'block',
                fontSize: '0.5625rem',
              }}
            >
              Workspace
            </Typography>

            <Stack spacing={0.25} sx={{ px: 1.5 }}>
              {tabRows.map((tab, index) => {
                const isActive = activeTabIndex === index;
                return (
                  <Box
                    key={tab.label}
                    onClick={() => setActiveTabIndex(index)}
                    role="button"
                    tabIndex={0}
                    onKeyDown={(e) =>
                      (e.key === 'Enter' || e.key === ' ') && setActiveTabIndex(index)
                    }
                    sx={{
                      position: 'relative',
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'space-between',
                      px: 1.5,
                      py: 0.875,
                      borderRadius: '6px',
                      cursor: 'pointer',
                      bgcolor: isActive
                        ? mode === 'dark'
                          ? 'rgba(255,255,255,0.07)'
                          : 'rgba(0,0,0,0.05)'
                        : 'transparent',
                      color: isActive ? 'text.primary' : 'text.secondary',
                      userSelect: 'none',
                      outline: 'none',
                      transition: 'background-color 80ms ease, color 80ms ease',
                      '&:hover': {
                        bgcolor: mode === 'dark' ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.035)',
                        color: 'text.primary',
                      },
                      '&:focus-visible': {
                        boxShadow: `0 0 0 2px ${theme.palette.primary.main}`,
                      },
                      // Left accent indicator
                      '&::before': {
                        content: '""',
                        position: 'absolute',
                        left: 0,
                        top: '50%',
                        transform: 'translateY(-50%)',
                        width: '2px',
                        height: '52%',
                        backgroundColor: theme.palette.primary.main,
                        borderRadius: '0 2px 2px 0',
                        opacity: isActive ? 1 : 0,
                        transition: 'opacity 150ms ease',
                      },
                    }}
                  >
                    <Stack direction="row" spacing={1.25} alignItems="center">
                      <Box
                        sx={{
                          color: 'inherit',
                          display: 'flex',
                          alignItems: 'center',
                          opacity: isActive ? 1 : 0.65,
                        }}
                      >
                        {tab.icon}
                      </Box>
                      <Typography
                        variant="body2"
                        sx={{
                          fontWeight: isActive ? 500 : 400,
                          color: 'inherit',
                          lineHeight: 1,
                        }}
                      >
                        {tab.label}
                      </Typography>
                    </Stack>

                    <Box
                      component="kbd"
                      sx={{
                        fontSize: '0.5625rem',
                        fontFamily: '"JetBrains Mono", monospace',
                        color: 'text.disabled',
                        bgcolor: mode === 'dark' ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.05)',
                        border: `1px solid ${theme.palette.divider}`,
                        borderRadius: '3px',
                        px: 0.75,
                        py: 0.25,
                        lineHeight: 1.6,
                        minWidth: 18,
                        textAlign: 'center',
                      }}
                    >
                      {tab.shortcut}
                    </Box>
                  </Box>
                );
              })}
            </Stack>

            {/* Sidebar footer */}
            <Box sx={{ mt: 'auto', px: 1.5, pt: 2 }}>
              <Stack
                direction="row"
                alignItems="center"
                justifyContent="space-between"
                sx={{ px: 1 }}
              >
                <Typography
                  variant="caption"
                  sx={{
                    color: 'text.disabled',
                    fontFamily: '"JetBrains Mono", monospace',
                    fontSize: '0.5625rem',
                  }}
                >
                  auto-annotator
                </Typography>
                <Typography variant="caption" color="text.secondary" sx={{ fontSize: '0.65rem' }} noWrap>
                  {modelStatus}
                </Typography>
              </Stack>
            </Box>
          </Box>
        )}

        {/* Main content */}
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
              <Stack direction="row" spacing={0.5} sx={{ mb: 3 }}>
                {tabRows.map((tab, index) => (
                  <Button
                    key={tab.label}
                    size="small"
                    startIcon={tab.icon}
                    onClick={() => setActiveTabIndex(index)}
                    sx={{
                      color: activeTabIndex === index ? 'text.primary' : 'text.secondary',
                      bgcolor:
                        activeTabIndex === index
                          ? mode === 'dark'
                            ? 'rgba(255,255,255,0.07)'
                            : 'rgba(0,0,0,0.05)'
                          : 'transparent',
                      border: `1px solid ${activeTabIndex === index ? theme.palette.divider : 'transparent'}`,
                      fontWeight: activeTabIndex === index ? 500 : 400,
                      '&:hover': {
                        bgcolor: mode === 'dark' ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.03)',
                        color: 'text.primary',
                      },
                    }}
                  >
                    {tab.label}
                  </Button>
                ))}
              </Stack>
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
