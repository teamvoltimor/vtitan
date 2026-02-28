import BrushIcon from '@mui/icons-material/Brush';
import DarkModeIcon from '@mui/icons-material/DarkMode';
import GridViewIcon from '@mui/icons-material/GridView';
import LightModeIcon from '@mui/icons-material/LightMode';
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
import { useState } from 'react';
import AnnotateTab from './AnnotateTab';
import BrowseTab from './BrowseTab';
import HeroHeader from './HeroHeader';
import SettingsTab from './SettingsTab';
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
  const [active, setActive] = useState(0);
  const mode = theme.palette.mode;

  return (
    <Box
      sx={{
        minHeight: '100vh',
        bgcolor: 'background.default',
        display: 'flex',
        flexDirection: 'column',
        backgroundImage:
          mode === 'dark'
            ? 'radial-gradient(circle, rgba(255,255,255,0.022) 1px, transparent 1px)'
            : 'radial-gradient(circle, rgba(0,0,0,0.038) 1px, transparent 1px)',
        backgroundSize: '20px 20px',
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
                const isActive = active === index;
                return (
                  <Box
                    key={tab.label}
                    onClick={() => setActive(index)}
                    role="button"
                    tabIndex={0}
                    onKeyDown={(e) => e.key === 'Enter' && setActive(index)}
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
                    onClick={() => setActive(index)}
                    sx={{
                      color: active === index ? 'text.primary' : 'text.secondary',
                      bgcolor:
                        active === index
                          ? mode === 'dark'
                            ? 'rgba(255,255,255,0.07)'
                            : 'rgba(0,0,0,0.05)'
                          : 'transparent',
                      border: `1px solid ${active === index ? theme.palette.divider : 'transparent'}`,
                      fontWeight: active === index ? 500 : 400,
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

            <HeroHeader onLaunchAnnotate={() => setActive(0)} />

            <Box sx={{ mt: 4 }}>
              {tabRows.map((tab, index) => (
                <Box key={tab.label} hidden={active !== index}>
                  {active === index && <tab.Component onNavigate={(to) => setActive(to)} />}
                </Box>
              ))}
            </Box>
          </Box>
        </Box>
      </Box>
    </Box>
  );
};

export default AppShell;
