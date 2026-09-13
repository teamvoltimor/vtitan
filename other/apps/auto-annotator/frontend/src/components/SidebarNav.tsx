import { Box, Stack, Typography, useTheme } from '@mui/material';
import type { ReactNode } from 'react';
import type { TabContentProps } from './types';

export interface TabConfig {
  label: string;
  Component: React.ComponentType<TabContentProps>;
  icon: ReactNode;
  shortcut: string;
}

export function SidebarNav({
  tabRows,
  activeTabIndex,
  setActiveTabIndex,
  modelStatus,
}: {
  tabRows: TabConfig[];
  activeTabIndex: number;
  setActiveTabIndex: (index: number) => void;
  modelStatus: string;
}) {
  const theme = useTheme();
  const mode = theme.palette.mode;

  return (
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
              component="button"
              type="button"
              onClick={() => setActiveTabIndex(index)}
              sx={{
                position: 'relative',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
                width: '100%',
                px: 1.5,
                py: 0.875,
                borderRadius: '6px',
                border: 'none',
                font: 'inherit',
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

      <Box sx={{ mt: 'auto', px: 1.5, pt: 2 }}>
        <Stack direction="row" alignItems="center" justifyContent="space-between" sx={{ px: 1 }}>
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
  );
}
