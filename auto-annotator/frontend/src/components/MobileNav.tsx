import { Button, Stack, useTheme } from '@mui/material';
import type { TabConfig } from './SidebarNav';

export function MobileNav({
  tabRows,
  activeTabIndex,
  setActiveTabIndex,
}: {
  tabRows: TabConfig[];
  activeTabIndex: number;
  setActiveTabIndex: (index: number) => void;
}) {
  const theme = useTheme();
  const mode = theme.palette.mode;

  return (
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
  );
}
