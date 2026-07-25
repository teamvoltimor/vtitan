import DarkModeIcon from '@mui/icons-material/DarkMode';
import LightModeIcon from '@mui/icons-material/LightMode';
import { Box, IconButton, Stack, Tooltip, Typography, useTheme } from '@mui/material';

export function AppHeader({ onToggleTheme }: { onToggleTheme: () => void }) {
  const theme = useTheme();
  const mode = theme.palette.mode;

  return (
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
  );
}
