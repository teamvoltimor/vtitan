import { createTheme } from '@mui/material'

const catppuccinPalette = {
  background: '#0e0f18',
  surface: '#151526',
  surfaceAlt: '#1a1b2f',
  border: '#2c2d49',
  primary: '#8b8dff',
  secondary: '#c1adff',
  error: '#f38ba8',
  warning: '#fab387',
  info: '#94e2d5',
  success: '#a6e3a1',
}

export const appTheme = createTheme({
  palette: {
    mode: 'dark',
    background: {
      default: catppuccinPalette.background,
      paper: catppuccinPalette.surface,
    },
    primary: {
      main: catppuccinPalette.primary,
    },
    secondary: {
      main: catppuccinPalette.secondary,
    },
    text: {
      primary: '#e1e3f5',
      secondary: '#a6a8c9',
    },
    divider: catppuccinPalette.border,
  },
  typography: {
    fontFamily: ['Space Grotesk', 'Inter', 'sans-serif'].join(', '),
    button: {
      textTransform: 'none',
      fontWeight: 600,
    },
  },
  shape: {
    borderRadius: 14,
  },
  components: {
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundColor: catppuccinPalette.surface,
          border: `1px solid ${catppuccinPalette.border}`,
          boxShadow: '0 12px 35px rgba(0, 0, 0, 0.45)',
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          fontWeight: 600,
        },
        containedPrimary: {
          boxShadow: '0 10px 20px rgba(137, 180, 250, 0.35)',
        },
      },
    },
    MuiTabs: {
      styleOverrides: {
        indicator: {
          height: 3,
          borderRadius: 8,
          backgroundImage:
            'linear-gradient(90deg, #8b8dff 0%, #bab8ff 55%, #f5b8ff 100%)',
        },
      },
    },
  },
})
