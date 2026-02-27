import { createTheme } from '@mui/material'

const catppuccinPalette = {
  background: '#1e1e2e',
  surface: '#1f1d2e',
  primary: '#89b4fa',
  secondary: '#cba6f7',
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
  },
  typography: {
    fontFamily: ['Inter', 'sans-serif'].join(', '),
  },
  components: {
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundColor: catppuccinPalette.surface,
        },
      },
    },
  },
})
