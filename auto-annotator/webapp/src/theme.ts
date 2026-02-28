import { createTheme } from '@mui/material'

const paletteTokens = {
  background: '#03040a',
  surface: '#070b13',
  surfaceAlt: '#0b0f19',
  border: '#12172b',
  primary: '#f0f1ff',
  secondary: '#d1d6e8',
  slate: '#b8bdd3',
  accent: '#9aa0c8',
}

export const appTheme = createTheme({
    palette: {
      mode: 'dark',
      background: {
        default: paletteTokens.background,
        paper: paletteTokens.surface,
      },
      primary: {
        main: paletteTokens.primary,
        contrastText: '#05060f',
      },
      secondary: {
        main: paletteTokens.secondary,
      },
      info: {
        main: '#a9afc2',
      },
      divider: paletteTokens.border,
      text: {
        primary: paletteTokens.slate,
        secondary: '#9ba1c1',
      },
    },
  typography: {
    fontFamily: ['JetBrainsMono Nerd Font', 'Fira Code', 'JetBrains Mono', 'Menlo', 'monospace'].join(', '),
    h1: { fontFamily: ['JetBrainsMono Nerd Font', 'Fira Code', 'JetBrains Mono', 'Menlo', 'monospace'].join(', ') },
    h2: { fontFamily: ['JetBrainsMono Nerd Font', 'Fira Code', 'JetBrains Mono', 'Menlo', 'monospace'].join(', ') },
    h3: { fontFamily: ['JetBrainsMono Nerd Font', 'Fira Code', 'JetBrains Mono', 'Menlo', 'monospace'].join(', ') },
    button: {
      textTransform: 'none',
      fontWeight: 600,
    },
  },
  shape: {
    borderRadius: 0,
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: {
          backgroundColor: paletteTokens.background,
          color: '#f6f7ff',
          minHeight: '100vh',
          fontFamily: ['Sora', 'Neue Haas Grotesk Display', 'sans-serif'].join(', '),
        },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          background: paletteTokens.surface,
          border: `1px solid ${paletteTokens.border}`,
          boxShadow: '0 8px 16px rgba(0, 0, 0, 0.35)',
          borderRadius: 0,
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: 0,
          boxShadow: '0 6px 12px rgba(0, 0, 0, 0.35)',
        },
        containedPrimary: {
          backgroundColor: paletteTokens.primary,
          color: '#05060f',
        },
        outlined: {
          borderColor: 'rgba(255,255,255,0.35)',
        },
      },
    },
    MuiTab: {
      styleOverrides: {
        root: {
          color: '#c4c7e2',
          '&.Mui-selected': {
            color: '#fff',
          },
        },
      },
    },
    MuiTabs: {
      styleOverrides: {
        indicator: {
          height: 4,
          borderRadius: 2,
          backgroundImage:
            'linear-gradient(120deg, rgba(101,227,255,1), rgba(249,138,255,1))',
          boxShadow: '0 0 20px rgba(249,138,255,0.35), 0 0 30px rgba(101,227,255,0.25)',
        },
      },
    },
    MuiChip: {
      styleOverrides: {
        root: {
          backgroundColor: 'rgba(255,255,255,0.05)',
          border: '1px solid rgba(255,255,255,0.08)',
        },
      },
    },
  },
})
