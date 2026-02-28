import { createTheme, type PaletteMode } from '@mui/material';

const dark = {
  bg: '#0d0d10',
  bgEl: '#131316',
  bgHover: '#1a1a1e',
  border: '#22222c',
  borderStr: '#2e2e3a',
  textHi: '#e8e8ec',
  textMid: '#8a8a96',
  textLo: '#50505c',
  accent: '#5e6ad2',
  accentHover: '#6d79da',
  accentFade: 'rgba(94,106,210,0.12)',
};

const light = {
  bg: '#ffffff',
  bgEl: '#f6f6f8',
  bgHover: '#eeeff1',
  border: '#e5e5e9',
  borderStr: '#cfcfd8',
  textHi: '#0d0d10',
  textMid: '#4a4a58',
  textLo: '#6a6a78',
  accent: '#4f5cc8',
  accentHover: '#3f4cb8',
  accentFade: 'rgba(79,92,200,0.08)',
};

export const makeTheme = (mode: PaletteMode) => {
  const t = mode === 'dark' ? dark : light;

  return createTheme({
    palette: {
      mode,
      background: { default: t.bg, paper: t.bgEl },
      primary: { main: t.accent, contrastText: '#ffffff' },
      secondary: {
        main: t.textHi,
        contrastText: mode === 'dark' ? t.bg : '#ffffff',
      },
      text: { primary: t.textHi, secondary: t.textMid, disabled: t.textLo },
      divider: t.border,
      action: {
        hover: mode === 'dark' ? 'rgba(255,255,255,0.04)' : 'rgba(0,0,0,0.035)',
        selected: mode === 'dark' ? 'rgba(255,255,255,0.07)' : 'rgba(0,0,0,0.05)',
        disabled: mode === 'dark' ? 'rgba(255,255,255,0.3)' : 'rgba(0,0,0,0.3)',
        disabledBackground: mode === 'dark' ? 'rgba(255,255,255,0.06)' : 'rgba(0,0,0,0.06)',
      },
      success: { main: '#36b37e' },
      warning: { main: '#e6a817' },
      error: { main: mode === 'dark' ? '#f87171' : '#dc2626' },
    },
    typography: {
      fontFamily: '"Instrument Sans", "DM Sans", system-ui, -apple-system, sans-serif',
      h1: { fontWeight: 700, letterSpacing: '-0.03em', lineHeight: 1.1 },
      h2: { fontWeight: 700, letterSpacing: '-0.025em', lineHeight: 1.15 },
      h3: { fontWeight: 600, letterSpacing: '-0.02em', lineHeight: 1.2 },
      h4: { fontWeight: 600, letterSpacing: '-0.015em', lineHeight: 1.25 },
      h5: { fontWeight: 700, letterSpacing: '-0.02em' },
      h6: { fontWeight: 600, letterSpacing: '-0.005em' },
      body1: { letterSpacing: '-0.007em', lineHeight: 1.6 },
      body2: { fontSize: '0.8125rem', letterSpacing: '-0.005em', lineHeight: 1.55 },
      caption: { fontSize: '0.6875rem', letterSpacing: '0.01em', lineHeight: 1.4 },
      overline: {
        fontSize: '0.6875rem',
        letterSpacing: '0.08em',
        fontWeight: 600,
        lineHeight: 1.4,
        textTransform: 'uppercase',
      },
      button: {
        textTransform: 'none',
        fontWeight: 500,
        letterSpacing: '-0.005em',
        fontSize: '0.8125rem',
      },
    },
    shape: { borderRadius: 6 },
    components: {
      MuiCssBaseline: {
        styleOverrides: {
          body: {
            backgroundColor: t.bg,
            color: t.textHi,
            fontFamily: '"Instrument Sans", "DM Sans", system-ui, -apple-system, sans-serif',
          },
        },
      },
      MuiPaper: {
        defaultProps: { elevation: 0 },
        styleOverrides: {
          root: {
            backgroundImage: 'none',
            backgroundColor: t.bgEl,
            border: `1px solid ${t.border}`,
            boxShadow: 'none',
          },
        },
      },
      MuiCard: {
        defaultProps: { elevation: 0 },
        styleOverrides: {
          root: {
            backgroundImage: 'none',
            backgroundColor: t.bgEl,
            border: `1px solid ${t.border}`,
            boxShadow: 'none',
          },
        },
      },
      MuiButton: {
        defaultProps: { disableElevation: true },
        styleOverrides: {
          root: {
            borderRadius: 6,
            padding: '5px 12px',
            fontSize: '0.8125rem',
            fontWeight: 500,
            boxShadow: 'none',
            transition: 'background-color 100ms ease, border-color 100ms ease, opacity 100ms ease',
          },
          containedPrimary: {
            backgroundColor: t.accent,
            color: '#ffffff',
            '&:hover': { backgroundColor: t.accentHover, boxShadow: 'none' },
          },
          outlined: {
            borderColor: t.border,
            color: t.textHi,
            '&:hover': { backgroundColor: t.bgHover, borderColor: t.borderStr },
          },
          text: {
            color: t.textMid,
            '&:hover': { backgroundColor: t.bgHover, color: t.textHi },
          },
        },
      },
      MuiButtonGroup: {
        styleOverrides: {
          root: { boxShadow: 'none' },
          grouped: { '&:not(:last-of-type)': { borderRightColor: t.border } },
        },
      },
      MuiIconButton: {
        styleOverrides: {
          root: {
            color: t.textMid,
            borderRadius: 6,
            padding: 6,
            transition: 'background-color 100ms ease, color 100ms ease',
            '&:hover': { backgroundColor: t.bgHover, color: t.textHi },
          },
        },
      },
      MuiTab: {
        styleOverrides: {
          root: {
            color: t.textMid,
            fontSize: '0.8125rem',
            fontWeight: 500,
            textTransform: 'none',
            minHeight: 40,
            letterSpacing: '-0.005em',
            padding: '6px 14px',
            '&.Mui-selected': { color: t.textHi, fontWeight: 600 },
          },
        },
      },
      MuiTabs: {
        styleOverrides: {
          indicator: { height: 1, backgroundColor: t.textHi },
          root: { minHeight: 40 },
        },
      },
      MuiChip: {
        styleOverrides: {
          root: {
            backgroundColor: mode === 'dark' ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.05)',
            border: `1px solid ${t.border}`,
            color: t.textMid,
            fontSize: '0.6875rem',
            fontWeight: 500,
            height: 22,
            borderRadius: 4,
          },
          colorSuccess: {
            backgroundColor: mode === 'dark' ? 'rgba(54,179,126,0.12)' : 'rgba(54,179,126,0.1)',
            borderColor: mode === 'dark' ? 'rgba(54,179,126,0.3)' : 'rgba(54,179,126,0.2)',
            color: mode === 'dark' ? '#36b37e' : '#1a8c5a',
          },
          colorError: {
            backgroundColor: mode === 'dark' ? 'rgba(248,113,113,0.12)' : 'rgba(220,38,38,0.1)',
            borderColor: mode === 'dark' ? 'rgba(248,113,113,0.25)' : 'rgba(220,38,38,0.2)',
            color: mode === 'dark' ? '#f87171' : '#dc2626',
          },
          colorWarning: {
            backgroundColor: mode === 'dark' ? 'rgba(230,168,23,0.12)' : 'rgba(217,119,6,0.1)',
            borderColor: mode === 'dark' ? 'rgba(230,168,23,0.25)' : 'rgba(217,119,6,0.2)',
            color: mode === 'dark' ? '#e6a817' : '#b45309',
          },
          colorSecondary: {
            backgroundColor: t.accentFade,
            borderColor: mode === 'dark' ? 'rgba(94,106,210,0.25)' : 'rgba(79,92,200,0.15)',
            color: mode === 'dark' ? '#a5b4fc' : t.accent,
          },
        },
      },
      MuiDivider: {
        styleOverrides: { root: { borderColor: t.border } },
      },
      MuiInputLabel: {
        styleOverrides: {
          root: { color: t.textMid, fontSize: '0.8125rem' },
        },
      },
      MuiOutlinedInput: {
        styleOverrides: {
          root: {
            fontSize: '0.8125rem',
            '& .MuiOutlinedInput-notchedOutline': { borderColor: t.border },
            '&:hover .MuiOutlinedInput-notchedOutline': { borderColor: t.borderStr },
            '&.Mui-focused .MuiOutlinedInput-notchedOutline': {
              borderColor: t.accent,
              borderWidth: 1,
            },
          },
        },
      },
      MuiSelect: {
        styleOverrides: { icon: { color: t.textLo } },
      },
      MuiMenuItem: {
        styleOverrides: {
          root: {
            fontSize: '0.8125rem',
            '&:hover': { backgroundColor: t.bgHover },
            '&.Mui-selected': {
              backgroundColor: t.accentFade,
              '&:hover': { backgroundColor: t.accentFade },
            },
          },
        },
      },
      MuiSlider: {
        styleOverrides: {
          root: { color: t.accent, height: 2 },
          thumb: {
            width: 14,
            height: 14,
            boxShadow: 'none',
            '&:hover, &.Mui-focusVisible': { boxShadow: `0 0 0 6px ${t.accentFade}` },
          },
          track: { height: 2, border: 'none' },
          rail: { height: 2, opacity: 0.2 },
          mark: { display: 'none' },
          markLabel: { fontSize: '0.625rem', color: t.textLo },
        },
      },
      MuiLinearProgress: {
        styleOverrides: {
          root: {
            height: 2,
            borderRadius: 1,
            backgroundColor: mode === 'dark' ? 'rgba(255,255,255,0.06)' : 'rgba(0,0,0,0.06)',
          },
          bar: {
            borderRadius: 1,
            backgroundColor: t.accent,
          },
        },
      },
      MuiSwitch: {
        styleOverrides: {
          root: { width: 36, height: 20, padding: 0 },
          switchBase: {
            padding: 2,
            '&.Mui-checked': {
              transform: 'translateX(16px)',
              '& + .MuiSwitch-track': {
                backgroundColor: t.accent,
                opacity: 1,
                border: 0,
              },
            },
          },
          thumb: { width: 16, height: 16, boxShadow: 'none' },
          track: {
            borderRadius: 10,
            backgroundColor: mode === 'dark' ? 'rgba(255,255,255,0.15)' : 'rgba(0,0,0,0.15)',
            opacity: 1,
          },
        },
      },
      MuiTooltip: {
        styleOverrides: {
          tooltip: {
            backgroundColor: '#1e1e26',
            color: '#e8e8ec',
            fontSize: '0.6875rem',
            border: '1px solid #2e2e3a',
            borderRadius: 4,
            padding: '4px 8px',
          },
          arrow: { color: '#1e1e26' },
        },
      },
    },
  });
};

export const appTheme = makeTheme('dark');
