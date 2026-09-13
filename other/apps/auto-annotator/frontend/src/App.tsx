import { CssBaseline, ThemeProvider } from '@mui/material';
import { useState } from 'react';
import AppShell from './components/AppShell';
import { AppProvider } from './state/appState';
import { makeTheme } from './theme';

function App() {
  const [mode, setMode] = useState<'dark' | 'light'>('dark');

  return (
    <ThemeProvider theme={makeTheme(mode)}>
      <CssBaseline />
      <AppProvider>
        <AppShell onToggleTheme={() => setMode((m) => (m === 'dark' ? 'light' : 'dark'))} />
      </AppProvider>
    </ThemeProvider>
  );
}

export default App;
