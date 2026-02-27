import { ThemeProvider, CssBaseline } from '@mui/material'
import AppShell from './components/AppShell'
import { AppProvider } from './state/appState'
import { appTheme } from './theme'

function App() {
  return (
    <ThemeProvider theme={appTheme}>
      <CssBaseline />
      <AppProvider>
        <AppShell />
      </AppProvider>
    </ThemeProvider>
  )
}

export default App
