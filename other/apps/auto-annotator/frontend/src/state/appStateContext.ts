import { createContext, useContext } from 'react';
import type { AppStateContextValue } from './appState.types';

export const AppStateContext = createContext<AppStateContextValue | null>(null);

export const useAppState = () => {
  const stateContext = useContext(AppStateContext);
  if (!stateContext) {
    throw new Error('useAppState must be used within AppProvider');
  }
  return stateContext;
};
