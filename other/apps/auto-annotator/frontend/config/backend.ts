export const DEFAULT_BACKEND_URL = 'http://localhost:8000';

export const resolveBackendUrl = (override?: string) => {
  if (override !== undefined) {
    return override;
  }
  return DEFAULT_BACKEND_URL;
};
