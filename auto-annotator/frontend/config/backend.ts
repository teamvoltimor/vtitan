export const DEFAULT_BACKEND_URL = 'http://localhost:8000';

export const resolveBackendUrl = (override?: string) => {
  if (override && override.trim().length > 0) {
    return override;
  }
  return DEFAULT_BACKEND_URL;
};
