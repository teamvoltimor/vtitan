import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';
import { resolveBackendUrl } from './config/backend';

const backendUrl = resolveBackendUrl(process.env.VITE_API_BASE_URL ?? process.env.API_BASE_URL);

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    // Proxy API routes to the FastAPI backend in dev mode.
    // Mirrors the nginx proxy rules in webapp/nginx.conf so that setting
    // VITE_API_BASE_URL (or API_BASE_URL) produces identical behaviour locally and in Docker.
    proxy: {
      '/gallery': backendUrl,
      '/images': backendUrl,
      '/segment': backendUrl,
    },
  },
});
