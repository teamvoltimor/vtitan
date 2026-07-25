/// <reference types="vitest/config" />
import { defineConfig, loadEnv } from 'vite';
import react from '@vitejs/plugin-react';

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '');
  // Proxy /v1 to the Go telemetry backend in dev so the app can run
  // same-origin (VITE_TELEMETRY_BASE left empty) without needing
  // backend-side CORS configuration just for local development.
  const backendUrl = env.VITE_TELEMETRY_BASE || 'http://localhost:8010';

  return {
    plugins: [react()],
    server: {
      proxy: {
        '/v1': {
          target: backendUrl,
          changeOrigin: true,
          ws: true,
        },
      },
    },
    build: {
      rollupOptions: {
        output: {
          manualChunks: {
            // three.js + react-three-fiber/drei (~600KB gzipped) rarely change
            // relative to app code — split into their own vendor chunk so
            // browsers cache it independently across app deploys.
            'vendor-three': ['three', '@react-three/fiber', '@react-three/drei'],
          },
        },
      },
    },
    test: {
      environment: 'happy-dom',
      globals: true,
    },
  };
});
