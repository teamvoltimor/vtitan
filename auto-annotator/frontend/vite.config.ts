import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    // Proxy API routes to the FastAPI backend in dev mode.
    // Mirrors the nginx proxy rules in webapp/nginx.conf so that setting
    // VITE_API_BASE_URL="" produces identical behaviour locally and in Docker.
    proxy: {
      '/gallery': 'http://localhost:8000',
      '/images': 'http://localhost:8000',
      '/segment': 'http://localhost:8000',
    },
  },
})
