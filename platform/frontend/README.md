# vTitan Telemetry Dashboard

React + TypeScript + Vite dashboard for visualising robot telemetry (3D scene,
sensor panels, ROS topic inspector) streamed from the Go telemetry backend.

## Demo Mode (mock data, no backend)

Demo Mode synthesises realistic telemetry locally so you can explore the UI
without running the backend. The mock data uses the exact backend wire shape, so
it flows through the same schemas and components as a live connection.

Enable it any of these ways:

- **Toggle button** — click **Demo Mode** (top-left of the 3D canvas) at runtime.
- **Error screen** — if the backend is unreachable, click **Launch Demo Mode**.
- **URL** — append `?demo` (or `?demo=true`) to the page URL; `?demo=false` forces it off.
- **Env var** — set `VITE_DEMO=true` (e.g. in `.env.development`) to boot into Demo Mode.

In Demo Mode a simulated robot drives a loop around the WRO track: LiDAR traces
the track walls, the path trail accumulates, and IMU/vision/motor/topic panels
animate. Two mock replay "sessions" are available in the Replays list.

```bash
npm install
npm run dev          # then click "Demo Mode", or open http://localhost:5173/?demo
```

---

## React + TypeScript + Vite

Built with Vite + `@vitejs/plugin-react` (Babel Fast Refresh). Linting and
formatting are handled by Biome (`pnpm lint`, `pnpm format`) — there is no
ESLint config in this project.

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).
