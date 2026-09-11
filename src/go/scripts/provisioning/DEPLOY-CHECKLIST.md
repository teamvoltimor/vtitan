# DEPLOY-CHECKLIST.md - vTitan Go robot-go → Pi

Generated: 2026-08-30 (agent J, §5c). No Pi is reachable from this environment;
these are the **manual on-device steps** an operator runs. Do NOT push from CI.

## Prerequisites (verified here)
- `CGO_ENABLED=0 GOOS=linux GOARCH=arm64 go build ./cmd/...` exits **0**.
- `go build ./...` (whole tree) is **GREEN** - no Go source edited for this task.
- Binaries produced (11): `motor-node`, `imu-node`, `lidar-node`, `navigator`?
  Actual: `bench-harness` `foxglove-bridge` `imu-node` `lidar-node` `motor-node`
  `pi-zero` `pi5` `sim-runner` `state-machine` `telemetry-node` `track-navigator`.
- `scripts/provisioning/build-go.sh` stages + ships the whole `src/config/`
  tree to `$INSTALL_DIR/src/config` on the Pi (updated 2026-09-10: it used
  to stage only `configs/profiles/*.toml`, a directory that only ever held
  `.gitkeep`, so the binaries silently ran on Go's hardcoded defaults on
  every real deploy). The systemd units now also pass
  `--config-root=/opt/vtitan-go` on ExecStart -- without it, `ConfigFor()`
  returns `DefaultConfig()` unconditionally regardless of what's staged.
- Motor-safety: `cmd/motor-node` (shared by `cmd/pi-zero`) calls
  `driver.Close()` → `forceGPIOLow` (driver.go:203) on SIGINT/SIGTERM via
  `signal.NotifyContext` (main.go:142). Defense-in-depth: `vtitan-go-pi-zero.service`
  `ExecStopPost` re-drives BTS7960 L_EN/R_EN/reverse pins low (pi-zero unit:40).
  Both layers present.

## Config drift check (pre-flip)
- Go `profile` package TOMLs are loaded from `src/config/` (deploy copies the
  whole tree; `internal/config/profile/*.go`'s `Default*TOMLPath` constants
  are each `src/config/...`, joined with `--config-root` at runtime). Confirm
  `VTITAN_HARDWARE_PROFILE` is set (env or `.env` on the Pi) so the active
  profile overlay under `src/config/profiles/<name>/` actually applies -- an
  unset profile silently means base config only, same as the Python stack.
- Unported sim fields (drift, expected - harness §5a not yet consuming them):
  `start_collision_window_s`, `start_collision_grace_s`, `lidar_invalid_ray_rate`,
  `detection_confidence`. Rule these out in any parity comparison.

## On-device steps
1. From dev machine, ship binaries + profiles:
   ```sh
   TARGET_HOST=user@pi5-host bash scripts/provisioning/build-go.sh user@pi5-host
   # repeat with user@pi-zero-host for the Zero
   ```
2. Install units:
   ```sh
   sudo cp systemd/vtitan-go-pi5.service systemd/vtitan-go-pi-zero.service \
     systemd/vtitan-go-lidar.service systemd/vtitan-go-imu.service systemd/vtitan-go-navigator.service \
     systemd/vtitan-robot@.service /etc/systemd/system/
   sudo systemctl daemon-reload
   ```
   `vtitan-go-lidar`/`vtitan-go-imu`/`vtitan-go-navigator` are new (2026-09-11):
   sibling processes to `vtitan-go-pi5` (camera capture only), together
   replacing the Python stack's `vtitan-pi5`+`vtitan-race` on the Pi 5. They
   own the LIDAR/IMU hardware directly and CANNOT run at the same time as
   Python's `vtitan-pi5.service`/`vtitan-lidar.service` (both sides open the
   same serial ports) -- enable them only when running the Go stack.
   `vtitan-go-navigator` runs `track-navigator` with no `--metadata`, always
   estimating the corridor layout from LIDAR (see its package doc comment);
   `--direction` defaults to `undetermined` and should stay that way unless
   an operator genuinely knows the round's draw in advance.
3. Verify motor-safety stop hook is in the unit:
   ```sh
   systemctl cat vtitan-go-pi-zero.service | grep -A1 ExecStopPost
   # expect: pinctrl set <L_EN>,<R_EN>,<REVERSE> op dl
   ```
4. Pre-flip parity diff (rule out drift):
   ```sh
   diff <(cat src/config/navigation/**/*.toml) <(cat configs/profiles/*.toml)
   ```
5. Flip to Go stack (whole-stack swap):
   ```sh
   sudo systemctl start vtitan-robot@go
   ```
6. Revert to Python:
   ```sh
   sudo systemctl start vtitan-robot@python
   ```
7. Confirm health:
   ```sh
   sudo systemctl status vtitan-go-pi5.service vtitan-go-pi-zero.service \
     vtitan-go-lidar.service vtitan-go-imu.service vtitan-go-navigator.service
   ls -l /opt/vtitan-go/bin
   ```
