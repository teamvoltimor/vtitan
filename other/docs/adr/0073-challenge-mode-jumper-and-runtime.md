# 0073. The challenge is read from a boot jumper and resolved at runtime

- Status: accepted
- Date: 2026-09-15

## Context

Relying on someone passing the right launch file by hand is how the wrong
challenge gets run. On one trace a Pi Zero that published its jumper reading one
second late was ignored for the rest of the round and the robot raced Open with
the jumper physically inserted: no in-bay start, no sign router, the wrong speed
ladder, and nothing on the wire to say so. Blind runs also always drove as Open
regardless of the jumper, because the track navigator never learned the resolved
challenge mode.

## Options considered

- (a) Fix the challenge at process start from the launch file.
- (b) Read a boot jumper and resolve the active challenge at runtime.

## Decision

(b). A two-pin jumper across GPIO23 and an adjacent GND, read on the Pi Zero with
the internal pull-up and no external resistor, selects the challenge: LOW (shorted)
is Obstacles, HIGH (absent) is Open. Three samples must agree
(`challenge_mode_samples_required = 3`); a disagreement is fail-closed, a warning
with `all_ready=False`, staying in BOOT_CHECK rather than letting the robot
silently pick a mode. The timeout fallback defaults to Open but is PROVISIONAL and
latches once; `challenge_mode_timeout_sec = 180.0`. The OLED shows the mode, a
neutral "Detecting challenge mode..." page, and a fault page only past a 5 s
grace, because every boot is unstable for the first second or two.

The active challenge is resolved at runtime: `track_navigator_node` loads both
challenge profiles eagerly and picks the active one plus a fresh
`SignRouter`/`ParkController` every time RACING is entered (including after a
long-press reset). A late jumper reading can correct the challenge only from
BOOT_CHECK or READY, never during RACING, because swapping `target_laps` and the
sign router under a running round is worse than the wrong challenge it would fix.
Per-challenge params and the downstream controllers must be re-derived when the
router is swapped.

The jumper is a boot-time confirmation layer, not a second source of truth: the
navigator still gates on the scenario metadata's `challenge_type`.

Bench teleop (`joy_teleop_node`) republishes `/ackermann_cmd` at 20 Hz. Steering
always follows the stick; drive speed is zero unless a dead-man button is held and
the joystick link is fresh. Exactly one of navigation or controller mode may run,
because both publish to `/ackermann_cmd` and nothing arbitrates between them.

## Consequences

- The robot cannot silently run the wrong challenge from a missing launch
  argument.
- An ambiguous reading fails closed instead of guessing.
- A late reading is recovered only before RACING; a race already started on the
  wrong mode is not corrected mid-round.

## History

- dce027ec 2026-07-25: implement the GPIO23 boot detection; fail-closed.
- 5130306b 2026-07-25: read the jumper on the Pi Zero, where it is wired (the Pi 5
  was reading its own unconnected pin and always saw Open).
- 83594c67 2026-07-25: joystick teleop for bench testing; dead-man gating.
- c8ea0d60 2026-08-08: raise the jumper wait from 15 s to 60 s (measured cold boot
  41 s to reach the pin).
- 047e0534 2026-08-09: stop the CHECK JUMPER page firing on every normal boot;
  5 s grace.
- a6e8ea1e 2026-08-11: resolve challenge mode at runtime; both profiles loaded,
  active one switched on every RACING entry.
- 00047e4c 2026-08-23: move `CompetitionSpecs` to TOML (see 0069).
- db38cfdb 2026-09-05: let a defaulted challenge mode correct itself; defaulted
  resolution becomes provisional.
- 04fbeb71 2026-09-06: raise the timeout 60 s to 180 s after a round scored as
  Open with the jumper arriving 92 s after boot.
- aa32e4e6 2026-09-11: let a late jumper reading correct the challenge from READY;
  RACING deliberately excluded.
- 1a6a46c3 2026-09-14: re-resolve per-challenge params on the router swap. 13,852
  driving ticks on Open tiers against 0 on Obstacles, in 9 of 9 rounds.
- 6334fd94 2026-09-14: rebuild the challenge controllers on a switch.
- 2026-08-06: a button-cycled rerun dropped straight back to FINISHED, because the
  finished race's stale lap count beat the navigator's post-reset zero; only a
  reboot cleared it.
- 2026-09-11: the Pi 5 timed out at 10:58:30 and the Zero published the jumper at
  10:58:51, 21 s late.
- 2026-09-12: the in-bay lot derivation deployed 2026-09-11 was inert; `reset()`
  rebuilt the controller from the raw metadata, got None and destroyed what
  construction built, so three hardware rounds reached three laps with
  `parking_engaged` null.

## Cross-references

- 0069 owns the competition specs the lap counts come from; 0070 owns the
  per-challenge overlays; 0050 owns the escape side the controllers carry.
