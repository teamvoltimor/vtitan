# 0023. Escape durations are stored in seconds, not frames

- Status: accepted
- Date: 2026-09-13

## Context

Every escape length, the stuck timeout and the parking give-up were once stored
as a frame count. A stored frame count silently means a different duration at a
different loop rate: 40 frames is 2 s at the shipped 20 Hz and 0.8 s at 50 Hz.
Changing the control loop would have shifted every one of those durations
together, with nothing raising and no config edited.

## Options considered

- (a) Store frame counts and accept that they are loop-rate dependent.
- (b) Store seconds and convert to ticks at the point of use.

## Decision

(b). The TOMLs hold seconds. `profile.Frames(seconds, controlHz)` converts each
one, rounding rather than truncating and flooring at a single tick: a duration
shorter than one tick is still a maneuver the caller asked for, and zero frames
would skip it entirely. `core_navigator`'s own escape fields convert the same
way at load.

## Consequences

- A control-rate change rescales every escape duration consistently instead of
  silently retuning them.
- Frame counts remain the runtime currency, so the consumers are unchanged.
