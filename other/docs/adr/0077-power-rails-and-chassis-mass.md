# 0077. Power rails are sized to measured load and the chassis mass is the battery midpoint

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0009

## Context

The robot is near a weight limit, and one branch was over-provisioned by guess:
the servo regulator (XLC4016) could deliver far more current than the servo branch
needs. The chassis mass also had to be one number for the simulation while two
battery configurations are flown.

## Options considered

- (a) Dimension each part against the imagined worst case; use the practice-battery
      or competition-battery mass.
- (b) Dimension each part against the measured load; use one deliberate middle mass.

## Decision

(b). Three power branches hang off the battery independently so a 20 A traction
spike cannot sag the Pi 5 and reset it mid-round: a KL89576 DC-to-USB-C converter
for the Pi 5 (5 V at 5 A, deliberately fine margin monitored with
`vcgencmd get_throttled`), a Step Down Mini-560 Pro for the servo rail, and the
BTS7960 for traction. The AI HAT+, camera, LIDAR and IMU are powered from the Pi
5's own 5 V rail, not summed again, and the Pi Zero is fed via the Pi 5's USB
VBUS.

The Mini-560 Pro replaced the XLC4016 because the servo branch needs well under
what the XLC4016 could supply: 24 g against 5 g, a 19 g saving for capacity that
would never be used. The general rule is to dimension each piece against the
measured load, not the worst case imaginable. Two Ovonic 3S packs are kept: a
3000 mAh 50C for practice and a 2200 mAh 120C XT60 shorty for competition, chosen
on weight and burst current (46 g lighter, double C-rating), not capacity.

`chassis.mass = 1.3` kg is the chassis body ALONE, because consumers sum it with
four wheel masses of 0.05 kg: `1.3 + 4 x 0.05 = 1.5` kg. That is a deliberate
middle value between the practice configuration (~1.51 kg) and the competition
configuration (~1.46 kg), so the simulation never under-predicts required torque.

## Consequences

- 19 g recovered on the servo regulator; the assembled car is within the weight
  limit.
- The computer branch margin is deliberately thin and monitored rather than
  over-provisioned unmeasured.
- Change chassis and wheel mass together; mass drives the inertia tensors.
- The on/off switch's continuous current capacity is still unmeasured, the only
  qualitative margin left.

## History

- 617e1905 2026-08-01: set `chassis.mass` 0.8 to 1.3 kg (total 1.5 kg measured).
- 36c363eb, dd5264de, be27129a 2026-08-11 to 2026-09-05: build and correct the
  power budget; L298N to BTS7960; traction branch into the sum.
- 3e05a320 2026-09-05: measured regulator weights (24 g to 5 g, -19 g).
- 197f3d6f 2026-09-07: resolve the computer-branch non-additivity.
- f475dc07, 823c6129, 20a734d6 2026-09-07: document both packs; competition pack
  140 g, saving corrected to 46 g.
- a23d725a 2026-09-07: split robot mass by configuration (1.51 practice / 1.46
  competition); keep the modelled 1.5 kg.
- 8f754cf8 2026-09-09: close the numeric contradictions; battery sufficiency ~9 min
  at ~14-16 A nominal / ~31 A peak.

## Refuted

- L298N (2 A per channel) against the measured traction current.
- XLC4016 for weight; smaller batteries; using the practice pack in competition.
- A naive peak sum for the computer branch (the Pi 5 5 A spec already covers the
  HAT, camera, LIDAR, IMU and Zero).
- The stale ~5 A / ~20 A / 26-minute battery story.

## Cross-references

- 0009 is superseded; its mass decision is carried above.
- 0076 owns the H-bridge and traction branch.
