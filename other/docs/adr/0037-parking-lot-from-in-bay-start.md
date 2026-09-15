# 0037. The parking lot is derived from the in-bay start

- Status: superseded by 0062
- Superseded by: 0062
- Date: 2026-09-11
- Commit: 996c7415

## Context

Parking was UNREACHABLE CODE on hardware. Scanned across 255 bags (227
readable): zero had `parking_engaged` non-null, i.e. a `ParkController` was
never CONSTRUCTED, while 67 bags reached three laps, so the lap precondition was
met. The controls in the same pass are all non-zero (normal_drive 260566 ticks,
bay_exit 35949, blind_creep 26523), so the reader works and the null is real.
Cause: hardware runs carry no `metadata_path`, so the navigator holds only
`starting_conditions`, and the factory returns None when `parking_lot` is
absent.

The lot needs no sensing to be located: in the Obstacles Challenge the robot
STARTS INSIDE IT, so the start pose IS the lot, with the fins
`spacing_factor` CHASSIS lengths apart (0.45 m). Using the fin's own length
gives 0.30 m and puts the fins inside the chassis.

## Options considered

- (a) Require `parking_lot` metadata for parking to run.
- (b) Derive the lot from the in-bay start pose when metadata carries none.

## Decision

(b). `derive_lot_from_in_bay_start` defaults ON. "It never tried" is a worse
failure than "it tried and could not".

## Consequences

- The parking attempt is now visible and its failure measurable on hardware.
- It will probably still fail, and that is geometry, not this flag: 6 mm of
  depth slack and 1.16 deg of heading tolerance. Unlimited reverse legs do not
  open it (a forward+reverse pair nets about 0.14 mm of lateral at R = 0.33 m).
