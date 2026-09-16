# 0032. Heading correction is a single crawl threshold

- Status: superseded by 0085
- Superseded by: 0085
- Date: 2026-08-09

## Context

`heading.CRAWL` was once a four-rung ladder (CRAWL/SLOW/MEDIUM/NORMAL). The
three intermediate rungs were deleted 2026-08-09 because they cost 33 percent of
lap time: clockwise 134.9 to 179.3 s and counter-clockwise 161.7 to 200.9 s. It
is now a binary cliff in the navigator: heading error at or above CRAWL selects
`creep_mps`, otherwise `fast_mps`.

This is a disguised speed reduction, not a compensating lever. Ordinary
cornering sits at 23-45 deg of heading error, so dropping the trigger to 0.3 rad
(~17 deg) makes nearly every corner creep. Swept at a 0.60 ceiling, 0.3 rad is
the only value that clears the +/-4-case noise floor:

| CRAWL (rad) | 0.6 | 0.5 | 0.4 | 0.3 | 0.2 | 0.1 |
|---|---|---|---|---|---|---|
| ok/128 | 113 | 115 | 116 | 123 | 116 | 110 |

The tail bounds the reading: if CRAWL were purely speed reduction, 0.1 would
approach the 0.156-everywhere arm's 127, but it gives 110, so creeping on
straights costs tier thrash. The creep floor is what does the safety work:
raising it to 0.30 is the only variant faster than baseline, and its regressions
go into collisions rather than timeouts.

## Options considered

- (a) Keep the four-rung ladder.
- (b) Delete the intermediate rungs and keep one threshold.

## Decision

(b). One threshold, consumed as a binary cliff. The deleted rungs cost a third
of lap time at the ceiling where they were measured.

## Consequences

- CRAWL has a narrow optimum near 17 deg, close to the band ordinary cornering
  occupies; it has to be tuned, not pushed.
- It does real work only at speed: worth +17 cases at 0.80, noise-level at 0.60.

## Superseded by 0085

Replaced by [0085](0085-speed-envelope.md): The speed envelope is absolute m/s with the
drivetrain as a clamp.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
