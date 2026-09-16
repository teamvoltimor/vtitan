# 0034. Wrong-side passes are scored from ground truth

- Status: superseded by 0059
- Superseded by: 0059
- Date: 2026-09-13

## Context

WRO rule 9.24.5 makes a wrong-side sign pass a round-ending offence. The Go
runner previously read the verdict from `SignRouter.WrongSideViolations` at the
end of a run. That is wrong three times over: the router scores itself in the
BELIEVED frame, the round never stopped on a violation (so runs banked laps the
rules would have denied), and `ResetForNewLap` clears the router's set at every
lap boundary, erasing every violation before the last.

## Options considered

- (a) Keep reading the router's own record.
- (b) An independent scorer over the TRUE layout and TRUE pose.

## Decision

(b). `scenario.passSideScorer` ports the Python `PassSideScorer`: it computes
the sign's radius line from the true corridor and direction, and records an
offence only when the chassis FOOTPRINT completely crosses the radius while on
the forbidden side. Until the line is fully crossed the rules explicitly permit
the vehicle to fix its side, so nothing is decided. A violation is never cleared
by a lap boundary, because one already committed does not stop being one.

The router's record is still kept, as a measure of discovery quality -- it just
must not be what ends a run.

## Consequences

- A run that commits a wrong-side pass ends there, matching the rule, instead of
  banking laps the rules would deny.
- The scorer needs the true sign list, direction and chassis rectangle, so it
  lives in the simulator's scenario layer rather than the navigator.

## Superseded by 0059

Replaced by [0059](0059-pass-side-travel-relative-and-scorer-independence.md): The
pass-side rule is travel-relative, and a scorer must not share the scored system's
convention.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
