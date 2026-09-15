# 0044. The camera autofocus is set to 1.25 dioptres

- Status: superseded by 0078
- Superseded by: 0078
- Date: 2026-09-11

## Context

The lens shipped at 0.8 D (focus 1.25 m, in-focus from 0.63 m). MEASURED over
7149 red/green boxes on four hardware rounds, range from the 0.10 m sign height
and a 621.9 px focal: p10 0.19 / p50 0.40 / p90 0.68 m, with 83.6 percent INSIDE
the old near limit and 99.9 percent inside 1.0 m. The lens was focused past
nearly everything it actually sees.

Chosen for the DECISION range, not the median box: the median is dominated by
frames of a sign already being passed, whose verdict was settled earlier. What
matters is first-usable-detection 0.824 m down to router commitment 0.469 m, and
0.485 m sits right on that lower edge. 1.25 D focuses 0.80 m and spans
0.485-2.29 m. 1.43 D centres the window better and is the value to try once the
dioptre scale is bench-verified, which it is still not on either driver.

Measured after the change and NOT reverted, deliberately: on the two rounds that
followed, confidence in the 0.0-0.5 m band fell with clean run-level separation
(before 0.831/0.842/0.862/0.869/0.902, after 0.767/0.771, no overlap, and the
drop survives inside every yaw bin). But the rounds differ in time and the
LIGHTING CHANGED, and one round that evening was discarded outright for poor
lighting. Operator's call 2026-09-11: the drop is attributable to light, keep
1.25 D. The optics agree: at 1.25 D the near in-focus limit moves 0.63 to
0.485 m, so this band should improve or hold, never worsen.

## Options considered

- (a) Keep 0.8 D.
- (b) 1.25 D.
- (c) 1.43 D.

## Decision

(b). What would settle the confidence question is a matched-lighting A/B, or the
bench check of the dioptre scale this file has always said is unverified.

## Consequences

- The near limit moves inside the decision range instead of starting past it.
- The confidence dip measured after the change is attributable to lighting on
  the operator's call, not to the optics.
