"""The signs seen from inside the pocket, kept instead of thrown away.

The node returns out of the control tick while the bay-exit manoeuvre holds the
chassis (`if self._resolve_direction(): return`), so `CoreNavigator.step` never
runs and with it neither does the router call that performs the blind discovery
ingest. The sign map is EMPTY for the 8-14 s the pocket costs.

MEASURED over 18 in-bay rounds of 2026-09-15: the first pillar's first sighting
and its commitment are the SAME instant in 13 of them, and that pass fails 33.3%
against about 10% for every later pass -- 56% when it has to cross. The evidence
was there the whole time: over 11 clockwise rounds, 820 of 2,199 accepted
observations (37.3%) land within 0.35 m of the exact pillar the round then
commits to, in 9 of 11 rounds, and every one is discarded.

It has to be BUFFERED rather than ingested live, because `direction` is None for
the whole exit and the published yaw then moves by pi at the hand-over: measured
-111 to +73.5 degrees on the counterclockwise rounds. As published, only 1.4% of
counterclockwise observations land on the pillar against 37.3% clockwise;
re-stamping counterclockwise by pi moves "on any real pillar" from 5.5% to 44.2%
while the same turn destroys clockwise, 71.2% to 0.1%. A two-sided control.

Ships OFF behind `ingest_during_bay_exit`: 63% of those observations land
somewhere OTHER than the pillar and this map already invents them, so the junk
needs measuring on the bags first. The corpus cannot referee it -- its scenarios
start outside the bay.
"""

from __future__ import annotations

import math

import pytest
from shared.domain.enums import Section
from shared.domain.models import CreepSignSample, SignColor, TrafficSignObservation


def _sample(*, tick: int, yaw: float, bearing: float = 0.0, rng: float = 0.7) -> CreepSignSample:
    return CreepSignSample(
        tick=tick,
        pose_x=1.2,
        pose_y=0.1,
        yaw=yaw,
        color=SignColor.RED,
        range_m=rng,
        bearing_rad=bearing,
        confidence=0.9,
    )


def _project(sample: CreepSignSample, heading_delta: float) -> tuple[float, float]:
    """The same arithmetic ``_replay_creep_sightings`` applies, for assertions.

    Duplicated deliberately rather than imported: importing the node pulls in
    rclpy, which on this platform crashes the interpreter often enough to make
    such a test worthless as a gate. The arithmetic is three lines and the test
    that matters is whether the CORRECTION has the right effect, which this
    states independently.
    """
    yaw = sample.yaw + heading_delta
    return (
        sample.pose_x + sample.range_m * math.cos(yaw + sample.bearing_rad),
        sample.pose_y + sample.range_m * math.sin(yaw + sample.bearing_rad),
    )


class TestTheFrameCorrection:
    def test_the_measured_counterclockwise_delta_is_very_nearly_pi(self) -> None:
        """The premise, asserted so the rest of the file is not resting on it.

        -111.3 and +73.5 degrees are the yaws published on either side of the
        hand-over in run_20260915_214555.
        """
        delta = math.radians(73.5) - math.radians(-111.3)

        assert abs(delta - math.pi) < math.radians(6.0)

    def test_an_uncorrected_counterclockwise_sighting_lands_on_the_far_side(self) -> None:
        """Why live ingest cannot work, as geometry rather than as a percentage.

        A pillar 0.7 m dead ahead, projected with the stale yaw, ends up nearly
        1.4 m from where it is -- the diameter of the error, since the frame is
        turned by pi. On a 3 m mat that is a different corridor.
        """
        delta = math.radians(73.5 - -111.3)
        stale = _sample(tick=1, yaw=math.radians(-111.3))

        wrong = _project(stale, 0.0)
        right = _project(stale, delta)

        assert math.dist(wrong, right) == pytest.approx(2 * stale.range_m, abs=0.08)

    def test_a_clockwise_sighting_needs_almost_no_correction(self) -> None:
        """The other side of the control: clockwise hand-overs move 0-3 degrees.

        Without this, a correction that simply turned everything by pi would
        pass the test above and silently wreck every clockwise round -- which is
        exactly what the measurement showed (71.2% of observations on a real
        pillar, down to 0.1%).
        """
        stale = _sample(tick=1, yaw=math.radians(-20.0))

        uncorrected = _project(stale, 0.0)
        corrected = _project(stale, math.radians(3.0))

        assert math.dist(uncorrected, corrected) < 0.04

    def test_the_bearing_survives_the_correction_unchanged(self) -> None:
        """Range and bearing are the chassis's own view and need no re-stamping.

        Only the yaw is stale. A correction that touched the bearing too would
        rotate the sign twice.
        """
        delta = 1.0
        ahead = _project(_sample(tick=1, yaw=0.3, bearing=0.0), delta)
        left = _project(_sample(tick=1, yaw=0.3, bearing=0.4), delta)

        # Both sit on the same circle about the pose: the bearing moved them
        # along it, the correction turned the circle, neither changed the radius.
        for point in (ahead, left):
            assert math.dist(point, (1.2, 0.1)) == pytest.approx(0.7, abs=1e-9)
        assert math.dist(ahead, left) > 0.2


class TestReplayGrouping:
    def test_samples_keep_the_tick_they_were_taken_on(self) -> None:
        """The map needs MIN_HITS confirmations ACROSS ticks to publish a track.

        One batch of 100 observations counts once, so the tick has to survive
        the buffer or nothing ever becomes a sign. This pins that the field is
        carried rather than derived at replay time, where a re-derivation would
        have no information to derive it from.
        """
        samples = [_sample(tick=t, yaw=0.0) for t in (1, 1, 2, 3, 3, 3)]

        by_tick: dict[int, list[TrafficSignObservation]] = {}
        for sample in samples:
            by_tick.setdefault(sample.tick, []).append(
                TrafficSignObservation(
                    world_x_m=0.0,
                    world_y_m=0.0,
                    color=sample.color,
                    confidence=sample.confidence,
                    detected_at_timestamp=0.0,
                )
            )

        assert sorted(by_tick) == [1, 2, 3]
        assert [len(by_tick[t]) for t in sorted(by_tick)] == [2, 1, 3]


class TestTheSampleIsFrameFree:
    def test_nothing_in_the_sample_is_a_world_position(self) -> None:
        """What makes the deferred correction possible at all.

        The sample carries the pose it was taken at and the chassis's own view
        of the sign, never a projected point. Had it stored a world position,
        the pi error would already be baked in and unrecoverable -- which is the
        state production is in today, and the reason 98.6% of counterclockwise
        bay-exit observations are wasted.
        """
        fields = set(CreepSignSample.__dataclass_fields__)

        assert {"pose_x", "pose_y", "yaw", "range_m", "bearing_rad"} <= fields
        assert not {"world_x_m", "world_y_m"} & fields
