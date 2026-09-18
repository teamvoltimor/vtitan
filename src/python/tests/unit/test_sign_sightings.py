"""Colour, range and bearing WITHOUT a pose, and why the pass-side rule needs them.

``_sign_dodge_side`` is the only pass-side rule the robot applies while the
travel direction is still unknown, and its docstring claims frame independence:
"the world-frame lookup REQUIRES a direction, but this does not". That was half
true. Mapping colour to a side needs no frame, but WHICH sign it obeyed, and the
activation gate that decided whether to obey one at all, were a world distance
from the believed pose -- computed with a heading recorded against a PROVISIONAL
frame that a counterclockwise round overturns by pi once the direction settles.

MEASURED on the 2026-09-15 in-bay rounds: of the observations accepted during a
counterclockwise bay exit, 1.8% land within 0.35 m of any pillar the round later
believes in, against 84.3% clockwise, where the yaw is not overturned. So in the
direction that fails most, both the nearest-sign pick and the range gate were
reading a position reflected across the mat.

These cover the frame-free path that replaces it. See
``adr:0053-direction-inference-and-start-pose`` and
``adr:0058-sign-discovery-range-and-barrier-belief``.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import CorridorDimensions, RobotSpecs, TrafficSignSpecs
from shared.domain.enums import Direction, Section
from shared.domain.models import Detection, SignColor

from src.navigation.planning.sign_discovery import detection_body_frame
from src.navigation.planning.sign_router import SignSpec
from src.simulation.kinematics import AckermannState
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.simulated_hardware_gateway import SimulatedHardwareGateway
from src.simulation.track_model import TrackModel

_WIDE_MM = int(CorridorDimensions.WIDE * 1000)


def _box(*, center_x: float, height_px: float) -> Detection:
    """A detection whose bbox is all these helpers read."""
    half_w = height_px / 4
    x_min, x_max = center_x - half_w, center_x + half_w
    y_min, y_max = 100.0, 100.0 + height_px
    return Detection(
        color=SignColor.RED,
        confidence=0.9,
        bbox=(x_min, y_min, x_max, y_max),
        x=center_x,
        y=(y_min + y_max) / 2,
        width=2 * half_w,
        height=height_px,
        area=2 * half_w * height_px,
    )


class TestDetectionBodyFrame:
    def test_a_centred_box_reads_zero_bearing(self) -> None:
        body = detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH / 2, height_px=120))

        assert body is not None
        assert body[1] == pytest.approx(0.0, abs=1e-9)

    def test_bearing_is_positive_to_the_left(self) -> None:
        """The sign convention, pinned.

        The robot frame is 0 = forward, +pi/2 = left, and an earlier version of
        this arithmetic elsewhere was positive for a box on the RIGHT of the
        image. That reflected every sign across the heading axis and, in a 1 m
        corridor, landed it on the far wall. Nothing in-tree caught it, because
        the emulator and the router's tests both built the pixel column from the
        same wrong formula. See
        adr:0058-sign-discovery-range-and-barrier-belief.
        """
        left_of_frame = detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH * 0.25, height_px=120))
        right_of_frame = detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH * 0.75, height_px=120))

        assert left_of_frame is not None
        assert right_of_frame is not None
        assert left_of_frame[1] > 0.0
        assert right_of_frame[1] < 0.0

    def test_range_follows_the_pinhole_and_shrinks_as_the_box_grows(self) -> None:
        near = detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH / 2, height_px=240))
        far = detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH / 2, height_px=60))

        assert near is not None
        assert far is not None
        # d = f * H / h, so halving the box doubles the range. Asserted as the
        # RATIO rather than two absolutes, which would also pass if the focal
        # length or RANGE_SCALE silently changed.
        assert far[0] == pytest.approx(4 * near[0], rel=1e-9)

    def test_a_box_below_the_reliable_floor_is_refused(self) -> None:
        """Same floor the world projection applies, not a second opinion."""
        assert detection_body_frame(_box(center_x=RobotSpecs.CAMERA_WIDTH / 2, height_px=1)) is None

    def test_no_pose_is_involved_at_all(self) -> None:
        """The property the blind phases depend on, asserted rather than assumed.

        There is no pose ARGUMENT to vary, so the only way to state this is that
        the answer is a pure function of the box. A future refactor that reached
        for a module-level pose would break here.
        """
        det = _box(center_x=RobotSpecs.CAMERA_WIDTH * 0.4, height_px=150)

        assert detection_body_frame(det) == detection_body_frame(det)


def _gateway(signs: list[SignSpec], yaw: float) -> SimulatedHardwareGateway:
    metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
    from src.navigation.track_geometry import corridor_widths_from_metadata

    return SimulatedHardwareGateway(
        track=TrackModel(corridor_widths_from_metadata(metadata.model_dump())),
        initial_state=AckermannState(x=1.5, y=0.5, yaw=yaw),
        signs=signs,
    )


class TestSimulatedSightings:
    def test_a_sign_ahead_is_reported_with_its_true_range_and_side(self) -> None:
        gw = _gateway([SignSpec(2.0, 0.5, SignColor.GREEN)], yaw=0.0)

        sightings = gw.get_sign_sightings()

        assert len(sightings) == 1
        assert sightings[0].color is SignColor.GREEN
        assert sightings[0].range_m == pytest.approx(0.5, abs=1e-6)
        assert sightings[0].bearing_rad == pytest.approx(0.0, abs=1e-6)

    def test_a_sign_to_the_left_reads_a_positive_bearing(self) -> None:
        # Off to the left but still INSIDE the cone: the half-FOV is about
        # 51 deg, so a sign at a true 90 deg is not a left-hand sighting, it is
        # an invisible one -- which the next test covers.
        gw = _gateway([SignSpec(2.0, 0.8, SignColor.RED)], yaw=0.0)

        sightings = gw.get_sign_sightings()

        assert len(sightings) == 1
        assert sightings[0].bearing_rad == pytest.approx(math.atan2(0.3, 0.5), abs=1e-6)
        assert 0.0 < sightings[0].bearing_rad < RobotSpecs.CAMERA_HFOV / 2

    def test_a_sign_behind_the_camera_is_not_reported(self) -> None:
        """The half-FOV cut, which is what keeps this from being omniscient."""
        gw = _gateway([SignSpec(1.0, 0.5, SignColor.RED)], yaw=0.0)

        assert gw.get_sign_sightings() == []

    def test_asking_does_not_consume_the_vision_rng(self) -> None:
        """Observing must not change the simulation.

        ``_detectable_signs`` draws from ``_vision_rng`` twice per sign per
        call, so a reader routed through it would shift every later tick's
        draws. This accessor bypasses it, which is what makes it safe to call
        alongside ``get_vision_detections`` in the same tick. The price is
        stated in the method's docstring: it is more generous than the emulated
        detector, so it cannot be used to score detection RATES.
        """
        gw = _gateway([SignSpec(2.0, 0.5, SignColor.GREEN)], yaw=0.0)
        state_before = gw._vision_rng.bit_generator.state

        reads = [gw.get_sign_sightings() for _ in range(3)]

        assert gw._vision_rng.bit_generator.state == state_before, "reading drew from the vision RNG"
        assert reads[0] == reads[1] == reads[2], "repeated reads in one tick must agree"

    def test_the_emulated_detector_by_contrast_does_consume_it(self) -> None:
        """The control, so the test above cannot pass by the RNG being unused.

        Without this, a future change that stopped drawing anywhere would leave
        the assertion above green while proving nothing.
        """
        gw = _gateway([SignSpec(2.0, 0.5, SignColor.GREEN)], yaw=0.0)
        state_before = gw._vision_rng.bit_generator.state

        gw._detectable_signs()

        assert gw._vision_rng.bit_generator.state != state_before

    def test_the_reading_does_not_move_when_the_believed_pose_does(self) -> None:
        """The whole point, stated against this gateway's own belief machinery.

        ``get_vision_detections`` deliberately reprojects through the BELIEVED
        pose, so a diverged estimate moves every reported sign. These three
        numbers must not move, because nothing projected them.
        """
        gw = _gateway([SignSpec(2.0, 0.5, SignColor.GREEN)], yaw=0.0)
        before = gw.get_sign_sightings()

        gw._estimator.update_position(0.3, 2.7)

        assert gw.get_sign_sightings() == before
