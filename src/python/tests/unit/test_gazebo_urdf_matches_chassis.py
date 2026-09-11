"""The Gazebo robot description must describe the same car the headless sim drives.

Two defects found together on 2026-08-22, both of which had been sitting in the
committed URDF and neither of which anything checked:

1. **It did not parse at all.** Three XML comments contained ``--``, which is
   illegal inside a comment, so ``xacro.process_file`` raised ExpatError on line
   4. Every Gazebo launch that depends on the robot description was therefore
   dead, and nothing said so -- the failure is a launch-time parse error in a
   path no test touched.

2. **It was a front-steer car.** The rear wheels were plain ``continuous``
   joints bolted straight to ``base_link``, while the real chassis and
   ``src/simulation/kinematics.py`` are counter-phase four-wheel steer. That is
   not cosmetic: counter-phase moves the instantaneous centre of rotation from
   the rear axle to the chassis centre, so the robot yaws TWICE as fast at a
   given steering angle. It is the same defect that was already found and fixed
   once on the headless side, where it had quietly made every gain tuned
   against the sim hotter on the real robot.

These are cheap assertions about a file that is otherwise only exercised by
launching Gazebo -- which does not run on Windows at all (the gz conda packages
ship .bat-only activation, so GZ_CONFIG_PATH is never set), so on a dev machine
there is no other signal whatsoever.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import pytest
import xacro
from shared.config.constants import RobotSpecs

if TYPE_CHECKING:
    from xml.dom.minidom import Element

_DESCRIPTION = (
    Path(__file__).resolve().parents[4] / "apps" / "gazebo" / "runtime" / "robot_description" / "wro_robot.urdf.xacro"
)


@pytest.fixture(scope="module")
def joints() -> dict[str, Element]:
    """Every joint in the expanded URDF, by name.

    Expanding it at all is assertion 1 -- this used to raise ExpatError. The
    document xacro hands back is walked directly rather than re-serialized and
    re-parsed, so nothing sits between the check and what xacro produced.
    """
    document = xacro.process_file(str(_DESCRIPTION))
    return {j.getAttribute("name"): j for j in document.getElementsByTagName("joint")}


def _child_attr(joint: Element, tag: str, attribute: str) -> str:
    return joint.getElementsByTagName(tag)[0].getAttribute(attribute)


class TestBothAxlesSteer:
    """The property that makes this chassis different from a textbook car."""

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_the_rear_axle_has_a_steering_joint(self, joints, side):
        joint = joints[f"rear_{side}_steering_joint"]

        assert joint.getAttribute("type") == "revolute", "a rear axle that cannot rotate is front-steer"
        assert _child_attr(joint, "parent", "link") == "base_link"

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_the_rear_wheel_hangs_off_that_steering_joint(self, joints, side):
        """Re-parenting is what actually turns the wheel.

        Adding the hinge but leaving the wheel on base_link would look correct
        in a diff and change nothing in simulation.
        """
        wheel = joints[f"rear_{side}_wheel_joint"]

        assert _child_attr(wheel, "parent", "link") == f"rear_{side}_steering"

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_the_rear_mimics_the_front_in_counter_phase(self, joints, side):
        """Negative multiplier, and its magnitude tracks robot.toml.

        A mimic constraint rather than a controller because the stock gz-sim
        AckermannSteering plugin steers one axle by construction; expressing it
        as a physics constraint also means the axles cannot drift out of phase.
        """
        joint = joints[f"rear_{side}_steering_joint"]
        mimic = joint.getElementsByTagName("mimic")

        assert mimic, "rear steering with nothing driving it would sit at zero forever"
        assert mimic[0].getAttribute("joint") == f"front_{side}_steering_joint", "must follow its OWN side"
        assert float(mimic[0].getAttribute("multiplier")) == pytest.approx(-RobotSpecs.REAR_STEER_RATIO)


class TestGeometryComesFromRobotToml:
    """The URDF and RobotSpecs read the same source; check they still agree."""

    @pytest.mark.parametrize(("side", "y_sign"), [("left", 1.0), ("right", -1.0)])
    @pytest.mark.parametrize(("axle", "x_sign"), [("front", 1.0), ("rear", -1.0)])
    def test_wheels_sit_on_the_measured_axle_geometry(self, joints, side, y_sign, axle, x_sign):
        origin = _child_attr(joints[f"{axle}_{side}_steering_joint"], "origin", "xyz")
        x, y, _z = (float(value) for value in origin.split())

        assert x == pytest.approx(x_sign * RobotSpecs.WHEELBASE / 2)
        assert y == pytest.approx(y_sign * RobotSpecs.TRACK_WIDTH / 2)

    def test_the_steering_limit_is_the_commandable_angle(self, joints):
        limit = joints["front_left_steering_joint"].getElementsByTagName("limit")[0]

        assert float(limit.getAttribute("upper")) == pytest.approx(RobotSpecs.MAX_STEERING_ANGLE)
        assert float(limit.getAttribute("lower")) == pytest.approx(-RobotSpecs.MAX_STEERING_ANGLE)
