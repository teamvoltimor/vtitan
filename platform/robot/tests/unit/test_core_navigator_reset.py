"""CoreNavigator.reset() must clear per-race state, or a re-run started purely
from the physical button (FINISHED -> BOOT_CHECK -> READY -> RACING, no
process restart) inherits the previous race's outcome -- e.g. a stale
laps_completed >= num_laps would make the very first tick of the new race
read as already finished.
"""

from __future__ import annotations

from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.core_navigator import CoreNavigator
from src.navigation.ports import DriveCommand, LidarScan
from tests.fixtures import FakeGateway


def test_reset_clears_lap_and_waypoint_state():
    gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0))
    waypoints = [(5.0, 0.0), (10.0, 0.0)]
    nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=NavigationTuning())

    # Simulate race 1 finishing: no park controller, so a finished lap holds forever.
    nav._laps_completed = 1  # noqa: SLF001 - simulating end-of-race state directly
    nav._waypoint_index = 1  # noqa: SLF001
    nav.step()
    assert gateway.commands[-1] == DriveCommand(speed_mps=0.0, steering_norm=0.0), "should be holding post-race"

    nav.reset()
    assert nav.laps_completed == 0
    assert nav._waypoint_index == 0  # noqa: SLF001

    # Race 2 must actually drive again, not immediately re-hold as finished.
    gateway.commands.clear()
    nav.step()
    assert gateway.commands[-1].speed_mps > 0.0, "reset navigator must drive, not hold as already finished"
