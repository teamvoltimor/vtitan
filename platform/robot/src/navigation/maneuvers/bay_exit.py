"""Back out of the parking pocket at the start of a round.

The WRO rules allow two legal starts: inside the parking lot, or parallel to it
in the same section. Getting OUT of the pocket is a different problem from
getting in -- :mod:`src.navigation.maneuvers.parking` drives the entry, and this
drives the exit.

Lived in ``ScenarioSimulator`` until 2026-08-31, where it published drive
commands straight to the gateway and so bypassed the navigation stack
completely. That made every in-bay simulation result a statement about the
simulator rather than about the robot, and left the real robot with no bay-exit
path at all -- on hardware an in-bay start fell through to
:func:`~src.navigation.corridor_follower.follow_corridor`'s generic back-off
pivot, a different manoeuvre. The tuning constants had always been nav-side
(``corridor_follower.BAY_EXIT_*``), which is where the code was meant to be.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.config.tuning_helpers import get_tuning
from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, clamp

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning

_LEG_STALL_EPSILON_M = 1e-4
"""Wheel travel below which a tick counts as no progress at all.

A tenth of a millimetre: two orders under the 7.5 mm a free tick covers at
creep, so ordinary slow motion never reads as a stall, while a chassis held
against a surface -- which reports no travel at all -- registers immediately."""


class BayExit:
    """Drives the reverse-then-swing exit, holding the reverse leg's origin.

    One instance per round. It holds the two things a scan cannot re-derive:
    where the reverse leg started (the pocket looks the same throughout it) and
    which side is open (the rays that answer that stop meaning anything as soon
    as the chassis rotates -- see ``BAY_EXIT_LATCH_DIRECTION``).
    """

    def __init__(self) -> None:
        self._reverse_start_m: float | None = None
        self._reverse_done = False
        self._reverse_ticks = 0
        self._forward_ticks = 0
        self._reverse_progress_m = 0.0
        self._open_is_left: bool | None = None
        self._open_flips = 0
        # Cycle manoeuvre state. Starts on the FORWARD leg: the steered wheels
        # are at the front, so a forward move is the one that rotates the nose
        # out, and the reverse exists only to buy back the room it spends.
        self._leg_is_reverse = False
        self._leg_start_m: float | None = None
        self._last_travelled_m = 0.0
        self._leg_stall_ticks = 0
        self._cycles = 0
        # Ticks of standstill still owed to the servo before this leg may move.
        # Starts at 0: the first arc begins from wherever the wheels already
        # are, and the settle is budgeted at each leg CHANGE.
        self._settle_ticks = 0

    @property
    def cycles(self) -> int:
        """Completed forward-then-reverse cycles, for diagnostics."""
        return self._cycles

    @property
    def legs(self) -> tuple[int, int, float]:
        """``(reverse_ticks, forward_ticks, reverse_progress_m)``, for diagnostics.

        The reverse gate reads SIGNED odometry, which cancels under rocking: a
        chassis pushed forward as much as it backs registers no progress and
        the manoeuvre never advances to the turn. Distance travelled cannot
        show that -- the path length accumulates either way -- so which leg the
        ticks were spent in has to be counted rather than inferred.
        """
        return self._reverse_ticks, self._forward_ticks, self._reverse_progress_m

    @property
    def open_flips(self) -> int:
        """Times the measured open side changed sides during the manoeuvre."""
        return self._open_flips

    def _begin_leg(
        self,
        *,
        is_reverse: bool,
        travelled_m: float,
        tuning: NavigationTuning,
        from_norm: float,
    ) -> None:
        """Switch legs and budget the standstill needed to reach the new angle.

        The pause is COMPUTED, not tuned: the servo covers
        ``MAX_STEERING_RATE / CONTROL_HZ`` radians per tick, and the swing is
        the difference between the two legs' angles, so the tick count follows
        from the geometry and moves correctly if either constant changes. A
        hand-set constant here would silently under-budget the moment somebody
        widened the arc.
        """
        follower = tuning.corridor_follower
        arc = clamp(follower.BAY_EXIT_ARC_STEER_NORM, 0.0, 1.0)
        back = clamp(follower.BAY_EXIT_CYCLE_REVERSE_STEER_NORM, 0.0, 1.0)
        sign = 1.0 if self._open_is_left else -1.0
        to_norm = -back * sign if is_reverse else arc * sign
        swing_rad = abs(to_norm - from_norm) * math.radians(RobotSpecs.MAX_WHEEL_ANGLE_DEG)
        per_tick_rad = tuning.pursuit.MAX_STEERING_RATE / tuning.control.CONTROL_HZ
        self._settle_ticks = math.ceil(swing_rad / per_tick_rad) if per_tick_rad > 0 else 0
        self._leg_is_reverse = is_reverse
        self._leg_start_m = travelled_m
        self._leg_stall_ticks = 0
        # The standstill would otherwise read as a stall on its very first
        # moving tick, since travel during it is zero by construction.
        self._last_travelled_m = travelled_m

    def _cycle_command(
        self,
        travelled_m: float,
        creep_speed_mps: float,
        tuning: NavigationTuning,
        open_is_left: bool,
    ) -> DriveCommand:
        """Alternate a steered forward arc with a straight reverse, until clear.

        A shuffle at CONSTANT steering magnitude provably cannot accumulate:
        ``dy/dtheta = sin(theta) / (k tan(delta))`` is independent of speed and
        of its sign, so ``y`` is a state function of ``theta`` and any cycle
        that returns theta to its start returns y with it. That is why the
        previous manoeuvre could only escape by leaning on wall contact, whose
        turn-clipping is what broke the conservation -- and why its result did
        not survive a change to the contact model.

        Asymmetric legs break it honestly instead. The forward leg arcs at
        ``BAY_EXIT_ARC_STEER_NORM`` toward the open side; the reverse backs
        STRAIGHT. A straight reverse returns no rotation at all, so the theta
        the arc won is kept while the room it spent is bought back, and each
        cycle nets outward displacement in free space -- no contact required.

        Moderate steering, not full lock, for the same reason the pocket needs:
        at 85 deg the turn radius is 17 mm and the chassis pivots about itself,
        translating nothing. Around 45 deg it is ~0.19 m, which actually moves
        the body sideways.

        Legs are latched, and that is not incidental. The previous gate compared
        ``reverse_start - travelled`` against a threshold that the FORWARD leg
        drives back down, so it flapped between two opposed commands ~600 times
        a run. Transitions here are one-way within a cycle: forward until the
        way ahead closes, reverse a bounded distance, repeat.
        """
        follower = tuning.corridor_follower
        arc = clamp(follower.BAY_EXIT_ARC_STEER_NORM, 0.0, 1.0)
        back = clamp(follower.BAY_EXIT_CYCLE_REVERSE_STEER_NORM, 0.0, 1.0)
        sign = 1.0 if open_is_left else -1.0
        target = -back * sign if self._leg_is_reverse else arc * sign

        # Steer FIRST, then drive. Every previous version commanded the angle and
        # the motion together, so the servo slewed while the leg ran and the leg
        # ended before the angle arrived: a 0.05 m leg is ~11 ticks at creep,
        # while 42 deg of slew is ~12 and 55 deg ~16. That is why the arc sweep
        # ordered backwards -- the SMALLEST angle travelled furthest, being the
        # only one reachable. Slewing at a standstill costs ticks but no travel,
        # and travel is the only thing a 7.5 cm pocket is short of.
        if self._settle_ticks > 0:
            self._settle_ticks -= 1
            return DriveCommand(speed_mps=0.0, steering_norm=target)

        # Stall is the primary leg-end signal, not distance. Wheel odometry
        # stops accumulating exactly when the chassis is blocked, so any leg
        # bounded only by distance runs FOREVER once it jams -- measured on the
        # first version of this manoeuvre: the reverse leg backed 6.5 cm onto
        # the rear fin and then held there for 174 ticks, because the 0.05 m it
        # was waiting for could no longer arrive. The previous manoeuvre failed
        # the same way from the other side. Counted only on ticks that COMMAND
        # motion -- the settle above returns first, so a deliberate standstill
        # is never mistaken for a jam.
        if abs(travelled_m - self._last_travelled_m) < _LEG_STALL_EPSILON_M:
            self._leg_stall_ticks += 1
        else:
            self._leg_stall_ticks = 0
        self._last_travelled_m = travelled_m
        stalled = self._leg_stall_ticks >= follower.BAY_EXIT_LEG_STALL_TICKS

        if self._leg_is_reverse:
            self._reverse_ticks += 1
            self._reverse_progress_m = (self._leg_start_m or travelled_m) - travelled_m
            if stalled or self._reverse_progress_m >= follower.BAY_EXIT_REVERSE_M:
                self._begin_leg(is_reverse=False, travelled_m=travelled_m, tuning=tuning, from_norm=target)
                self._cycles += 1
            # Straight back at 0 (the reverse then returns no rotation, so the
            # arc's gain is kept), or OPPOSITE lock, which is the classic
            # three-point turn: backing with the wheels the other way swings the
            # tail the other way, so the nose keeps turning the SAME sense on
            # both legs and heading accumulates twice as fast.
            #
            # Opposite lock is not free: it asks the servo for a full swing
            # between legs, 2 x the arc angle, and at MAX_STEERING_RATE that
            # takes ~25 ticks against a leg lasting ~11 at creep. If a sweep of
            # this value comes back flat, that is the slew clipping every
            # setting to the same reachable angle -- the same trap that made
            # BAY_EXIT_STEER_NORM read as inert -- not the idea failing.
            return DriveCommand(speed_mps=-creep_speed_mps * follower.REVERSE_SPEED_SCALE, steering_norm=target)

        self._forward_ticks += 1
        # Bounded by GEOMETRY plus a stall backstop, NOT by forward clearance.
        # Sensing does not work in here: a clean raycast at the bay pose gives
        # 0.215 m, but the live pipeline reads 0.05-0.13 m and fluctuates tick
        # to tick -- the chassis sits 0.1 m from one wall and 3 mm from the
        # other, so a forward-cone minimum is dominated by its surroundings and
        # by noise. Gated on it, the arc got ONE tick per cycle and the chassis
        # turned 0.1 deg in 57 ticks. This is the same reason the reverse leg
        # above is bounded by the lot's own dimensions rather than measured.
        if stalled or travelled_m - (self._leg_start_m or travelled_m) >= follower.BAY_EXIT_FORWARD_M:
            self._begin_leg(is_reverse=True, travelled_m=travelled_m, tuning=tuning, from_norm=target)
        return DriveCommand(speed_mps=creep_speed_mps * follower.CORNER_SPEED_SCALE, steering_norm=target)

    @staticmethod
    def is_clear(
        ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning | None = None
    ) -> bool:
        """Whether the chassis is out of the pocket and normal driving can resume.

        The caller must not simply return on this: the direction-settle block is
        what rebuilds the path for the committed direction and calls
        ``replace_path``, and skipping it hands the planner a stale plan still
        pointing at waypoint 0 while the robot has driven out of the bay.
        Measured: it drove straight back into a marker, 0.24-0.30 m every run.
        """
        tuning = get_tuning(tuning)
        return _forward_clearance(ranges_m, angles_rad, tuning) >= tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M

    def command(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        travelled_m: float,
        creep_speed_mps: float,
        tuning: NavigationTuning | None = None,
    ) -> DriveCommand:
        """Back out of the parking pocket, then swing the nose to the open side.

        Pivoting straight from a centred placement does not work: the pocket is
        0.45 m along the wall against a 0.30 m chassis, so there is only ~7.5 cm
        of slack at each end, and the nose reaches the marker before it has
        rotated clear. Measured -- the pivot alone escaped some scenarios and
        clipped a fin in most.

        So reverse first, to double the room ahead, then turn hard. The reverse
        was straight until 2026-09-02, on the reasoning that a steered reverse
        sweeps the tail across the pocket it is trying to leave. True as far as
        it goes, and outweighed: centring the wheels every reverse throws away
        the servo's slew and the turn never reaches its angle at all. See
        ``BAY_EXIT_HOLD_STEER``, which measured 0 -> 187 of 256 on its own.

        The reverse is bounded by GEOMETRY, not measured: the bound is the slack
        the lot is guaranteed to have by its own dimensions. That was originally
        forced -- there was no rear sensing on this mount at all. The rear slot
        came back on 2026-08-31 (~40 deg at +/-160..180), so backing until
        something appears IS now available and this bound is a deliberate
        holdover rather than a constraint. Changing it is a behaviour change and
        wants its own measurement; the move out of the simulator deliberately
        changed nothing.

        Which way to turn is not a guess either. The lot is always against the
        OUTER wall, so its opening faces the inner block, and a lap always turns
        toward the inner block -- open side, inner side and corner-turn side are
        the same side by track design. It is the LEFT of a counterclockwise lap
        and the RIGHT of a clockwise one, verified 64/64 against geometry in
        both directions. Being a fact about the layout, it is read once and
        latched rather than re-derived every tick: 0 -> 254 of 256 together with
        the steering hold, 187 -> 254 on top of it. See
        ``BAY_EXIT_LATCH_DIRECTION``.

        Args:
            ranges_m: LIDAR ranges.
            angles_rad: Matching robot-frame bearings (0 = forward).
            travelled_m: Signed wheel odometry. A quadrature encoder counts DOWN
                in reverse, so progress on the reverse leg is start-minus-current.
            creep_speed_mps: Blind-phase creep speed to scale both legs from.
            tuning: Navigation tuning, defaulting to the shipped profile.

        Returns:
            The drive command for this tick.
        """
        tuning = get_tuning(tuning)
        follower = tuning.corridor_follower
        if self._reverse_start_m is None:
            self._reverse_start_m = travelled_m

        # Which way is out. Single rays at +/-90 deg, so in the pocket one is the
        # outer wall and the other is open corridor.
        left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
        right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
        open_is_left = left > right
        if follower.BAY_EXIT_LATCH_DIRECTION and self._open_is_left is not None:
            # First reading wins: taken while the chassis is still parallel to
            # the wall, which is the only pose at which these two rays compare
            # wall against corridor at all.
            open_is_left = self._open_is_left
        # Counted, not assumed stable: these are single rays at +/-90 deg, which
        # point ACROSS the pocket only while the chassis is still parallel to the
        # wall. Once it rotates they point along it, at a fin on each side, and a
        # flip inverts the steering sign -- turning the escape into a re-entry.
        if self._open_is_left is not None and open_is_left != self._open_is_left:
            self._open_flips += 1
        self._open_is_left = open_is_left

        if follower.BAY_EXIT_CYCLE:
            return self._cycle_command(travelled_m, creep_speed_mps, tuning, open_is_left)

        # Wheel distance is SIGNED -- comparing current-minus-start gives a
        # negative that is below any positive threshold forever, which reversed
        # until the tail hit the rear fin. Measured before the fix: every
        # BAY_EXIT_STEER_NORM from 0.0 to 1.0 and every BAY_EXIT_REVERSE_M from
        # 0.001 to 0.20 produced byte-identical runs, because the turn was
        # unreachable in all of them.
        self._reverse_progress_m = self._reverse_start_m - travelled_m
        if follower.BAY_EXIT_LATCH_REVERSE and self._reverse_progress_m >= follower.BAY_EXIT_REVERSE_M:
            # One-shot once latching is on. The forward leg drives this same
            # quantity back DOWN -- it is start-minus-travelled, and travelling
            # forward raises travelled -- so without the latch the gate returns
            # to reverse on the very next tick and the manoeuvre chatters
            # between two opposed commands instead of holding the turn.
            self._reverse_done = True
        if not self._reverse_done and self._reverse_progress_m < follower.BAY_EXIT_REVERSE_M:
            self._reverse_ticks += 1
            # Steering is INVERTED on the reverse, the same way
            # follow_corridor's reverse branch inverts it: backing up swings the
            # nose away from the steer direction, so steering toward the WALL
            # walks the nose out toward the open corridor. That buys lateral
            # offset with no forward travel, which is the only thing the pocket
            # has no room for.
            reverse_steer = clamp(follower.BAY_EXIT_REVERSE_STEER_NORM, 0.0, 1.0)
            reverse_norm = -reverse_steer if open_is_left else reverse_steer
            if follower.BAY_EXIT_HOLD_STEER:
                # Hold the FORWARD leg's angle instead of returning to centre.
                #
                # The servo slews at MAX_STEERING_RATE = 1.2 rad/s, so reaching
                # full lock takes 1.24 s = 25 ticks, while a stroke bounded by
                # BAY_EXIT_REVERSE_M = 0.05 m lasts about 9. Commanding 0 here
                # slews the wheels back to centre every cycle, so the angle is
                # never reached: measured 30.9 deg of the 85 deg asked for, 36%
                # of full lock, ramping up for 9 ticks and straight back down
                # for 11. That also explains why BAY_EXIT_STEER_NORM swept
                # byte-identical at 0.4/0.6/0.8/1.0 -- every command at or above
                # 0.364 is clipped to the same achievable angle, and only 0.2
                # (17 deg, reachable inside a stroke) behaved differently.
                #
                # Lengthening the stroke instead is not available: the servo
                # needs ~0.14 m of travel and the pocket has 7.5 cm of slack.
                # Holding costs no travel at all.
                #
                # NOT the same as BAY_EXIT_REVERSE_STEER_NORM, which applies the
                # INVERTED sign and so slews even further, to opposite lock --
                # refuted 2026-08-29, every non-zero value collapsing to 0.02 m.
                reverse_norm = clamp(follower.BAY_EXIT_STEER_NORM, 0.0, 1.0) * (1.0 if open_is_left else -1.0)
            return DriveCommand(
                speed_mps=-creep_speed_mps * follower.REVERSE_SPEED_SCALE,
                steering_norm=reverse_norm,
            )

        # Magnitude is tuned, not pinned at full lock -- see BAY_EXIT_STEER_NORM.
        # Full lock spins the chassis about its own centre (8 mm radius at the
        # shipped 85 deg wheel angle) and the pocket has no room to rotate in;
        # what gets the robot out is translation.
        self._forward_ticks += 1
        magnitude = clamp(follower.BAY_EXIT_STEER_NORM, 0.0, 1.0)
        return DriveCommand(
            speed_mps=creep_speed_mps * follower.CORNER_SPEED_SCALE,
            steering_norm=magnitude if open_is_left else -magnitude,
        )
