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

from shared.config.constants import ParkingLotSpecs, RobotSpecs

from src.config.tuning_helpers import get_tuning
from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, clamp

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning

_EFFECTIVE_WHEELBASE_M = RobotSpecs.WHEELBASE / (1.0 + RobotSpecs.REAR_STEER_RATIO)
"""Wheelbase the chassis actually turns about, not the axle spacing.

The rear axle steers counter-phase, so the instantaneous centre sits between
the axles: ``wheelbase / (1 + rear_steer_ratio)``, which at the shipped ratio of
1.0 is HALF the wheelbase. Paired with ``YAW_GAIN``, the fraction of the
geometric yaw rate the real chassis achieves (0.55, calibrated against bag data
2026-08-29). A plain bicycle model understates the yaw by ~10% here, and the
clearance guard integrates that error over every tick it dead-reckons.
"""

_LEG_STALL_EPSILON_M = 1e-4
"""Wheel travel below which a tick counts as no progress at all.

A tenth of a millimetre: two orders under the 7.5 mm a free tick covers at
creep, so ordinary slow motion never reads as a stall, while a chassis held
against a surface -- which reports no travel at all -- registers immediately."""


def _rect_corners(along: float, out: float, yaw: float, length: float, width: float) -> list[tuple[float, float]]:
    """Corners of a ``length`` x ``width`` rectangle centred at (along, out), rotated by ``yaw``."""
    ca, sa = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    return [
        (along + sl * hl * ca - sw * hw * sa, out + sl * hl * sa + sw * hw * ca)
        for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    ]


def _gap(poly_a: list[tuple[float, float]], poly_b: list[tuple[float, float]]) -> float:
    """Separating-axis gap between two convex polygons; negative means overlap.

    Under-estimates vertex-to-vertex gaps, which is the safe direction for a
    guard whose job is to never touch.
    """
    best = -math.inf
    for poly in (poly_a, poly_b):
        count = len(poly)
        for i in range(count):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % count]
            ax, ay = y2 - y1, x1 - x2
            norm = math.hypot(ax, ay)
            if norm == 0.0:
                continue
            ax, ay = ax / norm, ay / norm
            a_lo = min(px * ax + py * ay for px, py in poly_a)
            a_hi = max(px * ax + py * ay for px, py in poly_a)
            b_lo = min(px * ax + py * ay for px, py in poly_b)
            b_hi = max(px * ax + py * ay for px, py in poly_b)
            best = max(best, b_lo - a_hi, a_lo - b_hi)
    return best


def _fin_rects() -> list[list[tuple[float, float]]]:
    """The two marker fins in the BAY FRAME, from the rulebook geometry.

    Origin is where the chassis was placed -- the pocket centre, on the lot's
    axis, ``ParkingLotSpecs.WALL_OFFSET`` out from the wall. ``along`` runs down
    the wall along the start heading; ``out`` runs away from the wall toward the
    corridor. The fins stand at +-half the block spacing, are ``WIDTH`` thick,
    and span the lot's full depth from the wall to their tips.

    Known without sensing: these are fixed by the rules, and the manoeuvre is
    only ever entered from a placement the judges made. The pocket cannot be
    measured from inside it -- a forward cone reads 0.05-0.13 m and fluctuates
    every tick -- so a guard that needed to see the fins could not work.
    """
    half_spacing = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH / 2.0
    inner = half_spacing - ParkingLotSpecs.WIDTH / 2.0
    outer = half_spacing + ParkingLotSpecs.WIDTH / 2.0
    wall = -ParkingLotSpecs.WALL_OFFSET
    tip = wall + ParkingLotSpecs.LENGTH
    return [
        [(-outer, wall), (-inner, wall), (-inner, tip), (-outer, tip)],
        [(inner, wall), (outer, wall), (outer, tip), (inner, tip)],
    ]


def _wall_feasible_yaw_rad(out_m: float) -> float:
    """Greatest yaw the pocket's DEPTH allows at this outward displacement.

    The lot is 0.20 m deep against a 0.194 m chassis, so the wall behind pins
    rotation until the body has eased out of the pocket: the swept depth
    ``(L sin t + W cos t) / 2`` has to clear ``out + WALL_OFFSET``. Written as
    ``hypot(L, W) sin(t + atan2(W, L))`` that inverts in closed form. It is
    1.15 degrees at the judges' placement, 3.11 at 5 mm out, 9.31 at 20 mm,
    and unbounded past 78.6 mm -- so the escape is a RATCHET, each shuffle
    buying the yaw that buys the next shuffle. Coupling
    ``d(out)/d(along) = tan t`` grows exponentially with a 0.15 m length
    scale, which reaches free rotation in about 0.50 m of shuffling.

    The FIRST crossing is the bound, not the largest feasible angle. Past the
    peak at ``atan2(W, L)`` the swept depth falls again, so wide angles are
    feasible too -- but a chassis rotating continuously from parallel cannot
    jump the infeasible band between them.
    """
    reach = out_m + ParkingLotSpecs.WALL_OFFSET
    diagonal = math.hypot(RobotSpecs.LENGTH, RobotSpecs.WIDTH)
    sin_sum = 2.0 * reach / diagonal
    if sin_sum >= 1.0:
        return math.pi / 2.0
    return max(0.0, math.asin(sin_sum) - math.atan2(RobotSpecs.WIDTH, RobotSpecs.LENGTH))


def _leg_speed(
    creep_speed_mps: float, follower: object, *, reverse: bool, exit_scale: bool = True
) -> float:
    """Speed for one bay-exit leg, as a POSITIVE magnitude.

    The manoeuvre inherits the driving ladder's creep speed and scales it down
    twice, which lands at 0.067 m/s -- and the drivetrain does not deliver that.
    Measured on run_20260906_181613/_181839: the navigator commanded 0.067 m/s
    on 876 of 882 ticks, never pausing more than 0.1 s, while /motor/drive_speed
    read 0 deg/s on 92-97% of them against the ~110 deg/s that speed implies on
    a 7 cm wheel. The chassis was not waiting; it was being asked for a speed
    below the motor's usable range, and it lurched only when a leg happened to
    break static friction.

    ``BAY_EXIT_SPEED_MPS`` overrides the whole chain with an ABSOLUTE value,
    because what this manoeuvre needs is set by torque against static friction
    at full lock, not by any relationship to cruising speed. 0 keeps the
    inherited scaling.

    The tension is real and is not resolved by picking a big number: the leg has
    to STOP inside the pocket, the drivetrain coasts v * SPEED_RESPONSE_TAU_S,
    and the fin guard refuses any leg it cannot stop in time. In the simulator
    -- which has no deadband and so moves at any commanded speed -- 0.086 m/s
    already collides in 32/32 scenarios. The working window may be narrow, and
    only the robot can say where it is.
    """
    absolute = follower.BAY_EXIT_SPEED_MPS
    if absolute > 0.0:
        # Absolute means absolute: the point is to ask for a speed the
        # drivetrain delivers, and re-scaling it would put it back under the
        # deadband this exists to clear.
        return absolute
    scale = follower.REVERSE_SPEED_SCALE if reverse else follower.CORNER_SPEED_SCALE
    # `exit_scale` is False on the legacy pre-guard paths, which never applied
    # BAY_EXIT_SPEED_SCALE; keeping that lets the override reach them without
    # changing what they do when it is unset.
    return creep_speed_mps * scale * (follower.BAY_EXIT_SPEED_SCALE if exit_scale else 1.0)


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
        # Ticks the manoeuvre has run, and whether the fallback has fired.
        self._ticks = 0
        self._switched = False
        # Dead-reckoned pose in the BAY FRAME, for the clearance guard. Signed so
        # +out is the OPEN side and +yaw turns toward it, which makes the fin
        # geometry symmetric and removes the left/right case split.
        self._dr_along = 0.0
        self._dr_out = 0.0
        self._dr_yaw = 0.0
        self._dr_prev_m: float | None = None
        self._dr_wheel_rad = 0.0
        self._guard_flips = 0
        self._guard_min_gap: float | None = None
        # Straight-reverse recovery from wall contact. Ticks still owed, and how
        # many times it has fired -- the count is the diagnostic: a manoeuvre
        # recovering repeatedly is one whose legs keep driving it back into the
        # wall, which is a different problem from touching it once.
        self._recovery_ticks_left = 0
        self._contact_recoveries = 0
        # Rotation achieved since placement, from MEASURED yaw. Accumulated
        # rather than differenced so a wrap at +/-pi does not read as a 360 deg
        # jump, and kept separate from `_dr_yaw` (which is dead-reckoned for the
        # fin guard and understates the real rotation by ~10%).
        self._start_yaw_rad: float | None = None
        self._prev_yaw_rad: float | None = None
        self._rotation_rad = 0.0

    @property
    def contact_recoveries(self) -> int:
        """How many times the nose-against-wall reverse has fired this exit."""
        return self._contact_recoveries

    @property
    def rotation_deg(self) -> float:
        """Rotation from the placement heading, in degrees, from measured yaw."""
        return math.degrees(abs(self._rotation_rad))

    def rotation_complete(self, tuning: NavigationTuning | None = None) -> bool:
        """Whether the chassis has turned far enough to leave the pocket.

        The completion test the manoeuvre actually needs. ``is_clear`` asks
        whether the way ahead is open, which is unanswerable in the pocket --
        the wall is inside ``MIN_VALID_RANGE_M`` and the forward arc reports
        nothing at all. Rotation is measurable throughout, so it is what says
        the exit has done its job.

        Returns False while no yaw has been supplied, which keeps the manoeuvre
        on its previous behaviour rather than releasing on an unmeasured claim.
        """
        if self._start_yaw_rad is None:
            return False
        tuning = get_tuning(tuning)
        return self.rotation_deg >= tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG

    def _track_rotation(self, yaw_rad: float | None) -> None:
        """Accumulate rotation since placement, unwrapping at +/-pi."""
        if yaw_rad is None:
            return
        if self._start_yaw_rad is None:
            self._start_yaw_rad = yaw_rad
            self._prev_yaw_rad = yaw_rad
            return
        previous = self._prev_yaw_rad
        if previous is not None:
            step = yaw_rad - previous
            if step > math.pi:
                step -= 2.0 * math.pi
            elif step < -math.pi:
                step += 2.0 * math.pi
            self._rotation_rad += step
        self._prev_yaw_rad = yaw_rad

    @property
    def guard_stats(self) -> tuple[int, float | None, float]:
        """``(direction_flips, min_predicted_gap_m, outward_travel_m)`` for the guard."""
        return self._guard_flips, self._guard_min_gap, self._dr_out

    def _dead_reckon(self, travelled_m: float, wheel_norm: float, tuning: NavigationTuning) -> None:
        """Advance the bay-frame pose from wheel odometry and the commanded steering.

        A bicycle model driven by the two things the robot genuinely has in the
        pocket: how far the wheels turned, and what angle it asked the servo
        for. No LIDAR, because the pocket cannot be sensed from inside it, and
        no pose estimate, because the localizer is matching a wall model the
        chassis is not yet out among.

        The servo's SLEW is modelled rather than assumed instant. Skipping it
        was what made ``BAY_EXIT_STEER_NORM`` read as inert: every command at or
        above 0.364 clipped to the same reachable angle, because a 0.05 m stroke
        is ~9 ticks and full lock takes 25.
        """
        previous = self._dr_prev_m
        self._dr_prev_m = travelled_m
        if previous is None:
            return
        step = travelled_m - previous
        max_rad = math.radians(RobotSpecs.MAX_WHEEL_ANGLE_DEG)
        target = clamp(wheel_norm, -1.0, 1.0) * max_rad
        slew = tuning.pursuit.MAX_STEERING_RATE / tuning.control.CONTROL_HZ
        self._dr_wheel_rad += clamp(target - self._dr_wheel_rad, -slew, slew)
        self._dr_yaw += step * math.tan(self._dr_wheel_rad) / _EFFECTIVE_WHEELBASE_M * RobotSpecs.YAW_GAIN
        # The wall behind the pocket CLIPS the rotation, and dead reckoning
        # cannot see it -- measured 4.5x high. Unclamped, the guard bounds a
        # pose the chassis can never reach: on the first arc it predicts ~19 deg
        # where 1.15 is available, takes the swept extent of that fantasy, finds
        # it inside a fin and ends the leg -- every tick, so the manoeuvre never
        # moves. Measured 2026-09-04 over 16 corpus scenarios: total travel
        # 0.05-0.06 m whether the arc was 0.02 or 0.3, i.e. the arc was INERT
        # because the guard rejected the leg before its value could matter.
        limit = _wall_feasible_yaw_rad(self._dr_out)
        self._dr_yaw = clamp(self._dr_yaw, -limit, limit)
        self._dr_along += step * math.cos(self._dr_yaw)
        self._dr_out += step * math.sin(self._dr_yaw)

    def _predicted_gap(self, step_m: float, wheel_norm: float, tuning: NavigationTuning) -> float:
        """Fin clearance the chassis WOULD have after one more step like this one."""
        max_rad = math.radians(RobotSpecs.MAX_WHEEL_ANGLE_DEG)
        target = clamp(wheel_norm, -1.0, 1.0) * max_rad
        slew = tuning.pursuit.MAX_STEERING_RATE / tuning.control.CONTROL_HZ
        wheel = self._dr_wheel_rad + clamp(target - self._dr_wheel_rad, -slew, slew)
        yaw = self._dr_yaw + step_m * math.tan(wheel) / _EFFECTIVE_WHEELBASE_M * RobotSpecs.YAW_GAIN
        # Same wall clip as `_dead_reckon`, for the same reason: a predicted
        # pose the pocket forbids is not a prediction the guard may act on.
        limit = _wall_feasible_yaw_rad(self._dr_out)
        yaw = clamp(yaw, -limit, limit)
        along = self._dr_along + step_m * math.cos(yaw)
        out = self._dr_out + step_m * math.sin(yaw)
        corners = _rect_corners(along, out, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
        return min(_gap(corners, fin) for fin in _fin_rects())

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

    def _reset_for_switch(self, travelled_m: float) -> None:
        """Re-origin both manoeuvres' odometry state at the handover.

        Every distance in here is measured from a remembered starting odometry
        reading, and those readings belong to the manoeuvre that just gave up.
        Carried across, the incoming reverse leg would believe it had already
        run -- ``_reverse_start_m`` is set on the first tick of the round, so by
        the switch it is hundreds of ticks stale.
        """
        self._reverse_start_m = travelled_m
        self._reverse_done = False
        self._leg_is_reverse = False
        self._leg_start_m = travelled_m
        self._last_travelled_m = travelled_m
        self._leg_stall_ticks = 0
        self._settle_ticks = 0

    def _begin_leg(
        self,
        *,
        is_reverse: bool,
        travelled_m: float,
        tuning: NavigationTuning,
        from_norm: float,
        to_norm: float,
    ) -> None:
        """Switch legs and budget the standstill needed to reach the new angle.

        The pause is COMPUTED, not tuned: the servo covers
        ``MAX_STEERING_RATE / CONTROL_HZ`` radians per tick, and the swing is
        the difference between the two legs' angles, so the tick count follows
        from the geometry and moves correctly if either constant changes. A
        hand-set constant here would silently under-budget the moment somebody
        widened the arc.

        Both angles are passed IN rather than re-derived here, because the two
        manoeuvres disagree about what a reverse leg steers: ``_cycle_command``
        takes the opposite lock, the wall ratchet in ``_guarded_command`` HOLDS
        the forward one. A single convention baked in here would hand the other
        manoeuvre the wrong swing, and so the wrong standstill.
        """
        swing_rad = abs(to_norm - from_norm) * math.radians(RobotSpecs.MAX_WHEEL_ANGLE_DEG)
        per_tick_rad = tuning.pursuit.MAX_STEERING_RATE / tuning.control.CONTROL_HZ
        self._settle_ticks = math.ceil(swing_rad / per_tick_rad) if per_tick_rad > 0 else 0
        self._leg_is_reverse = is_reverse
        self._leg_start_m = travelled_m
        self._leg_stall_ticks = 0
        # The standstill would otherwise read as a stall on its very first
        # moving tick, since travel during it is zero by construction.
        self._last_travelled_m = travelled_m

    def _guarded_command(
        self,
        travelled_m: float,
        creep_speed_mps: float,
        tuning: NavigationTuning,
        open_is_left: bool,
    ) -> DriveCommand:
        """Ratchet out of the pocket against the OUTER WALL, never touching a fin.

        The legal replacement for the stall-bounded cycle. Same shape -- steer
        toward the open side, alternate forward and reverse -- but the leg ends
        when the NEXT pose would come within
        ``BAY_EXIT_CLEARANCE_MARGIN_M`` of a fin, which is a prediction rather
        than a collision. 9.24.7 ends the round on the touch the old backstop
        waited for. The wall behind the pocket is a different matter: 9.18
        expressly permits touching a wall the vehicle does not move, and leaning
        on it is not incidental here but the entire mechanism.

        Why bounding the DISTANCE instead cannot work: the swept extent along
        the wall is ``(L cos t + W sin t) / 2`` -- 0.150 m square, 0.177 m at 25
        degrees. Against 0.215 m to a fin face, yaw alone consumes most of the
        slack before any leg bound applies. (``BAY_EXIT_FORWARD_M`` and
        ``BAY_EXIT_CYCLE_REVERSE_M`` sweeping byte-identical was long read as
        confirming this. It was not: their bound was DEAD -- see the falsy-zero
        note in ``_cycle_command``. A flat sweep meant an unreachable code path,
        not a refuted idea.)

        FREE SPACE CANNOT DO IT, and that is arithmetic rather than tuning. At
        constant steering magnitude the shuffle is a closed cycle:
        ``dy/dtheta = L sin(theta) / tan(delta)`` depends on neither speed nor
        its sign, so ``y`` is a state function of ``theta`` and returning theta
        returns y with it. Varying the magnitude BETWEEN legs does break that --
        the net is ``L (1 - cos a) (1/tan(d_fwd) - 1/tan(d_rev))`` -- but it
        buys outward displacement at a fixed exchange rate of ``a / 2`` per
        metre travelled ALONG the wall, in the same direction every cycle. The
        pocket grants ~0.032 m of along-wall slack each way against
        ``a <= 1.15 deg`` at the judges' placement, which is ~0.7 mm of the
        78.6 mm that frees the rotation. No leg schedule closes that gap.

        The wall does. It CLIPS the yaw (``_wall_feasible_yaw_rad``) while
        leaving the translation free, which is exactly what ``allowed_step``
        does against a solid surface, and a clipped rotation is precisely the
        non-holonomic constraint the conservation argument assumes away. So both
        legs run pinned to the clip and both PAY OUT: the forward leg sits at
        ``+theta_max`` and gains ``ds sin(theta_max)`` outward, the reverse leg
        settles at ``-theta_max`` and gains ``|ds| sin(theta_max)`` outward
        again, while the along-wall excursion cancels between them. The yaw won
        relaxes the clip for the next cycle -- ``d(out)/d(travel) =
        tan(theta_max(out))``, exponential with a 0.15 m length scale, free
        rotation after roughly half a metre of shuffling.

        Reaching ``-theta_max`` is why the reverse HOLDS the forward lock rather
        than reversing it. Backing with the wheels where they are swings the
        nose the other way, so yaw crosses zero and pins against the far side of
        the clip. Opposite lock -- what this did until 2026-09-04 -- drives yaw
        the SAME sense on both legs, so it saturates at one side of the clip and
        never reaches the side the reverse leg's gain lives on. It also charges
        a full servo swing at every leg change, ~25 ticks of standstill against
        an ~11-tick leg, so most of the manoeuvre was spent stationary. Holding
        costs nothing, for the same reason ``BAY_EXIT_HOLD_STEER`` was worth
        0 -> 187 of 256 on the older exit.

        The crossing between the two sides of the clip is DEAD DISTANCE -- the
        outward gain over it cancels by symmetry -- so what the steering angle
        buys is how cheaply the yaw gets across, which is the reverse of what
        this manoeuvre wanted when it arced through free space. It costs
        ``2 theta_max L / (tan(delta) YAW_GAIN)``: 14.5 mm at
        ``BAY_EXIT_ARC_STEER_NORM`` 0.3, 0.6 mm at full lock, against a leg the
        fin clearance bounds to 12-20 mm. At 0.3 the crossing IS the leg and
        nothing is ever pinned, which is why the ratchet measured 8.09 m of
        shuffling for 0.03 m of outward travel there and escaped in 127 ticks at
        1.0. Both bay-exit constants moved on 2026-09-04 for this reason; see
        their fields for the numbers.
        """
        follower = tuning.corridor_follower
        margin = follower.BAY_EXIT_CLEARANCE_MARGIN_M
        sign = 1.0 if open_is_left else -1.0
        arc = clamp(follower.BAY_EXIT_ARC_STEER_NORM, 0.0, 1.0)
        # ONE angle for both legs -- see the docstring; the reverse holds it
        # rather than mirroring it. Signed in the dead-reckoned frame, where
        # +yaw is toward the open side, so the guard's geometry needs no
        # left/right case split; the caller-facing command is re-signed on the
        # way out. ``BAY_EXIT_CYCLE_REVERSE_STEER_NORM`` deliberately does not
        # appear here: it belongs to ``_cycle_command``, whose reverse leg is a
        # different manoeuvre with a different sign convention.
        wheel_norm = arc

        # Slew at a STANDSTILL, as ``_cycle_command`` does. Holding the lock
        # makes it free -- ``_begin_leg`` budgets zero ticks when both legs ask
        # for the same angle -- and that is the point rather than a reason to
        # delete it: the budget is COMPUTED, so widening the arc or making the
        # reverse angle differ again reinstates the pause automatically instead
        # of silently slewing while the leg runs. Dead-reckoned through the
        # pause too: travel is zero, but the wheel is moving and the model has
        # to follow it there as much as anywhere.
        if self._settle_ticks > 0:
            self._settle_ticks -= 1
            self._dead_reckon(travelled_m, wheel_norm, tuning)
            return DriveCommand(speed_mps=0.0, steering_norm=wheel_norm * sign)

        self._dead_reckon(travelled_m, wheel_norm, tuning)
        if self._guard_min_gap is None:
            self._guard_min_gap = self._predicted_gap(0.0, wheel_norm, tuning)

        speed = _leg_speed(creep_speed_mps, follower, reverse=self._leg_is_reverse)
        step = (-speed if self._leg_is_reverse else speed) / tuning.control.CONTROL_HZ
        # Look a STOPPING DISTANCE ahead, not a single tick. Commanding zero
        # does not stop the chassis -- the drivetrain decays with
        # ``SPEED_RESPONSE_TAU_S``, so it coasts a further ``v * tau``, 40 mm at
        # creep against an along-wall budget of 31-57 mm. A one-tick guard
        # therefore ends the leg with the fin already inside the coast, which is
        # how a manoeuvre that never predicted a touch still measured one.
        # ``BAY_EXIT_SPEED_SCALE`` is the lever on this, and it only became one
        # once the settle above stopped the slew competing with the leg.
        coast_m = speed * RobotSpecs.SPEED_RESPONSE_TAU_S
        reach = step + math.copysign(coast_m, step)
        # The rectangle-against-fin gap at that reachable pose IS the bound. A
        # worst-case along-wall limit used to sit alongside it, taking the swept
        # extent at ``hypot(L, W) / 2`` because dead-reckoned yaw was not
        # trusted. It is redundant, and it is what made the guarded exit
        # immobile: that value is the extent at 32.9 degrees, the angle where it
        # PEAKS, so it left 31 mm of the 65 mm slack usable at EVERY yaw
        # including zero. The distrust also points the other way -- DR yaw runs
        # HIGH, and extent rises with yaw up to the peak, so a gap taken at DR
        # yaw is already the conservative reading.
        gap = self._predicted_gap(reach, wheel_norm, tuning)
        self._guard_min_gap = min(self._guard_min_gap, gap)
        if gap <= margin:
            # End the leg on the PREDICTION -- nothing has been touched -- and
            # pay the servo swing before the next one moves. Flipping the flag
            # inline, as this did until 2026-09-04, skipped ``_begin_leg``
            # entirely: no standstill was budgeted, ``_leg_start_m`` was never
            # re-origined, and the new leg inherited the old leg's lock.
            self._begin_leg(
                is_reverse=not self._leg_is_reverse,
                travelled_m=travelled_m,
                tuning=tuning,
                from_norm=wheel_norm * sign,
                to_norm=wheel_norm * sign,
            )
            self._guard_flips += 1
            self._cycles += 1
            return DriveCommand(
                speed_mps=0.0,
                steering_norm=wheel_norm * sign,
            )

        if self._leg_is_reverse:
            self._reverse_ticks += 1
        else:
            self._forward_ticks += 1
        return DriveCommand(
            speed_mps=-speed if self._leg_is_reverse else speed,
            steering_norm=wheel_norm * sign,
        )

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

        # Origin the FIRST leg too, and compare with `is None` rather than `or`
        # below. BOTH were needed: `_leg_start_m` was only ever set by
        # `_begin_leg`/`_reset_for_switch`, which run on a leg CHANGE, so until
        # the first one both bounds below read
        # ``travelled_m - (None or travelled_m)`` == 0 and could never fire. The
        # opening leg was therefore unbounded by distance and could only end on
        # `stalled` -- that is, on CONTACT with a fin, which ends the round under
        # 9.24.7. It is also why BAY_EXIT_FORWARD_M and BAY_EXIT_CYCLE_REVERSE_M
        # swept byte-identical at 0.02 and 0.04, and why `rev_m` measured 0.041
        # against the 0.09 asked for. And once it IS set, the old
        # ``self._leg_start_m or travelled_m`` idiom still discarded it whenever
        # it was 0.0 -- which is exactly what the first leg of a round starts
        # from, since odometry is zeroed at the start line. A float that can
        # legitimately be zero cannot be defaulted with `or`.
        if self._leg_start_m is None:
            self._leg_start_m = travelled_m

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
            leg_start = travelled_m if self._leg_start_m is None else self._leg_start_m
            self._reverse_progress_m = leg_start - travelled_m
            if stalled or self._reverse_progress_m >= follower.BAY_EXIT_CYCLE_REVERSE_M:
                self._begin_leg(
                    is_reverse=False,
                    travelled_m=travelled_m,
                    tuning=tuning,
                    from_norm=target,
                    to_norm=arc * sign,
                )
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
            return DriveCommand(
                speed_mps=-_leg_speed(creep_speed_mps, follower, reverse=True),
                steering_norm=target,
            )

        self._forward_ticks += 1
        # Bounded by GEOMETRY plus a stall backstop, NOT by forward clearance.
        # Sensing does not work in here: a clean raycast at the bay pose gives
        # 0.215 m, but the live pipeline reads 0.05-0.13 m and fluctuates tick
        # to tick -- the chassis sits 0.1 m from one wall and 3 mm from the
        # other, so a forward-cone minimum is dominated by its surroundings and
        # by noise. Gated on it, the arc got ONE tick per cycle and the chassis
        # turned 0.1 deg in 57 ticks. This is the same reason the reverse leg
        # above is bounded by the lot's own dimensions rather than measured.
        leg_start = travelled_m if self._leg_start_m is None else self._leg_start_m
        if stalled or travelled_m - leg_start >= follower.BAY_EXIT_FORWARD_M:
            self._begin_leg(
                is_reverse=True,
                travelled_m=travelled_m,
                tuning=tuning,
                from_norm=target,
                to_norm=-back * sign,
            )
        return DriveCommand(
            speed_mps=_leg_speed(creep_speed_mps, follower, reverse=False),
            steering_norm=target,
        )

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

        A forward arc with NO valid returns reads as BLOCKED, not clear.
        ``_forward_clearance`` reports ``inf`` when nothing in the arc survives
        ``MIN_VALID_RANGE_M``, which compares as clear against any threshold --
        and in a parking pocket that is exactly backwards. Measured on
        run_20260906_094342: the forward arc returned nothing but self-detection
        at 0.050-0.052 m for seconds (the nose was inside the sensor's minimum
        range, hard against the wall), then dropped out entirely for one tick.
        That tick ended the manoeuvre mid-reverse with the nose still pointed at
        the wall, which is what the operator watched happen. Same failure class
        as the escape path's whole-cone no-return, fixed there 2026-08-28.

        The caller must bound how long this can hold -- see
        ``BAY_EXIT_MAX_FRAMES``. Refusing to release on no evidence is only safe
        while something else can still time the manoeuvre out.
        """
        tuning = get_tuning(tuning)
        clearance = _forward_clearance(ranges_m, angles_rad, tuning)
        if math.isinf(clearance):
            return False
        return clearance >= tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M

    @staticmethod
    def _nose_in_contact(
        ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning
    ) -> bool:
        """Whether the forward arc says the nose is touching, or too close to see.

        Two readings mean the same thing here and both must count. A forward
        clearance BELOW ``BAY_EXIT_CONTACT_DIST_M`` is a wall within a chassis
        nose of the bumper. NO valid returns at all is the same wall, closer
        still: ``_forward_clearance`` drops everything under ``MIN_VALID_RANGE_M``
        and reports ``inf``, so the arc goes silent exactly when the obstacle is
        most present. Reading that second case as open space is what let a
        manoeuvre finish with the nose buried in the wall.
        """
        clearance = _forward_clearance(ranges_m, angles_rad, tuning)
        if math.isinf(clearance):
            return True
        return clearance < tuning.corridor_follower.BAY_EXIT_CONTACT_DIST_M

    def _resolve_open_side(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        tuning: NavigationTuning,
    ) -> bool:
        """Which way is out, latched once because it is a fact about the layout.

        Single rays at +/-90 deg, so in the pocket one is the outer wall and the
        other is open corridor. They point ACROSS the pocket only while the
        chassis is still parallel to the wall; once it rotates they point along
        it, at a fin on each side, and a flip inverts the steering sign --
        turning the escape into a re-entry. Hence the latch, and hence counting
        the flips rather than assuming stability.
        """
        left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
        right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
        open_is_left = left > right
        if tuning.corridor_follower.BAY_EXIT_LATCH_DIRECTION and self._open_is_left is not None:
            open_is_left = self._open_is_left
        if self._open_is_left is not None and open_is_left != self._open_is_left:
            self._open_flips += 1
        self._open_is_left = open_is_left
        return open_is_left

    def command(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        travelled_m: float,
        creep_speed_mps: float,
        tuning: NavigationTuning | None = None,
        yaw_rad: float | None = None,
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

        open_is_left = self._resolve_open_side(ranges_m, angles_rad, tuning)

        self._ticks += 1
        self._track_rotation(yaw_rad)

        # Nose against the wall is a STATE, not an absence of data, and it is
        # answered before any leg logic: a chassis in contact cannot steer its
        # way out, because the wheels that would turn it are the ones being
        # held. Back straight off first, then let the normal legs resume with
        # room to rotate in.
        #
        # Measured on run_20260906_112613: the forward arc fell to 0.052 m
        # (returns below MIN_VALID_RANGE_M are dropped, so a wall closer than
        # 5 cm reads as NOTHING AT ALL), then to zero valid rays for every
        # remaining tick of the round. Until now that blindness ended the
        # manoeuvre by the back door and normal driving -- which does not know
        # it is in a pocket -- drove FORWARD into the wall at 0.26 m/s.
        #
        # Straight, not steered: a steered reverse sweeps the tail across the
        # pocket, and the point of this leg is to buy room, not heading. The
        # turn that follows is the manoeuvre's own, toward the open side it
        # already identifies correctly.
        if follower.BAY_EXIT_CONTACT_RECOVERY_TICKS > 0 and (
            self._recovery_ticks_left > 0 or self._nose_in_contact(ranges_m, angles_rad, tuning)
        ):
            if self._recovery_ticks_left <= 0:
                self._recovery_ticks_left = follower.BAY_EXIT_CONTACT_RECOVERY_TICKS
                self._contact_recoveries += 1
            self._recovery_ticks_left -= 1
            return DriveCommand(
                speed_mps=-_leg_speed(creep_speed_mps, follower, reverse=True),
                steering_norm=0.0,
            )

        # Turned far enough. Past BAY_EXIT_TARGET_YAW_DEG the chassis lies along
        # the parking walls rather than across them, and every further degree
        # carries the nose back toward the outer wall -- run_20260906_112613
        # rotated well past this and ended up pointing at it. Drive STRAIGHT out
        # and let the caller release; continuing to steer is what over-rotates.
        #
        # AFTER the contact check, not before: a chassis that has turned far
        # enough AND is touching must still back off first. Driving forward out
        # of contact is the exact failure this whole path exists to stop, and
        # ordering these the other way round reintroduced it -- caught by
        # test_contact_recovery_still_wins_over_a_completed_rotation.
        #
        # Rotation alone is NOT enough to drive out on. 70 deg is where the
        # chassis stops lying across the pocket, not where it is guaranteed to
        # be aimed down the corridor -- the placement heading and the lot's
        # geometry decide that. Measured on run_20260906_145909: the exit
        # released and normal driving then took forward clearance from 0.54 m
        # to 0.08 m in three seconds, nose into the outer wall, because nothing
        # asked whether the way out was actually open. Turned AND clear, or
        # keep ratcheting -- the next reverse buys more angle, which is the
        # cheap way to be wrong.
        if self.rotation_complete(tuning) and self.is_clear(ranges_m, angles_rad, tuning):
            return DriveCommand(
                speed_mps=_leg_speed(creep_speed_mps, follower, reverse=False),
                steering_norm=0.0,
            )

        # The clearance guard supersedes both contact-bounded exits, so it is
        # answered before their fallback bookkeeping runs at all.
        if follower.BAY_EXIT_CLEARANCE_GUARD:
            return self._guarded_command(travelled_m, creep_speed_mps, tuning, open_is_left)

        # Which exit is driving. After BAY_EXIT_FALLBACK_FRAMES the OTHER one
        # takes over, once: the two are complementary (each 254/256 under the
        # contact model where the other is 0/256) and which one the real robot
        # needs is unknown, so covering both beats betting on one.
        use_cycle = follower.BAY_EXIT_CYCLE
        if follower.BAY_EXIT_FALLBACK_FRAMES and self._ticks > follower.BAY_EXIT_FALLBACK_FRAMES:
            use_cycle = not use_cycle
            if not self._switched:
                self._switched = True
                self._reset_for_switch(travelled_m)
        if use_cycle:
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
                speed_mps=-_leg_speed(creep_speed_mps, follower, reverse=True, exit_scale=False),
                steering_norm=reverse_norm,
            )

        # Magnitude is tuned, not pinned at full lock -- see BAY_EXIT_STEER_NORM.
        # Full lock spins the chassis about its own centre (8 mm radius at the
        # shipped 85 deg wheel angle) and the pocket has no room to rotate in;
        # what gets the robot out is translation.
        self._forward_ticks += 1
        magnitude = clamp(follower.BAY_EXIT_STEER_NORM, 0.0, 1.0)
        return DriveCommand(
            speed_mps=_leg_speed(creep_speed_mps, follower, reverse=False, exit_scale=False),
            steering_norm=magnitude if open_is_left else -magnitude,
        )
