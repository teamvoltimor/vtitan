"""The ``SignRouter`` class: stateful per-tick sign-avoidance router.

Holds the discovered sign list, engagement/passed bookkeeping, and the
per-tick ``deform_waypoint`` entry point. The pure pieces it calls live in
``config``, ``routing`` and ``deformation``; this module owns only state and
orchestration.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from shared.config.navigation_tuning import NavigationTuning, SignDiscoveryParams
from shared.domain.enums import Axis, Direction, Section
from shared.domain.models import SignColor, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.sign_discovery import ObservedSignMap, SignSpec
from src.navigation.planning.sign_slot_map import SlotSignMap
from src.navigation.planning.sign_router.config import SignRouterConfig, SignRouterContext
from src.navigation.planning.sign_router.deformation import apply_deformation, match_detection_to_sign
from src.navigation.planning.sign_router.routing import (
    BEHIND_TOLERANCE,
    ROUTING_TABLE,
    depth_consistent_corridor,
    is_squarely_in_corridor,
    pass_side_lateral_axis,
    satisfiable_corridor,
)
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.utils import _dist2d, wrap_angle

if TYPE_CHECKING:
    from shared.domain.models import TrafficSignObservation

logger = logging.getLogger(__name__)


class SignRouter:
    """Routes the robot past traffic signs using lateral waypoint deformations.

    Usage in navigation tick::

        deformed_wp = router.deform_waypoint(
            waypoint=target_wp,
            robot_pos=(robot_x, robot_y),
            robot_yaw=robot_yaw,
            corridor=current_corridor,
            detections=latest_detections,
        )

    Args:
        signs: Expected sign list from scenario metadata (position + color).
        config: Tuning parameters.
        discovery_config: Tuning parameters for blind sign discovery
            (ObservedSignMap), only used when ``discover=True``.
    """

    def __init__(
        self,
        signs: list[SignSpec],
        config: SignRouterConfig | None = None,
        direction: Direction = Direction.COUNTERCLOCKWISE,
        discover: bool = False,
        discovery_config: SignDiscoveryParams | None = None,
        tuning: NavigationTuning | None = None,
    ) -> None:
        self._signs = list(signs)
        # SignRouterConfig.from_tuning(), not a bare SignRouterConfig(): the
        # dataclass's own field defaults are a second, independent copy of
        # the TOML defaults and can drift from them (commit_hysteresis=True
        # here vs. False in signs/sign_router.toml, until this fix) --
        # `tuning` is already accepted by this constructor, so there's no
        # reason the fallback shouldn't use it too.
        self._config = config or SignRouterConfig.from_tuning(get_tuning(tuning).sign_router)
        self._context = SignRouterContext(tuning)
        # Prefer a corridor whose lane target is actually satisfiable; see
        # _corridor_for_spec. Read here rather than at each call site so the
        # per-tick path stays a plain attribute test.
        self._relabel_unsatisfiable = get_tuning(tuning).sign_router.SIGN_LANE_RELABEL_UNSATISFIABLE
        # Decide a corner sign's face on depth rather than proximity; see
        # _geometric_corridor. Same reason for reading it once here.
        self._depth_consistent_corridor = get_tuning(tuning).sign_router.SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR
        self._direction = direction
        self._passed: set[int] = set()
        self._engaged: set[int] = set()
        self._lap_tick = 0
        # Sign indices passed on the WRONG side of the corridor. The official
        # Obstacles rule is absolute: a RED obstacle must be cleared on its
        # OUTWARD side, a GREEN on its INWARD side. ``_active_sign_candidates``
        # records here, at the instant a sign is retired as passed, whether the
        # robot was on the permitted side — see ``_record_pass_side``. The
        # simulator reads ``wrong_side_violations`` and stops the run, the same
        # way it stops on a forbidden wall contact.
        self._wrong_side: set[int] = set()
        # The sign currently being routed around, kept across ticks so the
        # commanded line does not jump between two legal ones mid-pass. See
        # _prefer_committed.
        self._committed: int | None = None
        # Robot yaw at the tick each sign was first committed to, keyed by sign
        # index. Lets the depth pin (see pin_depth) release on heading drift
        # even when the position-only PIN_CORNER_GUARD still reads squarely in
        # the corridor -- see PIN_HEADING_GUARD.
        self._commit_yaw: dict[int, float] = {}
        # Each sign's own corridor, kept in step with _signs — deform_waypoint()
        # must never apply a sign's (x, y) through a different corridor's axis
        # convention (see _nearest_active_sign). Recomputed per sign rather than
        # once up front, since discovery can both append signs and move an
        # existing one across a corridor boundary as its estimate improves.
        self._sign_corridors = [self._corridor_for_spec(s) for s in self._signs]
        # Per-sign "how many ticks running has the estimate wanted to move to a
        # different corridor", keyed by sign index. See _settled_corridor.
        self._corridor_flip_streak: dict[int, tuple[Section, int]] = {}
        # Discovery mode: the sign layout is randomised every round and no
        # scenario file exists on the mat, so a blind robot has to find the
        # signs with its camera rather than be handed them. See sign_discovery.
        self._sign_map: ObservedSignMap | None
        if discover:
            # Same reason as _config above, and the same bug: a bare
            # SignDiscoveryParams() is a second copy of the TOML defaults that
            # the shipped tree can drift away from, and `tuning` is already
            # accepted here. Passing it on ALSO matters for the fields
            # ObservedSignMap reads straight off the group rather than through
            # a named argument -- SNAP_TO_LATTICE_M among them, which was
            # unreachable from a caller's tuning until this fix.
            discovery_config = discovery_config or get_tuning(tuning).sign_discovery
            if get_tuning(tuning).sign_router.SLOT_SIGN_MAP:
                # A constrained assignment over the 24 legal cells, capped at
                # two per section, instead of free clustering. Same surface --
                # observe/propose/newly_confirmed/published -- so this loop does
                # not change; what changes is that a published position IS a
                # legal cell and a section never publishes a third pillar. See
                # sign_slot_map for the design and the 125-bag measurement.
                self._sign_map = SlotSignMap(self._config.min_confidence, tuning=tuning)
            else:
                self._sign_map = ObservedSignMap(
                    self._config.min_confidence,
                    max_ingest_range_m=discovery_config.MAX_INGEST_RANGE_M,
                    association_dist_m=discovery_config.ASSOCIATION_DIST_M,
                    min_hits=discovery_config.MIN_HITS,
                    robot_corridor_flip_ticks=discovery_config.ROBOT_CORRIDOR_FLIP_TICKS,
                    tuning=tuning,
                )
        else:
            self._sign_map = None

    def adopt_direction(self, direction: Direction) -> None:
        """Re-key the travel-relative pass-side rule once inference settles.

        A blind round builds this router on a PROVISIONAL direction --
        ``Direction.CLOCKWISE`` (``track_navigator_node`` line ~191) -- because
        the real one is not known until the LIDAR settles it seconds later.
        ``_commit_direction`` then rebuilt the path, the lap detector, the width
        estimator and the start measurement, but NOT this router, so
        ``self._direction`` stayed at the placeholder for the whole race.

        The pass-side rule is travel-relative: ``ROUTING_TABLE`` is keyed on
        ``(corridor, direction)`` and every clockwise row is the negation of its
        counterclockwise partner. A stale direction therefore does not degrade
        the lane, it MIRRORS it -- red and green swap sides for every sign.

        Measured on four hardware bags: on the two rounds that inferred
        counterclockwise, the commanded lane matched the clockwise row on 24 of
        28 sign passes, and 22 of the 28 illegal passes are that mirrored
        command -- against 2 caused by phantom signs and 0 by colour errors. The
        one round that inferred CLOCKWISE, and so agreed with the placeholder by
        luck, passed 19 of 26 legally.

        In place rather than by rebuilding through
        ``CoreNavigator.replace_sign_router``, which explicitly drops discovered
        state: that is right between races, and wrong here, where the map
        accumulated during the blind creep is exactly what the round needs and
        is direction-INDEPENDENT anyway.

        What IS direction-derived is cleared: the per-sign corridor labels come
        from ``satisfiable_corridor``, and the commit/engagement bookkeeping and
        the wrong-side verdicts were all recorded under the mirrored rule.
        ``_passed`` is deliberately kept -- a sign already behind the robot is
        behind it whichever way the round turned out to run.
        """
        if direction == self._direction:
            return
        self._direction = direction
        self._sign_corridors = [self._corridor_for_spec(spec) for spec in self._signs]
        self._corridor_flip_streak.clear()
        self._wrong_side.clear()
        self._commit_yaw.clear()
        self._engaged.clear()
        self._committed = None

    @property
    def direction(self) -> Direction:
        """The travel direction this router routes for.

        Exposed so the lane planner uses the SAME direction the routing
        decision was made under. The pass-side rule is travel-relative, so a
        lane built from a second, independently-tracked direction can disagree
        with the routing it is supposed to realise -- and a lane on the wrong
        side is a round-ender under 9.24.5, not a tracking error.
        """
        return self._direction

    @property
    def signs(self) -> list[SignSpec]:
        """Signs currently being routed around — discovered ones included."""
        return list(self._signs)

    @property
    def _lateral_offset(self) -> float:
        """``SignRouterConfig.lateral_offset``, narrowed to the float it always is.

        The field is declared ``float | None`` because None is how a caller asks
        for the computed default, but ``__post_init__`` resolves that default
        before construction returns, so no instance ever carries None. The type
        cannot say so, which left three call sites passing ``float | None`` into
        parameters typed ``float`` -- one of them already carrying its own local
        assert. Stating the invariant once beats restating it per call site.
        """
        offset = self._config.lateral_offset
        assert offset is not None, "SignRouterConfig.__post_init__ resolves lateral_offset"
        return offset

    def _ingest_observations(
        self,
        observations: list[TrafficSignObservation] | None,
        robot_pos: Waypoint,
        lidar_proposals: list[tuple[float, float]] | None = None,
    ) -> None:
        """Fold a frame of observations into the discovered sign list.

        Appending only ever grows ``_signs``, which is what keeps the
        index-keyed ``_passed``/``_engaged`` bookkeeping valid. A published
        sign's position is refined in place for the same reason.

        ``lidar_proposals`` land BEFORE the camera frame so that a sign seen by
        both this tick is positioned by the LIDAR and coloured by the camera,
        rather than the camera creating the track and the proposal merely
        joining it.
        """
        if self._sign_map is None:
            return

        self._sign_map.propose(lidar_proposals, robot_pos)
        self._sign_map.observe(observations, robot_pos)

        for track in self._sign_map.newly_confirmed():
            track.published_index = len(self._signs)
            spec = track.as_spec()
            self._signs.append(spec)
            self._sign_corridors.append(self._geometric_corridor(spec))
            logger.info(
                "Discovered %s sign %d at (%.2f, %.2f) from %d detections",
                spec.color,
                track.published_index,
                spec.x,
                spec.y,
                track.hits,
            )

        # The map cannot see these two facts and both change what it may do: a
        # committed slot must not be re-pointed underneath the router, and a
        # PASSED index must never be re-pointed at all (the new pillar would
        # inherit the "behind us" flag and vanish for the lap).
        commit = getattr(self._sign_map, "set_committed", None)
        if commit is not None:
            commit(self._committed)
        retire = getattr(self._sign_map, "retire", None)
        if retire is not None:
            for passed_index in self._passed:
                retire(passed_index)

        for track in self._sign_map.published():
            index = track.published_index
            if index is None:
                continue
            spec = track.as_spec()
            if spec != self._signs[index]:
                self._signs[index] = spec
                self._sign_corridors[index] = self._corridor_for_spec_settled(index, spec)

    def _corridor_for_spec_settled(self, index: int, spec: SignSpec) -> Section:
        """``_settled_corridor``, then the satisfiability preference on top.

        Order matters: the damping runs first so the flip streak still sees the
        raw geometric answer, and the relabel only overrides the FINAL choice.
        Relabelling before damping would feed the streak counter a corridor that
        ``corridor_for_position`` never proposed.
        """
        settled = self._settled_corridor(index, spec)
        if not self._relabel_unsatisfiable:
            return settled
        return satisfiable_corridor(spec, settled, self._lateral_offset, self._direction, self._context)

    def _settled_corridor(self, index: int, spec: SignSpec) -> Section:
        """Corridor for a refined sign estimate, held steady against jitter.

        A sign's corridor is what selects the world axis its deformation treats
        as lateral, and ``corridor_for_position`` is a hard partition with no
        dead zone. On a corner boundary — where two-thirds of legal WRO grid
        positions sit — a discovery estimate wobbling by millimetres therefore
        alternates between two corridors whose lateral axes are ORTHOGONAL, and
        the commanded waypoint jumps between two unrelated targets on every
        tick. The chassis converges on neither.

        A genuine corridor change (the estimate really was in the wrong place
        early on) still lands, just ``corridor_flip_ticks`` later — 0.25 s at
        20 Hz, against a 1.40 m activation distance.

        Temporal rather than a geometric dead-band deliberately: the corner
        tie-break picks the nearest inner face, so moving a point further into
        the corridor you want to keep can make that corridor *less* likely, and
        there is no usable distance-to-decision-surface to threshold on.
        """
        fresh = self._geometric_corridor(spec)
        current = self._sign_corridors[index]
        if fresh == current:
            self._corridor_flip_streak.pop(index, None)
            return current

        candidate, streak = self._corridor_flip_streak.get(index, (fresh, 0))
        streak = streak + 1 if candidate == fresh else 1
        if streak < self._config.corridor_flip_ticks:
            self._corridor_flip_streak[index] = (fresh, streak)
            return current

        self._corridor_flip_streak.pop(index, None)
        logger.info(
            "Sign %d moved %s -> %s after %d consistent ticks at (%.2f, %.2f)",
            index,
            current.name,
            fresh.name,
            streak,
            spec.x,
            spec.y,
        )
        return fresh

    def _corridor_for_spec(self, spec: SignSpec) -> Section:
        """Corridor for a sign, preferring one whose lane target is satisfiable.

        Plain ``corridor_for_position`` unless ``SIGN_LANE_RELABEL_UNSATISFIABLE``
        is set; see ``satisfiable_corridor`` for why the corner tie-break can
        hand a sign a corridor in which no legal lane exists.

        ``SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR`` runs first when set, because it
        fixes the tie-break itself rather than repairing what the tie-break
        produced -- the relabel then applies to an already-sane corridor.
        """
        corridor = self._geometric_corridor(spec)
        if not self._relabel_unsatisfiable:
            return corridor
        return satisfiable_corridor(spec, corridor, self._lateral_offset, self._direction, self._context)

    def _geometric_corridor(self, spec: SignSpec) -> Section:
        """Corner tie-break for a sign, on depth rather than nearest face."""
        corridor = corridor_for_position(spec.x, spec.y)
        if not self._depth_consistent_corridor:
            return corridor
        return depth_consistent_corridor(spec.x, spec.y, corridor)

    @property
    def is_discovering(self) -> bool:
        """True when the sign layout is being found by camera rather than handed over.

        The distinguishing fact about a discovering run is that the robot
        cannot know a corridor's signs until it has entered that corridor --
        signs sit inside corridors and the next one is outside a 102 deg FOV
        until the corner is turned, so observations top out around 2.3 m
        regardless of discovery tuning. Callers use this to treat the first
        lap as reconnaissance; see ``EXPLORE_LAP_SPEED_FRAC``.
        """
        return self._sign_map is not None

    @property
    def lane_specs(self) -> list[tuple[SignSpec, Section]]:
        """Every routed sign paired with the corridor label the router uses for it.

        Pairs rather than positions alone because a sign's corridor is what
        selects the world axis its avoidance treats as lateral, and the
        router's own label is the settled one (see ``_settled_corridor``) --
        recomputing it in the consumer would reintroduce the corner jitter that
        damping exists to suppress.

        Passed signs are INCLUDED, unlike ``routed_sign_positions``: the lane
        is planned geometry, so retiring a sign mid-lap would rewrite the path
        under a chassis that is still on it. The signs come back every lap
        anyway (``reset_for_new_lap``), so the lane is a property of the layout,
        not of this lap's bookkeeping.
        """
        return list(zip(self._signs, self._sign_corridors, strict=True))

    @property
    def lane_fingerprint(self) -> tuple[tuple[float, float, str], ...]:
        """Identity of the current sign layout, for cheap change detection.

        Blind discovery both appends signs and refines existing positions every
        tick, and rebuilding the planned path on a tick where nothing moved
        would re-seek the waypoint index for no reason. Rounded to the
        centimetre so sub-millimetre estimate jitter -- which cannot move a
        waypoint visibly -- does not count as a change.
        """
        return tuple((round(s.x, 2), round(s.y, 2), str(s.color)) for s in self._signs)

    @property
    def active_sign_count(self) -> int:
        """Number of signs not yet marked as passed (this lap)."""
        return len(self._signs) - len(self._passed)

    @property
    def routed_sign_positions(self) -> list[Waypoint]:
        """World positions of the signs this router still intends to route around.

        The reactive collision layer uses this to tell a mapped obstacle it has
        a plan for from an unmapped one it does not (see
        ``collision_avoidance_controller.mask_mapped_obstacles``). Signs already
        marked passed are excluded: the router has stopped steering around them,
        so nothing owns them any more and they get the full reactive guard back.

        Discovered signs are included on the same footing as metadata ones —
        both live in ``_signs`` — but only once ``ObservedSignMap`` has actually
        published them, so an unconfirmed track never suppresses the guard.
        """
        return [Waypoint(s.x, s.y) for i, s in enumerate(self._signs) if i not in self._passed]

    @property
    def routed_sign_positions_by_corridor(self) -> list[tuple[Waypoint, Section]]:
        """``routed_sign_positions``, each paired with the sign's own corridor.

        For ``mask_mapped_obstacles``'s escape-mask attribution: proximity
        alone is not enough to trust a LIDAR ray's endpoint as "this routed
        sign" under a wrong-but-consistent rigid rotation of the believed
        pose, which can reproject a genuinely unmapped obstacle's ray onto a
        routed sign's coordinates by coincidence (the same rotational-lock
        failure ``_SignTrack.corridor`` exists to guard discovery against).
        Requiring the ray's own corridor to match this sign's closes that
        gap without needing to know the rotation is even present.
        """
        return [(Waypoint(s.x, s.y), c) for i, (s, c) in enumerate(self.lane_specs) if i not in self._passed]

    def reset_for_new_lap(self) -> None:
        """Re-arm every sign so it's routed again on the next lap.

        Without this, a sign marked ``_passed`` on lap 1 (once the robot moves
        beyond ``passed_dist``) stays passed for the rest of the run — the
        Obstacles Challenge requires clearing every sign on all 3 laps, not
        just the first time each one is encountered.
        """
        self._passed.clear()
        self._engaged.clear()
        self._lap_tick = 0
        self._committed = None
        self._commit_yaw.clear()
        self._wrong_side.clear()

    @property
    def wrong_side_violations(self) -> set[int]:
        """Sign indices retired as passed on the WRONG side of the corridor.

        Emptied by ``reset_for_new_lap`` so each lap is judged independently
        (a sign avoided correctly on lap 2 after a lap-1 violation is a fresh
        pass, not a reversal of the earlier miss). The simulator stops the run
        the moment this is non-empty.
        """
        return set(self._wrong_side)

    @property
    def committed_sign_position(self) -> Waypoint | None:
        """Believed world position of the sign currently being routed around.

        BELIEVED, not true: a sign seeded from a misclassified parking barrier
        appears here like any other, which is what makes it readable against the
        camera track that produced it. ``None`` when no sign is committed.
        """
        if self._committed is None or not 0 <= self._committed < len(self._signs):
            return None
        spec = self._signs[self._committed]
        return Waypoint(spec.x, spec.y)

    def committed_pass_side_offset(self, point: tuple[float, float]) -> float | None:
        """How far ``point`` sits on the LEGAL side of the committed sign, signed.

        Positive is the side rule 9.19 requires for that sign's colour in the
        current travel direction; negative is the side that ends the round.
        ``None`` when nothing is committed or the direction is unknown -- which
        callers must treat as "the rule is unavailable this tick", never as
        zero, for the reason ``pass_side_lateral_axis`` documents: a coin flip
        between two opposite answers is worse than declining to answer.

        Exists so a caller can ask whether the path it is ALREADY following
        satisfies the rule, rather than inferring it from the magnitude of a
        correction. Magnitude cannot say which side.
        """
        if self._committed is None or not 0 <= self._committed < len(self._signs):
            return None
        spec = self._signs[self._committed]
        axis_sign = pass_side_lateral_axis(spec.corridor, spec.color, self._direction)
        if axis_sign is None:
            return None
        axis, multiplier = axis_sign
        delta = point[0] - spec.x if axis is Axis.X else point[1] - spec.y
        return delta * multiplier

    def _record_pass_side(self, index: int, robot_pos: Waypoint) -> None:
        """Decide whether ``index`` was cleared on its permitted side.

        The permitted side is TRAVEL-RELATIVE -- red is passed on the vehicle's
        right, green on its left (rules 9.19) -- and is exactly the lateral
        direction ``ROUTING_TABLE`` deforms toward for that colour under the
        direction this round is driven. The robot's lateral coordinate relative
        to the sign's is compared against it: same sign ⇒ correct side,
        opposite sign ⇒ wrong-side pass, recorded in ``_wrong_side``.

        The lookup was keyed on a hardcoded ``Direction.CLOCKWISE`` until
        2026-09-03, which was harmless only while both rows of the table were
        identical. It is now ``self._direction``: keying a travel-relative rule
        on a constant direction judges half the rounds against the mirror of
        the rule they are actually driving.

        The comparison uses the robot's position at the instant the sign is
        retired (distance > ``passed_dist``). By then the chassis is ~1.6 m
        down the corridor axis from the sign, but it is travelling *along* that
        axis, so its lateral coordinate is the same one it held abeam the sign
        — which is precisely the choice of side that the pass represents.
        """
        sign = self._signs[index]
        entry = ROUTING_TABLE.get((self._sign_corridors[index], self._direction))
        if entry is None:
            return
        axis = entry.axis
        # An UNKNOWN sign cannot violate a colour-keyed rule: with no colour
        # there is no permitted side to be on the wrong side OF. Scoring it
        # against GREEN's side (what `else green_mult` did) would invent
        # round-ending violations for objects the camera never confirmed.
        if sign.color == SignColor.RED:
            permitted = entry.red_mult
        elif sign.color == SignColor.GREEN:
            permitted = entry.green_mult
        else:
            return
        robot_lat = robot_pos.x if axis == Axis.X else robot_pos.y
        sign_lat = sign.x if axis == Axis.X else sign.y
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        if side != 0 and side not in (0, permitted):
            self._wrong_side.add(index)

    def deform_waypoint(
        self,
        waypoint: tuple[float, float],
        robot_pos: tuple[float, float],
        robot_yaw: float,
        corridor: Section,
        observations: list[TrafficSignObservation] | None = None,
        lidar_proposals: list[tuple[float, float]] | None = None,
    ) -> tuple[float, float]:
        """Return a (possibly laterally deformed) version of the target waypoint.

        Checks all uncleared signs. The NEAREST active sign within activation
        distance drives the deformation. Camera observations are used to confirm
        the sign color if available and within match distance.

        Args:
            waypoint: Current target waypoint (x, y).
            robot_pos: Current robot position (x, y).
            robot_yaw: Robot heading (radians, 0 = east).
            corridor: Current track section.
            observations: Latest world-coordinate sign observations.
            lidar_proposals: World ``(x, y)`` pillar candidates from
                ``lidar_proposer.propose``. Position evidence only -- they carry
                no colour and cannot themselves produce a deformation.

        Returns:
            Deformed waypoint (x, y). Unchanged if no active sign nearby.
        """
        # Discovery first: in blind mode this frame may be what reveals the
        # sign about to be routed around, so it has to land before candidate
        # selection rather than after it.
        robot_wp = Waypoint(*robot_pos)
        self._ingest_observations(observations, robot_wp, lidar_proposals)

        candidates = self._prefer_committed(self._active_sign_candidates(robot_wp, robot_yaw, corridor))

        # Walk candidates nearest-first and use the first whose deformation is
        # actually applicable, rather than giving up entirely if the closest one
        # is not. The nearest sign is often one the robot is still alongside but
        # has effectively cleared, sitting in the corridor just left behind; its
        # own deformation no longer applies to a target point that has already
        # moved into the next corridor. Returning the waypoint untouched in that
        # case blanks out avoidance for exactly the stretch approaching the NEXT
        # sign — which, if that sign sits near the corner exit, is the entire
        # runway available to steer around it.
        for nearest_idx, nearest_dist in candidates:
            # Sorted nearest-first, so once one is out of range every later one
            # is too — nothing further can apply.
            if nearest_dist > self._config.activation_dist:
                return waypoint

            # Key the corner check and the deformation math off the CANDIDATE
            # SIGN's own corridor, not the robot's current corridor label. The
            # two can legitimately disagree right at a corner — see
            # _active_sign_candidates' same-corridor-OR-within-activation_dist
            # comment — and the sign's own corridor is what actually determines
            # which world axis is "lateral" for it; using the robot's (possibly
            # stale, pre-corner) label here would deform the wrong axis.
            sign_corridor = self._sign_corridors[nearest_idx]

            # The deformation model assumes a straight corridor segment (hold
            # the depth axis, override the lateral axis with a value derived
            # from the sign's fixed position). Once the *target* waypoint itself
            # has curved into a corner, that override is stale and increasingly
            # wrong — skip it rather than fight the path's own curve.
            # Deliberately stricter than corridor_for_position()'s corner
            # tie-break (which exists to always assign the ROBOT some corridor,
            # even ambiguously): a corner waypoint like (2.42, 2.42) ties NORTH
            # vs EAST there and gets assigned NORTH by insertion order, but it's
            # still on the turning arc, not the straight segment this
            # deformation model assumes.
            if is_squarely_in_corridor(waypoint[0], waypoint[1], sign_corridor, self._context):
                break
        else:
            # No candidate produced an applicable deformation.
            self._committed = None
            return waypoint

        # Stay with this sign until it is genuinely cleared, rather than
        # re-running the nearest-wins race from scratch next tick.
        if self._committed != nearest_idx:
            self._commit_yaw[nearest_idx] = robot_yaw
        self._committed = nearest_idx
        yaw_drift = abs(wrap_angle(robot_yaw - self._commit_yaw[nearest_idx]))
        sign = self._signs[nearest_idx]
        color = sign.color

        # Optionally override color with camera observation.
        if observations:
            camera_color = match_detection_to_sign(
                observations,
                Waypoint(sign.x, sign.y),
                self._config,
            )
            if camera_color is not None:
                # The enum itself, not `.value`: SignColor is a StrEnum, so the
                # two are equal as strings, and `color` is declared SignColor.
                color = camera_color

        # Taper the offset so it fades in and out over `passed_dist` instead of
        # snapping between full magnitude and zero in a single waypoint step at
        # a corridor boundary — a kink arriving at exactly the same place the
        # car is also turning through.
        #
        # Taper on whichever of the ROBOT or the TARGET POINT is nearer the
        # sign, not the target point alone. The lookahead target runs 0.2-0.4m
        # ahead of the robot, so keying on it alone means that at the instant
        # the robot draws level with the sign — the one moment full offset is
        # actually needed — the target is already that far PAST the sign and
        # the taper has quietly cut the offset by a third or more. The robot
        # then chases a half-hearted target and grazes the sign it was supposed
        # to clear. Taking the minimum holds full strength across the whole real
        # pass (either the robot or its target is near the sign throughout) and
        # decays only once both are clear, which preserves the smoothing this
        # taper exists for. Waypoint-at-sign callers still see taper == 1.0, so
        # single-point behaviour is unchanged.
        waypoint_wp = Waypoint(*waypoint)
        influence_dist = min(
            _dist2d(waypoint_wp, Waypoint(sign.x, sign.y)),
            _dist2d(robot_wp, Waypoint(sign.x, sign.y)),
        )
        # This shape peaks the commanded offset AT the sign: at activation_dist
        # 1.40 against passed_dist 1.60 the taper opens at 0.125, so avoidance
        # asks for 3.5 cm where a mid-turn pass needs 20.4 cm. That looks like
        # the reason the offset arrives late, and it is not. Holding full offset
        # from activation and fading only on the way out was measured over the
        # 256-scenario corpus at ramps of 0.20/0.40/0.70 m: byte-identical
        # without the depth pin (182 collisions at every value), and slightly
        # WORSE with it (137 -> 135 in-time). The lateral clamp saturates before
        # the taper ever binds, so the ramp has nothing to give. Do not re-try
        # it without new information; see docs/sign-avoidance-investigation.md.
        taper = max(0.0, 1.0 - influence_dist / self._config.passed_dist)
        effective_offset = self._lateral_offset * taper

        deformed_wp = apply_deformation(
            waypoint_wp,
            sign,
            color,
            sign_corridor,
            self._direction,
            effective_offset,
            robot_wp if self._config.depth_pin else None,
            self._context,
            yaw_drift,
        )
        deformed = (deformed_wp.x, deformed_wp.y)

        if deformed != waypoint:
            logger.debug(
                "Sign %d (%s) deformation: wp (%.3f,%.3f) → (%.3f,%.3f) [dist=%.2f m]",
                nearest_idx,
                color,
                waypoint[0],
                waypoint[1],
                deformed[0],
                deformed[1],
                nearest_dist,
            )

        return deformed

    def _prefer_committed(self, candidates: list[tuple[int, float]]) -> list[tuple[int, float]]:
        """Keep routing around the sign already being routed around.

        ``_active_sign_candidates`` re-runs a pure nearest-wins race every tick
        with no memory of the previous one. Where two signs are both in play —
        common, since the WRO grid puts them 0.50 m apart along a corridor and
        the corridor is only 1.0 m wide — the winner can flip while the chassis
        is already committed, and the commanded lateral line jumps from one
        sign's required value to the other's in a single tick. Both lines are
        legal; the damage is switching between them with no runway left to
        track the new one. Measured over the 256-scenario corpus, 28 of 229
        collisions had the winner change during the fatal approach.

        So a sign that is still an applicable candidate holds its claim. This
        is deliberately hysteresis on SELECTION only — the deformation math and
        the pass-side rule are untouched, which is what the two clearance-bound
        attempts got wrong (see the investigation doc).

        The claim is dropped as soon as it stops being reachable: when the sign
        retires (``_passed``), falls behind the chassis, leaves
        ``activation_dist``, or yields no applicable deformation. Without the
        distance test a receding sign could hold the claim from beyond its own
        activation range and mask the one coming up — the same masking bug
        already fixed once in the nearest-wins ordering.
        """
        if not self._config.commit_hysteresis or self._committed is None:
            return candidates
        for entry in candidates:
            if entry[0] == self._committed:
                if entry[1] > self._config.activation_dist:
                    break
                return [entry, *(c for c in candidates if c[0] != self._committed)]
        self._committed = None
        return candidates

    def _active_sign_candidates(
        self,
        robot_pos: Waypoint,
        robot_yaw: float,
        corridor: Section,
    ) -> list[tuple[int, float]]:
        """Not-yet-passed signs near ``corridor``, as ``(index, distance)`` nearest-first.

        Also maintains engagement/passed bookkeeping: a sign is engaged once the
        robot comes within activation distance, and retired only after it has
        been engaged and then left beyond ``passed_dist`` — never discarded from
        afar (which would silently disable routing at spawn). Bookkeeping runs
        for every sign regardless of corridor.

        Candidates are additionally restricted to signs not already behind the
        robot (measured along its heading, with ``BEHIND_TOLERANCE`` slack so a
        sign still alongside the chassis keeps holding the line out). A receding
        sign stays geometrically nearer than the next one for a while, so
        without this the nearest-wins rule below masks the upcoming sign until
        it is far too close to steer around.

        The full ranked list is returned, not just the nearest, so the caller can
        fall through to the next one when the closest sign's deformation is not
        applicable to the current target point.

        Candidates are restricted to signs that either belong to
        ``corridor`` or are within ``activation_dist`` of the robot — not
        strict same-corridor equality. A sign one corridor over can sit right
        at a corner (e.g. at that corridor's own "near" grid depth, exactly on
        CORNER_MIN/MAX); the robot's corridor label only flips once its
        cornering arc has already carried it past that point, which is too
        late for any deformation to matter. Requiring same-corridor OR
        within-activation_dist lets a genuinely close cross-corridor sign start
        bending the path before the label flips, while still keeping distant
        cross-corridor signs from being engaged prematurely. The caller
        (``deform_waypoint``) uses the CANDIDATE's own corridor — not this
        method's ``corridor`` argument — for the actual axis/clamp math, so a
        cross-corridor candidate is never run through the wrong convention.
        Engage/pass bookkeeping itself is suppressed for the first
        ``settle_ticks`` of a lap (see ``SignRouterConfig.settle_ticks``);
        candidate selection isn't, so a sign genuinely in the robot's current
        corridor still deforms normally even during that window.

        Returns:
            ``[(index, distance), ...]`` sorted nearest-first; empty when no
            active sign remains.
        """
        self._lap_tick += 1
        settled = self._lap_tick > self._config.settle_ticks
        candidates: list[tuple[int, float]] = []

        for i, sign in enumerate(self._signs):
            if i in self._passed:
                continue
            d = _dist2d(robot_pos, Waypoint(sign.x, sign.y))
            if settled and d < self._config.activation_dist:
                self._engaged.add(i)
            if d > self._config.passed_dist:
                if settled and i in self._engaged:
                    self._passed.add(i)
                    self._record_pass_side(i, robot_pos)
                    logger.debug("Sign %d marked as passed (dist=%.2f m)", i, d)
                continue
            # A sign the robot has already driven past needs no avoidance, and
            # letting one stay a candidate actively HARMS the next sign: the
            # nearest-first ordering would keep ranking the receding sign
            # (still geometrically closer for a while) above the one actually
            # coming up, masking it until it's too close to steer around.
            # Measure along the robot's own heading and keep signs still
            # alongside the chassis (they must go on holding the line out until
            # fully cleared).
            dx, dy = sign.x - robot_pos.x, sign.y - robot_pos.y
            along_track = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
            if along_track < -BEHIND_TOLERANCE:
                continue
            same_corridor = self._sign_corridors[i] == corridor
            # A sign in a DIFFERENT corridor than the robot's current label only
            # qualifies once the robot is within activation_dist of it — i.e.
            # close enough that the sign's own geometry is what actually
            # matters, not the robot's corridor bookkeeping. Without this, a
            # sign sitting right at a corner (e.g. at the corridor's own "near"
            # depth, exactly on CORNER_MIN/MAX) never becomes a deformation
            # candidate until the robot's corridor label flips — which happens
            # only once the robot's cornering arc has already carried it
            # straight past the sign, too late for any deformation to matter.
            # Requiring same-corridor OR within-activation_dist keeps distant
            # cross-corridor signs from being engaged prematurely while still
            # letting a genuinely close one start bending the path early.
            if not same_corridor and d > self._config.activation_dist:
                continue
            candidates.append((i, d))

        candidates.sort(key=lambda entry: entry[1])
        return candidates
