"""Typed, validated model of the mat's starting-square layout.

The numbers come from ``TrackConstants``/``RobotConstants`` (loaded from
``platform/config/track.toml``/``robot.toml``); this module gives them
a shape and checks the invariants that make them legal starts. The Go
generator validates the same invariants at generation time in
``internal/trackconfig``, so a bad edit to the TOML fails on both sides rather
than producing a robot that starts across a band boundary.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from shared.config.robot_constants import RobotConstants
from shared.config.track_constants import TrackConstants

_robot = RobotConstants.load_default()
_track = TrackConstants.load_default()

__all__ = ["STARTING_ZONE_LAYOUT", "StartingZoneLayout"]


class StartingZoneLayout(BaseModel):
    """The starting square's cross-corridor layout.

    Each side of the mat carries a marked square, a metre along the corridor by
    a metre across, divided into bands out from the outer wall, each band split
    into two cells of ``cell_length``. A band is a legal start only while it
    lies inside the corridor: past the corridor's inner edge the band is under
    the centre square. That is what makes the layout conditional on width —
    0.40 + 0.20 = 0.60 fills a narrow corridor exactly, so it holds two bands
    and four cells, while a wide one holds three bands and six cells.

    Attributes:
        band_widths: Band widths across the corridor, outer wall inward.
        spawn_offsets: Where the robot is placed inside each band, measured out
            from the outer wall. Absolute metres, not fractions of the corridor
            width, and deliberately not the band centres — the rules only
            require the robot to start inside the marked section.
        cell_length: Along-corridor length of one cell.
        cell_centers_along: Along-corridor midpoints of a band's two cells.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    band_widths: tuple[float, ...] = Field(min_length=1)
    spawn_offsets: tuple[float, ...] = Field(min_length=1)
    cell_length: float = Field(gt=0)
    cell_centers_along: tuple[float, float]

    @model_validator(mode="after")
    def _check_offsets_fit_their_bands(self) -> StartingZoneLayout:
        """Reject a layout whose spawns do not sit wholly inside their band.

        A spawn that straddles a boundary puts the chassis in two sections of
        the starting square at once, which is not a legal start. This is the
        invariant the 0.30/0.50/0.70 offsets were chosen to satisfy.
        """
        if len(self.spawn_offsets) != len(self.band_widths):
            msg = (
                f"spawn_offsets has {len(self.spawn_offsets)} entries but there are "
                f"{len(self.band_widths)} bands; one offset per band is required"
            )
            raise ValueError(msg)

        # Read from RobotConstants directly rather than RobotSpecs, so this
        # model depends only on the TOML-sourced values and never on constants.py.
        chassis_width = _robot.chassis.width
        half = chassis_width / 2.0
        edge = 0.0
        for i, (band, offset) in enumerate(zip(self.band_widths, self.spawn_offsets, strict=True)):
            lo, hi = edge, edge + band
            if offset - half < lo - 1e-9 or offset + half > hi + 1e-9:
                msg = f"spawn_offsets[{i}] = {offset} puts a {chassis_width} m chassis outside band {i} ({lo}..{hi})"
                raise ValueError(msg)
            edge = hi
        return self

    def bands_within(self, corridor_width: float) -> int:
        """How many bands fit inside a corridor of this width.

        Args:
            corridor_width: Corridor width in metres.

        Returns:
            The number of legal bands; multiply by two for the cell count.
        """
        edge = 0.0
        count = 0
        for band in self.band_widths:
            if edge + band > corridor_width + 1e-9:
                break
            edge += band
            count += 1
        return count


STARTING_ZONE_LAYOUT: StartingZoneLayout = StartingZoneLayout(
    band_widths=_track.corridor.band_widths,
    spawn_offsets=_track.spawn_offsets(_robot.chassis.width),
    cell_length=_track.starting_zone.default_length,
    cell_centers_along=_track.cell_centers_along,
)
"""The layout as configured, validated at import."""
