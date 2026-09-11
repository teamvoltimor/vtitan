"""RViz/visualization-only appearance constants for the live scenario view.

These are NOT official WRO track specs -- they are purely how the simulation
draws itself in RViz (marker colours, alphas, line-thickness multipliers, the
floor slab). Official, competition-rules values (wall dimensions/colour, sign
dimensions/colours, corridor geometry) live in ``shared.config.constants`` and
are sourced from ``src/config/track.toml``; this module holds only
what the renderer chooses that no physical track measurement decides.

Loaded from ``src/config/visualization/rviz.toml`` at runtime -- the
single source of truth. Edit the TOML, not the code, to retune the look.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from pydantic import BaseModel, ConfigDict

DEFAULT_CONFIG_PATH: Path = Path(__file__).resolve().parents[2] / "config" / "visualization" / "rviz.toml"
"""src/python/config/visualization/rviz.toml -- resolved relative to this
module's own location rather than the caller's."""


class Rgb(BaseModel):
    """An RGB colour, channels normalised to 0-1.

    A named model rather than a bare ``(r, g, b)`` tuple so call sites read
    ``color.r`` instead of indexing a positional triple that carries no meaning
    at the use site.
    """

    model_config = ConfigDict(frozen=True)

    r: float
    g: float
    b: float


class RvizColors(BaseModel):
    """Colours for the robot-model and utility markers.

    The steer-arrow colours are per axle on purpose: the two axles turn in
    OPPOSITE directions on this chassis, so colouring them apart is what makes
    the counter-phase legible at a glance instead of looking like one axle
    drawn twice.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    floor: Rgb = Rgb(r=1.0, g=1.0, b=1.0)
    chassis: Rgb = Rgb(r=0.0, g=0.0, b=0.8)
    wheel_rubber: Rgb = Rgb(r=0.1, g=0.1, b=0.1)
    wheel_spoke: Rgb = Rgb(r=0.9, g=0.9, b=0.35)
    lidar: Rgb = Rgb(r=0.1, g=0.1, b=0.1)
    camera: Rgb = Rgb(r=0.2, g=0.2, b=0.2)
    camera_facing: Rgb = Rgb(r=1.0, g=1.0, b=0.0)
    front_axle_arrow: Rgb = Rgb(r=0.0, g=0.9, b=0.9)
    rear_axle_arrow: Rgb = Rgb(r=1.0, g=0.5, b=0.0)
    sign_floor_marking: Rgb = Rgb(r=0.0, g=0.0, b=0.0)


class RvizVisualizationConfig(BaseModel):
    """All RViz-only appearance tuning for the live visualizer.

    Floor slab: a thin cube so no side faces show at shallow viewing angles
    (a thick slab's sides read as an opaque wall and swamp the mat colour).
    Top face sits ~1 mm below z=0 so line markings render on top.

    Floor-marking line width = starting-zone thickness (1 mm) ×
    ``floor_line_thickness_factor``. The 1 mm print is a hairline at normal
    viewing distance, so the factor scales it up for legibility while keeping
    it recognisably a printed line.

    Chassis alpha is well below half on purpose: the wheels sit INSIDE the
    chassis box, so at higher alpha the running gear is there but not readable
    through the body.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    floor_thickness: float = 0.001  # 1 mm
    floor_alpha: float = 1.0

    # Floor-marking line width = starting-zone thickness (1 mm) × this factor.
    # Scales the hairline print up for legibility at normal viewing distance.
    floor_line_thickness_factor: float = 3.0

    # Chassis is semi-transparent so the wheels (which sit inside the chassis box)
    # stay readable through the body.
    chassis_alpha: float = 0.3

    colors: RvizColors = RvizColors()

    @classmethod
    def load_default(cls) -> RvizVisualizationConfig:
        """Load from the checked-in ``src/config/visualization/rviz.toml``.

        Falls back to the hardcoded defaults if the file is absent, so the
        visualizer still works in a stripped-down test/sim context.
        """
        if not DEFAULT_CONFIG_PATH.exists():
            return cls()
        with DEFAULT_CONFIG_PATH.open("rb") as f:
            data: dict[str, object] = tomllib.load(f)
        return cls.model_validate(data)
