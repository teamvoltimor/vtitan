"""WRO 2026 simulation enumerations.

These enums are owned by ``shared.domain.enums`` — the canonical, cross-context
definitions. This module re-exports them so existing ``shared.config.enums``
importers keep resolving to the same class objects: one definition per concept,
no silent inequality (or divergent members/behaviour) across import paths.

``Section``/``Direction`` were previously redefined here as plain ``Enum``
(not ``StrEnum``) with a hand-written ``__str__`` — a second, distinct class
from the ``shared.domain.enums`` versions. Two contexts importing "the same"
enum from different paths then held genuinely unequal objects
(``shared.config.enums.Section.NORTH != shared.domain.enums.Section.NORTH``).
``RiskLevel`` was already fixed this way; the rest follow the same pattern.
"""

from __future__ import annotations

from shared.domain.enums import Direction as Direction
from shared.domain.enums import LightingScenario as LightingScenario
from shared.domain.enums import NodeHealth as NodeHealth
from shared.domain.enums import RiskLevel as RiskLevel
from shared.domain.enums import RobotState as RobotState
from shared.domain.enums import ScenarioType as ScenarioType
from shared.domain.enums import Section as Section
