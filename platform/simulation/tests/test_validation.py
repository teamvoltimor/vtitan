"""Unit tests for scenario geometry validation."""

from shared.config.enums import Direction, Section

from src.generation.validation import WorldContext, validate_scenario


def _make_ctx(
    sign_positions: list[tuple[float, float]] | None = None,
    sign_colors: list[str] | None = None,
    parking_config: dict | None = None,
    spawn: tuple[float, float] = (0.5, 0.5),
) -> WorldContext:
    return WorldContext(
        corridor_widths={},
        sign_positions=sign_positions or [],
        sign_colors=sign_colors or [],
        parking_config=parking_config,
        starting_conditions={
            "direction": Direction.CLOCKWISE,
            "section": Section.SOUTH,
            "section_name": "south",
            "position": spawn,
            "yaw": 0.0,
        },
    )


class TestSignBounds:
    def test_valid_signs_pass(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 0.6), (1.5, 1.5), (2.0, 2.4)])
        assert validate_scenario(ctx) == []

    def test_sign_outside_bounds_flagged(self) -> None:
        ctx = _make_ctx(sign_positions=[(-0.1, 1.5)])
        violations = validate_scenario(ctx)
        assert any(v.rule == "sign_bounds" for v in violations)

    def test_sign_at_max_bound_edge_flagged(self) -> None:
        ctx = _make_ctx(sign_positions=[(3.0, 1.5)])
        violations = validate_scenario(ctx)
        assert any(v.rule == "sign_bounds" for v in violations)


class TestSignOverlap:
    def test_well_separated_signs_pass(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 0.6), (2.0, 0.6)])
        assert validate_scenario(ctx) == []

    def test_overlapping_signs_flagged(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 0.6), (1.01, 0.6)])
        violations = validate_scenario(ctx)
        assert any(v.rule == "sign_overlap" for v in violations)

    def test_single_sign_never_overlaps(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.5, 1.5)])
        assert validate_scenario(ctx) == []


class TestParkingBounds:
    def _make_parking(
        self, b1: tuple[float, float] = (1.0, 0.1), b2: tuple[float, float] = (1.3, 0.1)
    ) -> dict:
        return {
            "block1_pos": b1,
            "block2_pos": b2,
            "block1_yaw": 0.0,
            "block2_yaw": 0.0,
            "depth": 1.0,
        }

    def test_valid_parking_passes(self) -> None:
        ctx = _make_ctx(parking_config=self._make_parking())
        assert validate_scenario(ctx) == []

    def test_parking_block_outside_bounds_flagged(self) -> None:
        ctx = _make_ctx(parking_config=self._make_parking(b1=(-0.2, 0.1)))
        violations = validate_scenario(ctx)
        assert any(v.rule == "parking_bounds" for v in violations)


class TestSignParkingClearance:
    def _make_parking(self) -> dict:
        return {
            "block1_pos": (1.0, 0.1),
            "block2_pos": (1.3, 0.1),
            "block1_yaw": 0.0,
            "block2_yaw": 0.0,
            "depth": 1.0,
        }

    def test_sign_far_from_parking_passes(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 0.8)], parking_config=self._make_parking())
        assert validate_scenario(ctx) == []

    def test_sign_too_close_to_parking_flagged(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 0.11)], parking_config=self._make_parking())
        violations = validate_scenario(ctx)
        assert any(v.rule == "sign_parking_clearance" for v in violations)


class TestRobotSpawnClearance:
    def test_spawn_far_from_signs_passes(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.0, 1.5)], spawn=(1.5, 0.2))
        assert validate_scenario(ctx) == []

    def test_spawn_on_sign_flagged(self) -> None:
        ctx = _make_ctx(sign_positions=[(1.5, 0.2)], spawn=(1.5, 0.2))
        violations = validate_scenario(ctx)
        assert any(v.rule == "spawn_sign_clearance" for v in violations)

    def test_spawn_too_close_to_parking_flagged(self) -> None:
        parking = {
            "block1_pos": (1.5, 0.22),
            "block2_pos": (1.8, 0.22),
            "block1_yaw": 0.0,
            "block2_yaw": 0.0,
            "depth": 1.5,
        }
        ctx = _make_ctx(parking_config=parking, spawn=(1.5, 0.2))
        violations = validate_scenario(ctx)
        assert any(v.rule == "spawn_parking_clearance" for v in violations)


class TestValidScenario:
    def test_empty_scenario_valid(self) -> None:
        ctx = _make_ctx()
        assert validate_scenario(ctx) == []
