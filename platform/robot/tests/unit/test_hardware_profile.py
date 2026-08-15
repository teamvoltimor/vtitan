"""Coverage for shared.config.hardware_profile and its consumers.

The profile-overlay mechanism (VTITAN_HARDWARE_PROFILE -> profile_dirs() ->
deep_merge onto the base config) landed in 090bc43 with no automated tests --
only ad hoc manual runs. This locks down: env-var parsing, the unknown-profile
failure mode, that an unset env var reproduces base-only behavior byte for
byte, and that RobotConstants.load_default() actually merges an overlay.
"""

from __future__ import annotations

import pytest
from shared.config import hardware_profile
from shared.config.robot_constants import RobotConstants


@pytest.fixture(autouse=True)
def _clear_profile_env(monkeypatch):
    monkeypatch.delenv("VTITAN_HARDWARE_PROFILE", raising=False)


def test_active_profiles_defaults_to_empty():
    assert hardware_profile.active_profiles() == []


def test_active_profiles_parses_single_name(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "servo270")
    assert hardware_profile.active_profiles() == ["servo270"]


def test_active_profiles_parses_comma_separated_ordered_list(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "servo270,widetrack")
    assert hardware_profile.active_profiles() == ["servo270", "widetrack"]


def test_active_profiles_strips_whitespace_and_drops_empties(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", " servo270 ,, widetrack ,")
    assert hardware_profile.active_profiles() == ["servo270", "widetrack"]


def test_profile_dirs_empty_when_no_profile_selected():
    assert hardware_profile.profile_dirs() == []


def test_profile_dirs_raises_on_unknown_profile_name(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "not_a_real_profile")

    with pytest.raises(ValueError, match="not_a_real_profile"):
        hardware_profile.profile_dirs()


def test_profile_dirs_resolves_known_profile_directories(monkeypatch, tmp_path):
    (tmp_path / "servo270").mkdir()
    (tmp_path / "widetrack").mkdir()
    monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "servo270,widetrack")

    dirs = hardware_profile.profile_dirs()

    assert dirs == [tmp_path / "servo270", tmp_path / "widetrack"]


def test_profile_dirs_unknown_profile_error_names_expected_path(monkeypatch, tmp_path):
    monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "ghost")

    with pytest.raises(ValueError, match=str(tmp_path / "ghost").replace("\\", "\\\\")):
        hardware_profile.profile_dirs()


class TestRobotConstantsProfileOverlay:
    def test_load_default_with_no_profile_matches_base_only_values(self):
        base = RobotConstants.load_default()

        assert base.steering.servo_max_angle_deg == pytest.approx(90.0)
        assert base.steering.max_wheel_angle_deg == pytest.approx(55.0)

    def test_load_default_merges_the_checked_in_servo270_profile(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "servo270")

        overlaid = RobotConstants.load_default()

        assert overlaid.steering.servo_max_angle_deg == pytest.approx(135.0)
        assert overlaid.steering.max_wheel_angle_deg == pytest.approx(85.0)

    def test_load_default_profile_overlay_leaves_untouched_sections_at_base_values(self, monkeypatch):
        base = RobotConstants.load_default()

        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "servo270")
        overlaid = RobotConstants.load_default()

        assert overlaid.chassis == base.chassis
        assert overlaid.ackermann == base.ackermann
        assert overlaid.drivetrain == base.drivetrain

    def test_load_default_raises_on_unknown_profile(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "definitely_not_a_profile")

        with pytest.raises(ValueError, match="definitely_not_a_profile"):
            RobotConstants.load_default()

    def test_load_default_synthetic_overlay_merges_only_declared_keys(self, monkeypatch, tmp_path):
        profile_dir = tmp_path / "bench_rig"
        profile_dir.mkdir()
        (profile_dir / "robot.toml").write_text(
            "[steering]\nmax_wheel_angle_deg = 42.0\n",
            encoding="utf-8",
        )
        monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "bench_rig")

        overlaid = RobotConstants.load_default()

        assert overlaid.steering.max_wheel_angle_deg == pytest.approx(42.0)
        # servo_max_angle_deg wasn't declared in the overlay -- base value survives.
        assert overlaid.steering.servo_max_angle_deg == pytest.approx(90.0)
