"""Coverage for shared.config.hardware_profile and its consumers.

The profile-overlay mechanism (VTITAN_HARDWARE_PROFILE -> profile_dirs() ->
deep_merge onto the base config) landed in 090bc43 with no automated tests --
only ad hoc manual runs. This locks down: env-var parsing, the unknown-profile
failure mode, that a MISSING component profile is refused rather than guessed,
and that RobotConstants.load_default() composes one profile per physical part.

The "unset env var reproduces base-only behavior" case this file used to pin is
deliberately gone: the base config no longer describes a motor or a servo, so
there is no base-only robot left to reproduce. Naming the parts is now
mandatory, which is the whole point -- an unnamed configuration used to mean
"whichever hardware happens to be checked in", and nothing in a run's behaviour
revealed which one that was.
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
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg")
    assert hardware_profile.active_profiles() == ["270deg-hiwonder-35kg"]


def test_active_profiles_parses_comma_separated_ordered_list(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,widetrack")
    assert hardware_profile.active_profiles() == ["270deg-hiwonder-35kg", "widetrack"]


def test_active_profiles_strips_whitespace_and_drops_empties(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", " 270deg-hiwonder-35kg ,, widetrack ,")
    assert hardware_profile.active_profiles() == ["270deg-hiwonder-35kg", "widetrack"]


def test_profile_dirs_empty_when_no_profile_selected():
    assert hardware_profile.profile_dirs() == []


def test_profile_dirs_raises_on_unknown_profile_name(monkeypatch):
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "not_a_real_profile")

    with pytest.raises(ValueError, match="not_a_real_profile"):
        hardware_profile.profile_dirs()


def test_profile_dirs_resolves_known_profile_directories(monkeypatch, tmp_path):
    (tmp_path / "270deg-hiwonder-35kg").mkdir()
    (tmp_path / "widetrack").mkdir()
    monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,widetrack")

    dirs = hardware_profile.profile_dirs()

    assert dirs == [tmp_path / "270deg-hiwonder-35kg", tmp_path / "widetrack"]


def test_profile_dirs_unknown_profile_error_names_expected_path(monkeypatch, tmp_path):
    monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
    monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "ghost")

    with pytest.raises(ValueError, match=str(tmp_path / "ghost").replace("\\", "\\\\")):
        hardware_profile.profile_dirs()


class TestRobotConstantsProfileOverlay:
    def test_load_default_without_a_component_profile_refuses_to_guess(self, monkeypatch):
        """The whole point of stripping the motor and servo out of the base file.

        Previously this asserted that a bare load returned 90/55 deg -- i.e.
        that a run with no profile named silently modelled whichever servo
        happened to be checked in. That is the failure mode that put a night of
        motor tests on the wrong ceiling, and it is undetectable from behaviour
        alone, so the contract is now inverted: refuse, and say what is missing.
        """
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "")

        with pytest.raises(ValueError, match="max_speed_mps"):
            RobotConstants.load_default()

    def test_load_default_error_names_the_variable_and_the_candidates(self, monkeypatch):
        """A config error is only useful if it carries its own cure."""
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "")

        with pytest.raises(ValueError) as excinfo:
            RobotConstants.load_default()

        message = str(excinfo.value)
        assert "VTITAN_HARDWARE_PROFILE" in message
        assert "270deg-hiwonder-35kg" in message
        assert "rev-hd-hex-motor-6000rpm" in message

    def test_load_default_merges_the_checked_in_servo_profile(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,generic-motor-1500rpm")

        overlaid = RobotConstants.load_default()

        assert overlaid.steering.servo_max_angle_deg == pytest.approx(135.0)
        assert overlaid.steering.max_wheel_angle_deg == pytest.approx(85.0)

    def test_components_compose_independently(self, monkeypatch):
        """Naming a servo and a motor must give exactly that pair.

        This is what replaced the fastonly/wideonly/fastwide cross-product:
        one profile per part, composed at the environment variable, so N
        components need N profiles rather than one per combination.
        """
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "180deg-injora-14kg,rev-hd-hex-motor-6000rpm")
        mixed = RobotConstants.load_default()

        assert mixed.steering.max_wheel_angle_deg == pytest.approx(55.0)
        # Both literals track the profile files they come from. max_speed_mps
        # was 0.234, then an estimated 1.0, and is 0.58 since the 2026-08-30
        # bench. If a profile is re-measured, this moves with it -- the
        # assertion is that the NAMED motor's value wins, not that the value is
        # any particular one.
        assert mixed.drivetrain.max_speed_mps == pytest.approx(0.58)

    def test_load_default_profile_overlay_leaves_untouched_sections_at_base_values(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "180deg-injora-14kg,generic-motor-1500rpm")
        base = RobotConstants.load_default()

        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,generic-motor-1500rpm")
        overlaid = RobotConstants.load_default()

        assert overlaid.chassis == base.chassis
        assert overlaid.ackermann == base.ackermann
        # Only the servo changed, so the motor's section must be untouched.
        assert overlaid.drivetrain == base.drivetrain

    def test_load_default_raises_on_unknown_profile(self, monkeypatch):
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "definitely_not_a_profile")

        with pytest.raises(ValueError, match="definitely_not_a_profile"):
            RobotConstants.load_default()

    def test_load_default_synthetic_overlay_merges_only_declared_keys(self, monkeypatch, tmp_path):
        """A profile supplies its own keys and inherits the rest of its section.

        The overlay must declare every component fact the base file no longer
        does, so this rig stands in for a complete servo + motor pair; the
        assertion is that ``yaw_gain`` (declared only in the base) still
        survives the merge.

        ``max_accel_mps2`` and ``speed_response_tau_s`` used to be the base's
        and were the inheritance being checked here. They have since moved into
        the motor profiles alongside ``max_speed_mps`` -- the base [drivetrain]
        now declares only ``rear_steer_ratio`` and ``yaw_gain`` -- so a rig
        omitting them is no longer a partial overlay, it is an incomplete motor,
        and ``_require_component_facts`` correctly refuses it. The rig declares
        all three, and inheritance is asserted on a key the base still owns.
        """
        profile_dir = tmp_path / "bench_rig"
        profile_dir.mkdir()
        (profile_dir / "robot.toml").write_text(
            "[steering]\nservo_max_angle_deg = 90.0\nmax_wheel_angle_deg = 42.0\n"
            "[drivetrain]\nmax_speed_mps = 0.111\nmax_accel_mps2 = 2.0\nspeed_response_tau_s = 0.35\n",
            encoding="utf-8",
        )
        monkeypatch.setattr(hardware_profile, "PROFILES_ROOT", tmp_path)
        monkeypatch.setenv("VTITAN_HARDWARE_PROFILE", "bench_rig")

        overlaid = RobotConstants.load_default()

        assert overlaid.steering.max_wheel_angle_deg == pytest.approx(42.0)
        assert overlaid.drivetrain.max_speed_mps == pytest.approx(0.111)
        # yaw_gain wasn't declared in the overlay -- base value survives.
        assert overlaid.drivetrain.yaw_gain == pytest.approx(0.55)
