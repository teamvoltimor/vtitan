"""Detection-tuning control selectors for the RPi Camera Module 3 driver.

Typed enums let Config reject an invalid TOML value (e.g. a typo'd AWB mode)
at settings-load time via pydantic, instead of failing later with a KeyError
when the driver builds the libcamera controls dict in connect().
"""

from enum import StrEnum

from libcamera import controls as lc_controls


class AfMode(StrEnum):
    """Autofocus mode."""

    MANUAL = "manual"
    AUTO = "auto"
    CONTINUOUS = "continuous"

    @property
    def libcamera_value(self) -> int:
        """The libcamera `AfMode` control value for this mode."""
        return _AF_MODE_VALUES[self]


class AfSpeed(StrEnum):
    """Autofocus search speed. Only applies when `AfMode` is AUTO or CONTINUOUS."""

    NORMAL = "normal"
    FAST = "fast"

    @property
    def libcamera_value(self) -> int:
        """The libcamera `AfSpeed` control value for this mode."""
        return _AF_SPEED_VALUES[self]


class AeExposureMode(StrEnum):
    """Auto-exposure bias. SHORT favours shorter exposure times (less motion blur, more noise)."""

    NORMAL = "normal"
    SHORT = "short"
    LONG = "long"

    @property
    def libcamera_value(self) -> int:
        """The libcamera `AeExposureMode` control value for this mode."""
        return _AE_EXPOSURE_MODE_VALUES[self]


class AwbMode(StrEnum):
    """Auto white balance mode.

    Sign colour classification (red vs green) is threshold-based, so a fixed
    mode avoids AWB drift shifting hue readings under changing venue lighting.
    """

    AUTO = "auto"
    TUNGSTEN = "tungsten"
    FLUORESCENT = "fluorescent"
    INDOOR = "indoor"
    DAYLIGHT = "daylight"
    CLOUDY = "cloudy"

    @property
    def libcamera_value(self) -> int:
        """The libcamera `AwbMode` control value for this mode."""
        return _AWB_MODE_VALUES[self]


class NoiseReductionMode(StrEnum):
    """Noise reduction mode. HIGH_QUALITY adds latency the control loop can't afford."""

    OFF = "off"
    FAST = "fast"
    HIGH_QUALITY = "high_quality"
    MINIMAL = "minimal"

    @property
    def libcamera_value(self) -> int:
        """The libcamera `NoiseReductionMode` control value for this mode."""
        return _NOISE_REDUCTION_MODE_VALUES[self]


_AF_MODE_VALUES: dict[AfMode, int] = {
    AfMode.MANUAL: lc_controls.AfModeEnum.Manual,
    AfMode.AUTO: lc_controls.AfModeEnum.Auto,
    AfMode.CONTINUOUS: lc_controls.AfModeEnum.Continuous,
}

_AF_SPEED_VALUES: dict[AfSpeed, int] = {
    AfSpeed.NORMAL: lc_controls.AfSpeedEnum.Normal,
    AfSpeed.FAST: lc_controls.AfSpeedEnum.Fast,
}

_AE_EXPOSURE_MODE_VALUES: dict[AeExposureMode, int] = {
    AeExposureMode.NORMAL: lc_controls.AeExposureModeEnum.Normal,
    AeExposureMode.SHORT: lc_controls.AeExposureModeEnum.Short,
    AeExposureMode.LONG: lc_controls.AeExposureModeEnum.Long,
}

_AWB_MODE_VALUES: dict[AwbMode, int] = {
    AwbMode.AUTO: lc_controls.AwbModeEnum.Auto,
    AwbMode.TUNGSTEN: lc_controls.AwbModeEnum.Tungsten,
    AwbMode.FLUORESCENT: lc_controls.AwbModeEnum.Fluorescent,
    AwbMode.INDOOR: lc_controls.AwbModeEnum.Indoor,
    AwbMode.DAYLIGHT: lc_controls.AwbModeEnum.Daylight,
    AwbMode.CLOUDY: lc_controls.AwbModeEnum.Cloudy,
}

_NOISE_REDUCTION_MODE_VALUES: dict[NoiseReductionMode, int] = {
    NoiseReductionMode.OFF: lc_controls.draft.NoiseReductionModeEnum.Off,
    NoiseReductionMode.FAST: lc_controls.draft.NoiseReductionModeEnum.Fast,
    NoiseReductionMode.HIGH_QUALITY: lc_controls.draft.NoiseReductionModeEnum.HighQuality,
    NoiseReductionMode.MINIMAL: lc_controls.draft.NoiseReductionModeEnum.Minimal,
}
