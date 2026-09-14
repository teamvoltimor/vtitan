"""Shared pydantic-settings base for a config backed by one checked-in TOML file.

Generalises what used to live under ``src/hardware``: any consumer can subclass
:class:`TomlSettings`, set ``model_config``'s ``toml_file``, and get the same
precedence and profile handling:

- An env var of the same name overrides the TOML value (``env_settings`` runs
  before the TOML source), so systemd EnvironmentFile= and local ``.env``
  overrides keep working without editing the committed tree.
- An active hardware profile (``VTITAN_HARDWARE_PROFILE``, see
  :mod:`shared.config.hardware_profile`) layers an optional
  ``<toml's dir>/profiles/<name>/<toml's filename>`` on top, deep-merged, so an
  overlay only declares the keys it changes. A profile that does not touch a
  file is skipped silently.
- The file read goes through :mod:`shared.config.paths`' single parse cache, so
  a file is parsed once per process no matter how many configs load it.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Self

from pydantic_settings import BaseSettings, PydanticBaseSettingsSource, TomlConfigSettingsSource

from shared.config.hardware_profile import active_profiles
from shared.config.paths import load_toml_merged

if TYPE_CHECKING:
    from collections.abc import Sequence


def _profile_overlay_paths(base_toml: Path) -> list[Path]:
    """``<base_toml's dir>/profiles/<name>/<base_toml's filename>`` for each active profile.

    A missing overlay file is not an error here -- this source skips any path in
    its list that doesn't exist, the same way a missing ``<group>.toml`` falls
    back through the base in :class:`NavigationTuning.load_from_toml_dirs`.
    """
    return [base_toml.parent / "profiles" / name / base_toml.name for name in active_profiles()]


class _CachedTomlSettingsSource(TomlConfigSettingsSource):
    """``TomlConfigSettingsSource`` whose file read goes through the shared cache.

    Subclasses it (rather than implementing the base source) so pydantic-settings
    still recognises ``toml_file`` as a used config key, but overrides the file
    read to route through :func:`shared.config.paths.load_toml_merged`, so the
    TOML is parsed once per process like every other config in the repo, with
    the same profile-overlay deep merge.
    """

    def __init__(
        self,
        settings_cls: type[BaseSettings],
        base_toml: Path,
        overlays: Sequence[Path],
    ) -> None:
        self._base_toml = base_toml
        self._overlays = overlays
        super().__init__(settings_cls, toml_file=base_toml)

    def _read_files(self, files: Any, deep_merge: bool = False) -> dict[str, Any]:  # noqa: ARG002
        return load_toml_merged(self._base_toml, overlays=self._overlays)


class TomlSettings(BaseSettings):
    """Base for a config sourced from its own TOML file, profile overlay and env.

    A subclass sets ``model_config``'s ``toml_file`` to its file and inherits
    :meth:`load` / :meth:`load_with` plus the source precedence above.
    """

    @classmethod
    def load(cls) -> Self:
        """This config, populated from its TOML file, profile overlay and env.

        Equivalent to calling the class with no arguments, which is what every
        caller wants and what pydantic-settings is built to do -- the values
        arrive from ``settings_customise_sources`` below, never from the caller.

        A type checker cannot know that. Pydantic v2 is ``dataclass_transform``-
        ed, so mypy synthesises an ``__init__`` from the model fields, and a
        field that is deliberately REQUIRED with no default (an encoder's
        ``counts_per_rev``, say, whose docstring says exactly why it must not
        have one) reads as a missing named argument at every direct construction
        site. Those sites are correct; the synthesised signature is what is
        wrong. Constructing through ``cls`` sidesteps that synthesis, so prefer
        ``Config.load()`` over ``Config()``.
        """
        return cls()

    @classmethod
    def load_with(cls, **overrides: Any) -> Self:
        """This config, populated from TOML/env/profile, with ``overrides`` applied.

        For a caller that wants the file-backed values with a few keys replaced,
        or that must supply a wrapper field the TOML deliberately does not carry
        (``DetectorConfig.class_to_color``). Init wins, then env, then the TOML
        file, exactly as a direct construction -- so an omitted key still comes
        from the TOML rather than a hardcoded default.

        ``**overrides`` is untyped on purpose: mypy reads a direct partial
        construction against the synthesised ``__init__`` as missing arguments,
        but a ``**kwargs`` call site is not checked that way, so this stays clean
        without a ``type: ignore``.
        """
        return cls(**overrides)

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> tuple[PydanticBaseSettingsSource, ...]:
        """Prefer init/env, then fall back to the TOML config file (+ profile overlay)."""
        base_toml = settings_cls.model_config.get("toml_file")
        if isinstance(base_toml, (str, Path)):
            base_toml = Path(base_toml)
            overlays = _profile_overlay_paths(base_toml)
        else:
            msg = (
                f"{settings_cls.__name__} must declare a 'toml_file' in its model_config to source "
                "its config; refusing to silently run with defaults."
            )
            raise TypeError(msg)
        return (
            init_settings,
            env_settings,
            _CachedTomlSettingsSource(settings_cls, base_toml, overlays),
            dotenv_settings,
            file_secret_settings,
        )
