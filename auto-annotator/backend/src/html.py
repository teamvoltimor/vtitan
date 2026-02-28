"""src.html – HTML snippet generators for the Gradio UI.

All output is self-contained inline HTML suitable for Gradio's ``gr.HTML``
component.  Uses the Catppuccin Mocha palette throughout for visual consistency.

Palette reference (Catppuccin Mocha):
  Crust #11111b  Mantle #181825  Base #1e1e2e
  Surface0 #313244  Surface1 #45475a  Surface2 #585b70
  Overlay0 #6c7086  Subtext1 #bac2de  Text #cdd6f4
  Blue #89b4fa  Green #a6e3a1  Red #f38ba8
  Peach #fab387  Purple #cba6f7  Teal #94e2d5
"""

from __future__ import annotations

from src import db as _db

# ── Shared style tokens ───────────────────────────────────────────────────────
_BASE = "#1e1e2e"
_MANTLE = "#181825"
_CRUST = "#11111b"
_SURFACE0 = "#313244"
_SURFACE1 = "#45475a"
_OVERLAY = "#6c7086"
_TEXT = "#cdd6f4"
_SUBTEXT = "#bac2de"
_BLUE = "#89b4fa"
_GREEN = "#a6e3a1"
_RED = "#f38ba8"
_PEACH = "#fab387"
_PURPLE = "#cba6f7"
_TEAL = "#94e2d5"

_MONO = "JetBrains Mono, ui-monospace, monospace"
_SANS = "Outfit, ui-sans-serif, sans-serif"


def _metric_card(value: str | int, label: str, color: str) -> str:
    return (
        f'<div style="text-align:center;padding:8px 4px">'
        f'<div style="font-family:{_MONO};font-size:22px;font-weight:700;'
        f'color:{color};line-height:1">{value}</div>'
        f'<div style="font-family:{_SANS};font-size:10px;font-weight:600;'
        f'letter-spacing:.1em;text-transform:uppercase;color:{_OVERLAY};'
        f'margin-top:4px">{label}</div>'
        f"</div>"
    )


def stats_html() -> str:
    """Return a Catppuccin Mocha metric-card panel with image status counts.

    Queries the database on every call so counts are always current.

    Returns:
        HTML string with four metric cards (pending, done, skipped, total),
        a percentage badge, and a gradient progress bar.
    """
    s = _db.get_stats()
    bar_fill = int(s.pct)
    pct_color = _GREEN if s.pct >= 80 else _BLUE if s.pct >= 40 else _PEACH

    cards = (
        _metric_card(s.pending, "pending", _RED)
        + _metric_card(s.done, "done", _GREEN)
        + _metric_card(s.skipped, "skipped", _PEACH)
        + _metric_card(s.total, "total", _BLUE)
    )

    return (
        f'<div style="background:{_CRUST};border:1px solid {_SURFACE0};'
        f'border-radius:6px;padding:4px 8px 8px">'
        # metric grid
        f'<div style="display:grid;grid-template-columns:repeat(4,1fr);'
        f'gap:0;border-bottom:1px solid {_SURFACE0};margin-bottom:8px">'
        f"{cards}"
        f"</div>"
        # percentage row
        f'<div style="display:flex;align-items:center;gap:8px">'
        f'<div style="flex:1;background:{_SURFACE0};border-radius:3px;height:5px;overflow:hidden">'
        f'<div style="background:linear-gradient(90deg,{_GREEN},{_TEAL});'
        f'width:{bar_fill}%;height:100%"></div>'
        f"</div>"
        f'<span style="font-family:{_MONO};font-size:11px;font-weight:700;'
        f'color:{pct_color};white-space:nowrap">{s.pct}%</span>'
        f"</div>"
        f"</div>"
    )


def class_swatches_html(classes: list) -> str:
    """Return a row of pill badges for every annotation class.

    Args:
        classes: List of :class:`~src.models.ClassInfo` objects.

    Returns:
        HTML string with one pill per class, showing its index, colour swatch,
        and name in JetBrains Mono.  Returns an italicised placeholder when the
        list is empty.
    """
    if not classes:
        return (
            f'<span style="font-family:{_SANS};font-size:12px;'
            f'color:{_OVERLAY};font-style:italic">No classes defined.</span>'
        )

    pills = []
    for index, cls in enumerate(classes):
        pills.append(
            f'<div style="display:inline-flex;align-items:center;gap:5px;'
            f'padding:3px 8px 3px 6px;border-radius:20px;'
            f'background:{_SURFACE0};border:1px solid {_SURFACE1}">'
            f'<span style="font-family:{_MONO};font-size:9px;'
            f'color:{_OVERLAY};min-width:14px;text-align:right">{index}</span>'
            f'<div style="width:10px;height:10px;border-radius:50%;'
            f'background:{cls.color};flex-shrink:0"></div>'
            f'<span style="font-family:{_MONO};font-size:11px;'
            f'color:{_TEXT}">{cls.name}</span>'
            f"</div>",
        )

    return (
        '<div style="display:flex;flex-wrap:wrap;gap:5px;padding:2px 0">'
        + "".join(pills)
        + "</div>"
    )
