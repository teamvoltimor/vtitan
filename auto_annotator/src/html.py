"""src.html – HTML snippet generators for the Gradio UI."""

from src.db import get_stats


def _swatch_html(color: str, name: str = "") -> str:
    """Small HTML badge: coloured square + hex + optional class name."""
    label = f"{name}  {color}" if name else color
    return (
        '<div style="display:flex;align-items:center;gap:6px;'
        'padding:3px 8px;border-radius:4px;background:#313244;'
        'font-family:monospace;font-size:12px">'
        f'<div style="width:14px;height:14px;border-radius:3px;'
        f'background:{color};border:1px solid #45475a;flex-shrink:0"></div>'
        f'<span style="color:#cdd6f4">{label}</span>'
        "</div>"
    )


def stats_html() -> str:
    """Catppuccin-styled progress bar with pending / done / skipped counts."""
    s = get_stats()
    pct = s["pct"]
    bar_fill = int(pct)
    return (
        '<div style="font-family:monospace;background:#1e1e2e;color:#cdd6f4;'
        'padding:8px 12px;border-radius:6px;font-size:13px;line-height:1.6">'
        f'Pending: <b style="color:#f38ba8">{s["pending"]}</b> &nbsp;'
        f'Done: <b style="color:#a6e3a1">{s["done"]}</b> &nbsp;'
        f'Skipped: <b style="color:#fab387">{s["skipped"]}</b> &nbsp;'
        f'Total: <b style="color:#89b4fa">{s["total"]}</b> &nbsp;|&nbsp; '
        f'<b style="color:#cba6f7">{pct}%</b>'
        '<div style="background:#313244;border-radius:4px;height:6px;margin-top:4px">'
        f'<div style="background:#a6e3a1;width:{bar_fill}%;height:100%;border-radius:4px"></div>'
        "</div></div>"
    )
