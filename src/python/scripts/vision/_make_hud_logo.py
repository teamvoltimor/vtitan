"""One-off: derive the small HUD logo mark from the team's transparent logo asset.

Not wired into any pipeline -- run once by hand whenever the source brand
asset changes, then check in the resulting PNG (see
src/assets/vision/voltimor-mark.png, loaded by src/vision/hud.py).
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image

_SRC = Path(__file__).resolve().parents[4] / "other" / "assets" / "voltimor-logo-transparent.webp"
_DST = Path(__file__).resolve().parents[3] / "assets" / "vision" / "voltimor-mark.png"
_MARK_SIZE = 160  # final square size, px -- full logo incl. wordmark needs more room than the icon alone


def main() -> int:
    img = Image.open(_SRC).convert("RGBA")
    print(f"source size={img.size}")

    # The source is already transparent, so keep the full logo (icon +
    # "VOLTIMOR ROBOTICS TEAM" wordmark) and just trim to the alpha channel's
    # bounding box so there is no leftover margin.
    mark = img.crop(img.getbbox())

    # Pad to square so it scales/anchors predictably in the HUD corner.
    side = max(mark.size)
    square = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    square.paste(mark, ((side - mark.width) // 2, (side - mark.height) // 2), mark)
    square = square.resize((_MARK_SIZE, _MARK_SIZE), Image.LANCZOS)

    _DST.parent.mkdir(parents=True, exist_ok=True)
    square.save(_DST)
    print(f"wrote {_DST} ({square.size}, mode={square.mode})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
