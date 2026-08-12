"""One-off: derive the small transparent HUD logo mark from the team's square logo asset.

Not wired into any pipeline -- run once by hand whenever the source brand
asset changes, then check in the resulting PNG (see
platform/robot/assets/vision/voltimor-mark.png, loaded by src/vision/hud.py).
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image

_SRC = Path(__file__).resolve().parents[3] / "frontend" / "public" / "voltimor-logo-square.png"
_DST = Path(__file__).resolve().parents[2] / "assets" / "vision" / "voltimor-mark.png"
_MARK_SIZE = 160  # final square size, px -- full logo incl. wordmark needs more room than the icon alone


def main() -> None:
    img = Image.open(_SRC).convert("RGBA")
    bg = img.getpixel((0, 0))
    print(f"source size={img.size} corner colour={bg}")

    # Chroma-key the flat light-blue background to transparent -- it's a
    # generated flat-fill, not a photo, so a plain distance threshold with a
    # soft falloff (for anti-aliased mark edges) is enough, no proper matting
    # needed.
    px = img.load()
    width, height = img.size
    for y in range(height):
        for x in range(width):
            r, g, b, a = px[x, y]
            dist = ((r - bg[0]) ** 2 + (g - bg[1]) ** 2 + (b - bg[2]) ** 2) ** 0.5
            if dist < 12:
                px[x, y] = (r, g, b, 0)
            elif dist < 40:
                px[x, y] = (r, g, b, int(a * (dist - 12) / 28))

    # Keep the full square logo (icon + "VOLTIMOR ROBOTICS TEAM" wordmark) --
    # just the chroma-keyed transparency above, trimmed to the alpha
    # channel's bounding box so there's no leftover keyed-out border.
    mark = img.crop(img.getbbox())

    # Pad to square so it scales/anchors predictably in the HUD corner.
    side = max(mark.size)
    square = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    square.paste(mark, ((side - mark.width) // 2, (side - mark.height) // 2), mark)
    square = square.resize((_MARK_SIZE, _MARK_SIZE), Image.LANCZOS)

    _DST.parent.mkdir(parents=True, exist_ok=True)
    square.save(_DST)
    print(f"wrote {_DST} ({square.size}, mode={square.mode})")


if __name__ == "__main__":
    main()
