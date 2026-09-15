r"""What would the MAGENTA barrier belief have suppressed, on the real rounds?

The simulator never emits a magenta detection and never mislabels a colour
(``vision_color_flip_rate`` ships at 0.0, unmeasured), so the barrier-as-red
failure is structurally unreachable in the corpus. A bag is therefore the ONLY
instrument that can score
:mod:`src.navigation.planning.barrier_belief`, and this is that instrument.

It replays each bag's recorded detections in order against the recorded pose,
builds the belief out of the magenta boxes exactly as the gateway now does, and
asks of every RED box: would it have been refused?

Two numbers decide whether this is worth shipping, and they pull opposite ways:

* WALL-SHAPED reds suppressed -- the barrier being kept out of the sign map,
  which is the point.
* PILLAR-SHAPED reds suppressed -- real signs wrongly refused, which is the
  cost. A pillar standing legitimately close to the lot is the failure case,
  and it is why the suppression radius is not simply made large.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_barrier_belief.py RUN_DIR [RUN_DIR ...]
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
from scripts.common.bag_io import Topics, decode_detections, decode_nav_debug, open_reader  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402

from scripts.bag.diag_bag_barrier_gate import MAX_PILLAR_ASPECT  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.barrier_belief import BarrierBelief  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_world_point  # noqa: E402


def replay(bag_dir: Path) -> dict[str, float | int | str]:
    """Rebuild the belief from one bag and score it against that bag's reds."""
    sd = get_tuning(None).sign_discovery
    belief = BarrierBelief(
        min_sightings=sd.barrier_belief_min_sightings,
        merge_radius_m=sd.barrier_merge_radius_m,
        suppression_radius_m=sd.barrier_suppression_radius_m,
    )

    reader = open_reader(bag_dir)
    pose: Pose | None = None
    magenta = reds = 0
    wall_supp = wall_total = pillar_supp = pillar_total = 0

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(snap.pose_x, (int, float)) and isinstance(snap.pose_y, (int, float)):
                yaw = snap.pose_yaw if isinstance(snap.pose_yaw, (int, float)) else 0.0
                pose = Pose(x=float(snap.pose_x), y=float(snap.pose_y), yaw=float(yaw))
            continue
        if topic != Topics.VISION_DETECTIONS or pose is None:
            continue
        try:
            payload = json.loads(deserialize_message(data, String).data)
        except Exception:  # noqa: BLE001
            continue
        for det in decode_detections(payload):
            if det.color is SignColor.MAGENTA:
                point = detection_to_world_point(det, pose)
                if point is not None:
                    belief.observe(*point)
                    magenta += 1
                continue
            if det.color is not SignColor.RED:
                continue
            reds += 1
            point = detection_to_world_point(det, pose)
            if point is None:
                continue
            bbox = det.as_bbox()
            height = bbox.y_max - bbox.y_min
            if height <= 0:
                continue
            wall_shaped = (bbox.x_max - bbox.x_min) / height > MAX_PILLAR_ASPECT
            suppressed = belief.suppresses(*point)
            if wall_shaped:
                wall_total += 1
                wall_supp += int(suppressed)
            else:
                pillar_total += 1
                pillar_supp += int(suppressed)

    return {
        "run": bag_dir.name.replace("run_", ""),
        "magenta": magenta,
        "reds": reds,
        "believed lots": len(belief.believed()),
        "wall supp": f"{wall_supp}/{wall_total}",
        "wall %": f"{100.0 * wall_supp / wall_total:.1f}%" if wall_total else "-",
        "PILLAR supp": f"{pillar_supp}/{pillar_total}",
        "PILLAR %": f"{100.0 * pillar_supp / pillar_total:.1f}%" if pillar_total else "-",
    }


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    sd = get_tuning(None).sign_discovery
    print(
        f"belief: min_sightings={sd.barrier_belief_min_sightings} "
        f"merge={sd.barrier_merge_radius_m} suppress={sd.barrier_suppression_radius_m}"
    )
    rows = [replay(Path(b)) for b in sys.argv[1:]]
    headers = list(rows[0].keys())
    print_table([[r[h] for h in headers] for r in rows], headers)
    print(
        "\nwall supp is the BENEFIT (barrier kept out of the sign map);\n"
        "PILLAR supp is the COST (a real sign refused). Read both."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
