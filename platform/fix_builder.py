import re

path = "C:/Users/ralva/OneDrive/Documents/private/projects/archived/teamsteelbot/klevor-v2-platform/platform/simulation/src/generation/sdf_builder.py"
with open(path, "r", encoding="utf-8") as f:
    c = f.read()

# Remove random import
c = re.sub(r"import random\n", "", c)

# Remove the functions _compute_zone_placement and _zone_from_parking
c = re.sub(
    r"def _compute_zone_placement.*?def _add_robot_plugin",
    "def _add_robot_plugin",
    c,
    flags=re.DOTALL,
)

with open(path, "w", encoding="utf-8") as f:
    f.write(c)
