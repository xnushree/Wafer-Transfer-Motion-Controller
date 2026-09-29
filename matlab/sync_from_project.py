"""Refresh this folder from the working MATLAB project.

The MATLAB/Simulink work lives in its own project folder, which is kept
outside Git on purpose. This script copies the files worth publishing into
matlab/ so the repository has a snapshot of them:

    python matlab/sync_from_project.py [path/to/MATLAB/project]

Default source: the sibling folder Wafer-Transfer-Motion-Control-Robot-MATLAB.
Copied: models/*.slx, scripts/*.m, results/*.png and *.csv.
Skipped: Simulink caches (*.slxc, slprj/), autosaves, the MATLAB project
metadata, and the empty hand-build model r_axis_model.slx.
"""

import os
import shutil
import sys


here = os.path.dirname(os.path.abspath(__file__))
source = (
    sys.argv[1] if len(sys.argv) > 1
    else os.path.join(here, "..", "..", "Wafer-Transfer-Motion-Control-Robot-MATLAB")
)
source = os.path.abspath(source)

if not os.path.isdir(source):
    sys.exit(f"MATLAB project not found: {source}")

RULES = {
    "models": (".slx",),
    "scripts": (".m",),
    "results": (".png", ".csv"),
}
SKIP = {"r_axis_model.slx"}

copied = 0

for folder, extensions in RULES.items():

    target = os.path.join(here, folder)
    os.makedirs(target, exist_ok=True)

    wanted = {
        name for name in os.listdir(os.path.join(source, folder))
        if name.endswith(extensions) and name not in SKIP
    }

    # Remove files that no longer exist in the project
    for name in os.listdir(target):
        if name.endswith(extensions) and name not in wanted:
            os.remove(os.path.join(target, name))

    for name in sorted(wanted):
        shutil.copy2(os.path.join(source, folder, name), os.path.join(target, name))
        copied += 1

print(f"Copied {copied} files from {source}")
