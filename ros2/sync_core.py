"""Copy the tested Python modules from ../src into the ROS 2 package.

The ROS nodes reuse the same code as the Python simulations instead of a
second copy that could drift. Run this after changing anything in src/:

    python ros2/sync_core.py

It rewrites "from src.x import y" to "from wafer_transfer_robot.core.x import y"
so the modules work inside the installed ROS package.
"""

import os
import re


MODULES = [
    "controller.py",
    "feedforward_controller.py",
    "robot_dynamics.py",
    "robot_kinematics.py",
    "safety.py",
    "trajectory.py",
    "wafer_transfer.py",
]

here = os.path.dirname(os.path.abspath(__file__))
source = os.path.join(here, "..", "src")
target = os.path.join(
    here, "src", "wafer_transfer_robot", "wafer_transfer_robot", "core"
)

os.makedirs(target, exist_ok=True)

with open(os.path.join(target, "__init__.py"), "w") as file:
    file.write(
        "# Copied from the Python project's src/ by ros2/sync_core.py.\n"
        "# Edit the originals there, then re-run the sync script.\n"
    )

for name in MODULES:

    with open(os.path.join(source, name)) as file:
        code = file.read()

    code = re.sub(
        r"^from src\.",
        "from wafer_transfer_robot.core.",
        code,
        flags=re.MULTILINE
    )

    header = (
        "# Copied from src/" + name + " by ros2/sync_core.py - do not edit here.\n"
    )

    with open(os.path.join(target, name), "w") as file:
        file.write(header + code)

    print(f"Synced {name}")
