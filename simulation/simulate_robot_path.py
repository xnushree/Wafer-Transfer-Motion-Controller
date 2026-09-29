import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.robot_kinematics import RThetaZRobot
from src.trajectory import quintic_trajectory


robot = RThetaZRobot(500, 0, 300)

duration = 2
num_points = 200

time, radius, _, _ = quintic_trajectory(
    100,
    400,
    duration,
    num_points
)

_, theta, _, _ = quintic_trajectory(
    0,
    np.pi / 2,
    duration,
    num_points
)

_, z, _, _ = quintic_trajectory(
    50,
    150,
    duration,
    num_points
)


positions = []

for i in range(num_points):

    position = robot.forward_kinematics(
        radius[i],
        theta[i],
        z[i]
    )

    positions.append(position)


positions = np.array(positions)

x = positions[:, 0]
y = positions[:, 1]
z = positions[:, 2]


fig = plt.figure()

ax = fig.add_subplot(111, projection="3d")

ax.plot(x, y, z)

ax.scatter(x[0], y[0], z[0], label="Start")
ax.scatter(x[-1], y[-1], z[-1], label="End")

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")

ax.set_title("Wafer Transfer Robot End-Effector Path")

ax.legend()

plt.tight_layout()

os.makedirs("results", exist_ok=True)

plt.savefig(
    "results/end_effector_path.png",
    dpi=300
)

plt.show()