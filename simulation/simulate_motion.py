import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.trajectory import quintic_trajectory


duration = 2
num_points = 200

radius_time, radius, radius_velocity, radius_acceleration = quintic_trajectory(
    100,
    400,
    duration,
    num_points
)

theta_time, theta, theta_velocity, theta_acceleration = quintic_trajectory(
    0,
    np.pi / 2,
    duration,
    num_points
)

z_time, z, z_velocity, z_acceleration = quintic_trajectory(
    50,
    150,
    duration,
    num_points
)

theta_degrees = np.degrees(theta)

plt.figure()

plt.plot(radius_time, radius, label="R (mm)")
plt.plot(theta_time, theta_degrees, label="Theta (degrees)")
plt.plot(z_time, z, label="Z (mm)")

plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.title("Wafer Transfer Robot Joint Trajectory")
plt.legend()
plt.grid()

plt.tight_layout()

os.makedirs("../results", exist_ok=True)

plt.savefig("../results/joint_trajectory.png", dpi=300)

plt.show()