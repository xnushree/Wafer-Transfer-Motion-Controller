import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.trajectory import quintic_trajectory
from src.controller import PIDController


duration = 2
num_points = 500

dt = duration / (num_points - 1)


# R-axis trajectory
time, desired_r, _, _ = quintic_trajectory(
    100,
    400,
    duration,
    num_points
)


# Theta-axis trajectory
_, desired_theta, _, _ = quintic_trajectory(
    0,
    np.pi / 2,
    duration,
    num_points
)


# Z-axis trajectory
_, desired_z, _, _ = quintic_trajectory(
    100,
    250,
    duration,
    num_points
)


# PID controllers
controller_r = PIDController(
    kp=8.0,
    ki=2.0,
    kd=0.2
)

controller_theta = PIDController(
    kp=8.0,
    ki=2.0,
    kd=0.2
)

controller_z = PIDController(
    kp=8.0,
    ki=2.0,
    kd=0.2
)


# Actual robot positions
actual_r = np.zeros(num_points)
actual_theta = np.zeros(num_points)
actual_z = np.zeros(num_points)


actual_r[0] = 100
actual_theta[0] = 0
actual_z[0] = 100


# Closed-loop simulation
# Sampled controller: at step k each axis compares the reference and the
# measured position at the same instant; commands are held until step k + 1
for k in range(num_points - 1):

    control_r = controller_r.update(
        desired_r[k],
        actual_r[k],
        dt
    )

    control_theta = controller_theta.update(
        desired_theta[k],
        actual_theta[k],
        dt
    )

    control_z = controller_z.update(
        desired_z[k],
        actual_z[k],
        dt
    )

    actual_r[k + 1] = actual_r[k] + control_r * dt
    actual_theta[k + 1] = actual_theta[k] + control_theta * dt
    actual_z[k + 1] = actual_z[k] + control_z * dt


# Convert theta to degrees for plotting
desired_theta_deg = np.degrees(desired_theta)
actual_theta_deg = np.degrees(actual_theta)


# Calculate errors
error_r = desired_r - actual_r
error_theta = desired_theta - actual_theta
error_z = desired_z - actual_z


# Performance metrics
rmse_r = np.sqrt(np.mean(error_r ** 2))
rmse_theta = np.sqrt(np.mean(error_theta ** 2))
rmse_z = np.sqrt(np.mean(error_z ** 2))


print("3-Axis PID Performance")
print("----------------------")
print(f"R-axis RMSE: {rmse_r:.2f} mm")
print(f"Theta-axis RMSE: {np.degrees(rmse_theta):.2f} degrees")
print(f"Z-axis RMSE: {rmse_z:.2f} mm")


# Plot R axis
plt.figure()

plt.plot(
    time,
    desired_r,
    label="Desired R"
)

plt.plot(
    time,
    actual_r,
    label="Actual R"
)

plt.xlabel("Time (s)")
plt.ylabel("R (mm)")
plt.title("R-Axis PID Tracking")

plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/r_axis_tracking.png",
    dpi=300
)


# Plot theta axis
plt.figure()

plt.plot(
    time,
    desired_theta_deg,
    label="Desired Theta"
)

plt.plot(
    time,
    actual_theta_deg,
    label="Actual Theta"
)

plt.xlabel("Time (s)")
plt.ylabel("Theta (degrees)")
plt.title("Theta-Axis PID Tracking")

plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/theta_axis_tracking.png",
    dpi=300
)


# Plot Z axis
plt.figure()

plt.plot(
    time,
    desired_z,
    label="Desired Z"
)

plt.plot(
    time,
    actual_z,
    label="Actual Z"
)

plt.xlabel("Time (s)")
plt.ylabel("Z (mm)")
plt.title("Z-Axis PID Tracking")

plt.legend()
plt.grid()
plt.tight_layout()

plt.show()