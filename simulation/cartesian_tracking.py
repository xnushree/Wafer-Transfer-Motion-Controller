import os
import sys

import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.trajectory import quintic_trajectory
from src.controller import PIDController


duration = 2
num_points = 500

dt = duration / (num_points - 1)


# Desired R trajectory
time, desired_r, _, _ = quintic_trajectory(
    100,
    400,
    duration,
    num_points
)


# Desired theta trajectory
_, desired_theta, _, _ = quintic_trajectory(
    0,
    np.pi / 2,
    duration,
    num_points
)


# Desired Z trajectory
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


# Actual joint positions
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


# Convert joint coordinates to Cartesian coordinates
desired_x = desired_r * np.cos(desired_theta)
desired_y = desired_r * np.sin(desired_theta)
desired_z_cartesian = desired_z

actual_x = actual_r * np.cos(actual_theta)
actual_y = actual_r * np.sin(actual_theta)
actual_z_cartesian = actual_z


# Cartesian position error
cartesian_error = np.sqrt(
    (desired_x - actual_x) ** 2
    + (desired_y - actual_y) ** 2
    + (desired_z_cartesian - actual_z_cartesian) ** 2
)


max_cartesian_error = np.max(cartesian_error)

rmse_cartesian = np.sqrt(
    np.mean(cartesian_error ** 2)
)


final_cartesian_error = cartesian_error[-1]


print("Cartesian Tracking Performance")
print("--------------------------------")
print(
    f"Maximum Cartesian error: "
    f"{max_cartesian_error:.2f} mm"
)

print(
    f"Cartesian RMSE: "
    f"{rmse_cartesian:.2f} mm"
)

print(
    f"Final Cartesian error: "
    f"{final_cartesian_error:.2f} mm"
)


# Create results folder if necessary
os.makedirs("results", exist_ok=True)


# 3D trajectory plot
fig = plt.figure()

ax = fig.add_subplot(111, projection="3d")


ax.plot(
    desired_x,
    desired_y,
    desired_z_cartesian,
    label="Desired Path"
)

ax.plot(
    actual_x,
    actual_y,
    actual_z_cartesian,
    label="Actual Path"
)


ax.scatter(
    desired_x[0],
    desired_y[0],
    desired_z_cartesian[0],
    label="Start"
)


ax.scatter(
    desired_x[-1],
    desired_y[-1],
    desired_z_cartesian[-1],
    label="End"
)


ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")

ax.set_title(
    "Wafer Transfer Robot Cartesian Trajectory"
)

ax.legend()

plt.tight_layout()


plt.savefig(
    "results/cartesian_tracking.png",
    dpi=300
)

plt.show()