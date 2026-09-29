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


time, desired_position, _, _ = quintic_trajectory(
    100,
    400,
    duration,
    num_points
)


controller = PIDController(
    kp=8.0,
    ki=2.0,
    kd=0.2
)


actual_position = np.zeros(num_points)

actual_position[0] = 100


# Sampled controller: at step k it compares the reference and the measured
# position at the same instant, and its command is held until step k + 1
for k in range(num_points - 1):

    control = controller.update(
        desired_position[k],
        actual_position[k],
        dt
    )

    actual_position[k + 1] = (
        actual_position[k]
        + control * dt
    )


error = desired_position - actual_position

max_error = np.max(np.abs(error))

rmse = np.sqrt(np.mean(error ** 2))

final_error = error[-1]

overshoot = np.max(actual_position) - np.max(desired_position)

print("PID Performance")
print("----------------")
print(f"Maximum absolute error: {max_error:.2f} mm")
print(f"RMSE: {rmse:.2f} mm")
print(f"Final error: {final_error:.2f} mm")
print(f"Overshoot: {overshoot:.2f} mm")


plt.figure()

plt.plot(
    time,
    desired_position,
    label="Desired Position"
)

plt.plot(
    time,
    actual_position,
    label="Actual Position"
)

plt.xlabel("Time (s)")
plt.ylabel("R Position (mm)")

plt.title("PID Control of Robot R Axis")

plt.legend()
plt.grid()

plt.tight_layout()

os.makedirs("results", exist_ok=True)

plt.savefig(
    "results/pid_tracking.png",
    dpi=300
)

plt.show()


plt.figure()

plt.plot(
    time,
    error
)

plt.xlabel("Time (s)")
plt.ylabel("Position Error (mm)")

plt.title("PID Tracking Error")

plt.grid()

plt.tight_layout()

plt.savefig(
    "results/pid_error.png",
    dpi=300
)

plt.show()