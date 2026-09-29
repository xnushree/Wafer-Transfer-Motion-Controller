import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.trajectory import quintic_trajectory
from src.controller import PIDController
from src.robot_dynamics import CylindricalRobotDynamics


duration = 2.0
num_points = 1000

dt = duration / (num_points - 1)


robot = CylindricalRobotDynamics(
    radial_mass=5.0,
    radial_damping=1.0,
    rotational_inertia=0.5,
    rotational_damping=0.1,
    vertical_mass=4.0,
    vertical_damping=1.0
)


r_time, r_desired, r_velocity, r_acceleration = quintic_trajectory(
    0.1,
    0.4,
    duration,
    num_points
)

theta_time, theta_desired, theta_velocity, theta_acceleration = quintic_trajectory(
    0.0,
    np.pi / 2,
    duration,
    num_points
)

z_time, z_desired, z_velocity, z_acceleration = quintic_trajectory(
    0.1,
    0.25,
    duration,
    num_points
)


r_controller = PIDController(
    kp=300.0,
    ki=20.0,
    kd=30.0
)

theta_controller = PIDController(
    kp=100.0,
    ki=5.0,
    kd=10.0
)

z_controller = PIDController(
    kp=300.0,
    ki=20.0,
    kd=30.0
)


actual_position = np.zeros((num_points, 3))
actual_velocity = np.zeros((num_points, 3))

actual_position[0] = [
    0.1,
    0.0,
    0.1
]


radial_force = np.zeros(num_points)
rotational_torque = np.zeros(num_points)
vertical_force = np.zeros(num_points)


# Sampled controller, same timing as the Simulink model: at step k the
# controller reads the position at t_k, the force is held over [t_k, t_k+1)
# and the plant is advanced with one RK4 step
for k in range(num_points):

    r_control = r_controller.update(
        r_desired[k],
        actual_position[k, 0],
        dt
    )

    theta_control = theta_controller.update(
        theta_desired[k],
        actual_position[k, 1],
        dt
    )

    z_control = z_controller.update(
        z_desired[k],
        actual_position[k, 2],
        dt
    )

    radial_force[k] = np.clip(r_control, -100.0, 100.0)
    rotational_torque[k] = np.clip(theta_control, -20.0, 20.0)
    vertical_force[k] = np.clip(
        z_control + robot.vertical_mass * robot.gravity,
        0.0,
        100.0
    )

    if k < num_points - 1:
        actual_position[k + 1], actual_velocity[k + 1] = robot.rk4_step(
            actual_position[k],
            actual_velocity[k],
            [radial_force[k], rotational_torque[k], vertical_force[k]],
            dt
        )


r_error = r_desired - actual_position[:, 0]
theta_error = theta_desired - actual_position[:, 1]
z_error = z_desired - actual_position[:, 2]


r_rmse = np.sqrt(np.mean(r_error ** 2))
theta_rmse = np.sqrt(np.mean(theta_error ** 2))
z_rmse = np.sqrt(np.mean(z_error ** 2))


print("Dynamic Robot Simulation")
print("------------------------")
print(f"R-axis RMSE: {r_rmse * 1000:.2f} mm")
print(f"Theta-axis RMSE: {np.degrees(theta_rmse):.2f} degrees")
print(f"Z-axis RMSE: {z_rmse * 1000:.2f} mm")


os.makedirs("results", exist_ok=True)


plt.figure()

plt.plot(
    r_time,
    r_desired * 1000,
    label="Desired"
)

plt.plot(
    r_time,
    actual_position[:, 0] * 1000,
    label="Actual"
)

plt.xlabel("Time (s)")
plt.ylabel("R Position (mm)")
plt.title("Dynamic R-Axis Tracking")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/dynamic_r_tracking.png",
    dpi=300
)

plt.show()


plt.figure()

plt.plot(
    theta_time,
    np.degrees(theta_desired),
    label="Desired"
)

plt.plot(
    theta_time,
    np.degrees(actual_position[:, 1]),
    label="Actual"
)

plt.xlabel("Time (s)")
plt.ylabel("Theta Position (degrees)")
plt.title("Dynamic Theta-Axis Tracking")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/dynamic_theta_tracking.png",
    dpi=300
)

plt.show()


plt.figure()

plt.plot(
    z_time,
    z_desired * 1000,
    label="Desired"
)

plt.plot(
    z_time,
    actual_position[:, 2] * 1000,
    label="Actual"
)

plt.xlabel("Time (s)")
plt.ylabel("Z Position (mm)")
plt.title("Dynamic Z-Axis Tracking")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/dynamic_z_tracking.png",
    dpi=300
)

plt.show()