import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.trajectory import quintic_trajectory
from src.controller import PIDController
from src.feedforward_controller import FeedforwardController
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


_, r_desired, r_velocity, r_acceleration = quintic_trajectory(
    0.1,
    0.4,
    duration,
    num_points
)

_, theta_desired, theta_velocity, theta_acceleration = quintic_trajectory(
    0.0,
    np.pi / 2,
    duration,
    num_points
)

time, z_desired, z_velocity, z_acceleration = quintic_trajectory(
    0.1,
    0.25,
    duration,
    num_points
)


def simulate_pid():

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

    position = np.zeros((num_points, 3))
    velocity = np.zeros((num_points, 3))

    position[0] = [0.1, 0.0, 0.1]

    # Sampled controller (same timing as Simulink): read q(t_k), hold the
    # force over [t_k, t_k+1), advance the plant with one RK4 step
    for k in range(num_points - 1):

        force = np.clip(
            [
                r_controller.update(r_desired[k], position[k, 0], dt),
                theta_controller.update(theta_desired[k], position[k, 1], dt),
                z_controller.update(z_desired[k], position[k, 2], dt)
                + robot.vertical_mass * robot.gravity,
            ],
            [-100.0, -20.0, 0.0],
            [100.0, 20.0, 100.0]
        )

        position[k + 1], velocity[k + 1] = robot.rk4_step(
            position[k],
            velocity[k],
            force,
            dt
        )

    return position


def simulate_feedforward():

    r_controller = FeedforwardController(
        kp=300.0,
        ki=20.0,
        kd=30.0,
        mass=robot.radial_mass,
        damping=robot.radial_damping
    )

    theta_controller = FeedforwardController(
        kp=100.0,
        ki=5.0,
        kd=10.0,
        mass=robot.rotational_inertia,
        damping=robot.rotational_damping
    )

    z_controller = FeedforwardController(
        kp=300.0,
        ki=20.0,
        kd=30.0,
        mass=robot.vertical_mass,
        damping=robot.vertical_damping,
        gravity=robot.gravity
    )

    position = np.zeros((num_points, 3))
    velocity = np.zeros((num_points, 3))

    position[0] = [0.1, 0.0, 0.1]

    # Sampled controller (same timing as Simulink): read q(t_k), hold the
    # force over [t_k, t_k+1), advance the plant with one RK4 step
    for k in range(num_points - 1):

        force = np.clip(
            [
                r_controller.update(
                    r_desired[k], position[k, 0], r_velocity[k],
                    r_acceleration[k], velocity[k, 0], dt
                ),
                theta_controller.update(
                    theta_desired[k], position[k, 1], theta_velocity[k],
                    theta_acceleration[k], velocity[k, 1], dt
                ),
                z_controller.update(
                    z_desired[k], position[k, 2], z_velocity[k],
                    z_acceleration[k], velocity[k, 2], dt
                ),
            ],
            [-100.0, -20.0, 0.0],
            [100.0, 20.0, 100.0]
        )

        position[k + 1], velocity[k + 1] = robot.rk4_step(
            position[k],
            velocity[k],
            force,
            dt
        )

    return position


pid_position = simulate_pid()
feedforward_position = simulate_feedforward()


pid_error = np.column_stack([
    r_desired,
    theta_desired,
    z_desired
]) - pid_position

feedforward_error = np.column_stack([
    r_desired,
    theta_desired,
    z_desired
]) - feedforward_position


pid_rmse = np.sqrt(
    np.mean(pid_error ** 2, axis=0)
)

feedforward_rmse = np.sqrt(
    np.mean(feedforward_error ** 2, axis=0)
)


pid_max_error = np.max(
    np.abs(pid_error),
    axis=0
)

feedforward_max_error = np.max(
    np.abs(feedforward_error),
    axis=0
)


print()
print("Controller Comparison")
print("---------------------")

print()
print("PID Controller")
print(f"R RMSE: {pid_rmse[0] * 1000:.3f} mm")
print(f"Theta RMSE: {np.degrees(pid_rmse[1]):.3f} degrees")
print(f"Z RMSE: {pid_rmse[2] * 1000:.3f} mm")

print()
print("PID + Feedforward Controller")
print(f"R RMSE: {feedforward_rmse[0] * 1000:.3f} mm")
print(
    f"Theta RMSE: "
    f"{np.degrees(feedforward_rmse[1]):.3f} degrees"
)
print(f"Z RMSE: {feedforward_rmse[2] * 1000:.3f} mm")

print()
print("Maximum Absolute Error")

print(
    f"R - PID: "
    f"{pid_max_error[0] * 1000:.3f} mm"
)

print(
    f"R - PID + Feedforward: "
    f"{feedforward_max_error[0] * 1000:.3f} mm"
)

print(
    f"Theta - PID: "
    f"{np.degrees(pid_max_error[1]):.3f} degrees"
)

print(
    f"Theta - PID + Feedforward: "
    f"{np.degrees(feedforward_max_error[1]):.3f} degrees"
)

print(
    f"Z - PID: "
    f"{pid_max_error[2] * 1000:.3f} mm"
)

print(
    f"Z - PID + Feedforward: "
    f"{feedforward_max_error[2] * 1000:.3f} mm"
)


os.makedirs("results", exist_ok=True)


plt.figure()

plt.plot(
    time,
    r_desired * 1000,
    label="Desired"
)

plt.plot(
    time,
    pid_position[:, 0] * 1000,
    label="PID"
)

plt.plot(
    time,
    feedforward_position[:, 0] * 1000,
    label="PID + Feedforward"
)

plt.xlabel("Time (s)")
plt.ylabel("R Position (mm)")
plt.title("R-Axis Controller Comparison")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/r_controller_comparison.png",
    dpi=300
)

plt.show()


plt.figure()

plt.plot(
    time,
    np.degrees(theta_desired),
    label="Desired"
)

plt.plot(
    time,
    np.degrees(pid_position[:, 1]),
    label="PID"
)

plt.plot(
    time,
    np.degrees(feedforward_position[:, 1]),
    label="PID + Feedforward"
)

plt.xlabel("Time (s)")
plt.ylabel("Theta Position (degrees)")
plt.title("Theta-Axis Controller Comparison")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/theta_controller_comparison.png",
    dpi=300
)

plt.show()


plt.figure()

plt.plot(
    time,
    z_desired * 1000,
    label="Desired"
)

plt.plot(
    time,
    pid_position[:, 2] * 1000,
    label="PID"
)

plt.plot(
    time,
    feedforward_position[:, 2] * 1000,
    label="PID + Feedforward"
)

plt.xlabel("Time (s)")
plt.ylabel("Z Position (mm)")
plt.title("Z-Axis Controller Comparison")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(
    "results/z_controller_comparison.png",
    dpi=300
)

plt.show()