import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.feedforward_controller import FeedforwardController
from src.robot_dynamics import CylindricalRobotDynamics
from src.safety import SafetyMonitor
from src.wafer_transfer import Station, WaferTransferSequence


dt = 0.002

force_min = np.array([-100.0, -20.0, 0.0])
force_max = np.array([100.0, 20.0, 100.0])


robot = CylindricalRobotDynamics(
    radial_mass=5.0,
    radial_damping=1.0,
    rotational_inertia=0.5,
    rotational_damping=0.1,
    vertical_mass=4.0,
    vertical_damping=1.0
)

safety = SafetyMonitor()

pick_station = Station("Load port", 0.4, 0.0, 0.10)
place_station = Station("Process chamber", 0.4, np.pi / 2, 0.25)


#  Plan the sequence

sequence = WaferTransferSequence(pick_station, place_station, safety)

if not sequence.run():
    print(f"Sequence refused by safety layer: {sequence.fault}")
    sys.exit(1)

plan_time, plan_position, plan_velocity, plan_acceleration, plan_states = (
    sequence.timeline()
)


#  Resample the plan onto a fixed controller time step, then hold the final
#  position for a while so the settled error can be measured

hold_time = 1.0

time = np.arange(0.0, plan_time[-1] + hold_time + dt / 2, dt)
num_points = len(time)

desired_position = np.column_stack([
    np.interp(time, plan_time, plan_position[:, i]) for i in range(3)
])
desired_velocity = np.column_stack([
    np.interp(time, plan_time, plan_velocity[:, i]) for i in range(3)
])
desired_acceleration = np.column_stack([
    np.interp(time, plan_time, plan_acceleration[:, i]) for i in range(3)
])

state_index = np.clip(
    np.searchsorted(plan_time, time, side="right") - 1,
    0,
    len(plan_states) - 1
)
states = [
    "COMPLETE" if t > plan_time[-1] else plan_states[k]
    for t, k in zip(time, state_index)
]


#  Simulate PID + feedforward tracking

controllers = [
    FeedforwardController(
        kp=300.0,
        ki=20.0,
        kd=30.0,
        mass=robot.radial_mass,
        damping=robot.radial_damping
    ),
    FeedforwardController(
        kp=100.0,
        ki=5.0,
        kd=10.0,
        mass=robot.rotational_inertia,
        damping=robot.rotational_damping
    ),
    FeedforwardController(
        kp=300.0,
        ki=20.0,
        kd=30.0,
        mass=robot.vertical_mass,
        damping=robot.vertical_damping,
        gravity=robot.gravity
    ),
]

position = np.zeros((num_points, 3))
velocity = np.zeros((num_points, 3))
force = np.zeros((num_points, 3))

position[0] = desired_position[0]

# Sampled controller, same timing as the Simulink model: at sample k the
# controller reads q(t_k), the force is held over [t_k, t_k+1) and the plant
# is advanced with one RK4 step (see simulink_validation.py)
for k in range(num_points):

    command = np.array([
        controllers[j].update(
            desired_position[k, j],
            position[k, j],
            desired_velocity[k, j],
            desired_acceleration[k, j],
            velocity[k, j],
            dt
        )
        for j in range(3)
    ])

    force[k] = np.clip(command, force_min, force_max)

    if k < num_points - 1:
        position[k + 1], velocity[k + 1] = robot.rk4_step(
            position[k],
            velocity[k],
            force[k],
            dt
        )


#  Results

error = desired_position - position

rmse = np.sqrt(np.mean(error ** 2, axis=0))
max_error = np.max(np.abs(error), axis=0)
final_error = np.abs(error[-1])

unsafe = [
    i for i in range(num_points)
    if not safety.is_safe(position[i])
]

print()
print("Wafer Pick-Transfer-Place Simulation (PID + Feedforward)")
print("--------------------------------------------------------")
print(f"Motion time: {plan_time[-1]:.2f} s, then {hold_time:.1f} s hold")
print("(Final error is measured at the end of the hold)")
print()
print(f"{'Axis':<8}{'RMSE':>12}{'Max error':>14}{'Final error':>14}")
print(
    f"{'R':<8}{rmse[0] * 1000:>9.3f} mm{max_error[0] * 1000:>11.3f} mm"
    f"{final_error[0] * 1000:>11.3f} mm"
)
print(
    f"{'Theta':<8}{np.degrees(rmse[1]):>8.3f} deg"
    f"{np.degrees(max_error[1]):>10.3f} deg"
    f"{np.degrees(final_error[1]):>10.3f} deg"
)
print(
    f"{'Z':<8}{rmse[2] * 1000:>9.3f} mm{max_error[2] * 1000:>11.3f} mm"
    f"{final_error[2] * 1000:>11.3f} mm"
)
print()
print(
    f"Peak |F_r| {np.max(np.abs(force[:, 0])):.2f} N, "
    f"|T_theta| {np.max(np.abs(force[:, 1])):.2f} N*m, "
    f"F_z {np.min(force[:, 2]):.2f}..{np.max(force[:, 2]):.2f} N"
)
print(
    "Actual path inside workspace and outside forbidden zones: "
    f"{'YES' if not unsafe else f'NO ({len(unsafe)} samples)'}"
)


#  Plots

os.makedirs("results", exist_ok=True)

state_colors = {
    "PICK": "tab:blue",
    "LIFT": "tab:orange",
    "TRANSFER": "tab:green",
    "PLACE": "tab:red",
    "RETRACT": "tab:purple",
    "COMPLETE": "0.6",
}


def shade_states(axis):

    start = 0
    for k in range(1, num_points + 1):
        if k == num_points or states[k] != states[start]:
            axis.axvspan(
                time[start],
                time[k - 1],
                color=state_colors[states[start]],
                alpha=0.12,
                linewidth=0
            )
            start = k


def add_state_legend(figure):

    handles = [
        plt.Rectangle((0, 0), 1, 1, color=color, alpha=0.3)
        for color in state_colors.values()
    ]
    figure.legend(
        handles,
        state_colors.keys(),
        loc="lower center",
        ncol=6,
        frameon=False
    )


scales = [1000, np.degrees(1), 1000]
labels = ["R (mm)", "Theta (deg)", "Z (mm)"]


figure, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

for j in range(3):
    shade_states(axes[j])
    axes[j].plot(time, desired_position[:, j] * scales[j], label="Desired")
    axes[j].plot(
        time,
        position[:, j] * scales[j],
        "--",
        label="Actual"
    )
    axes[j].set_ylabel(labels[j])
    axes[j].grid()

axes[0].legend(loc="upper right")
axes[0].set_title("Wafer Pick-Transfer-Place: Joint Tracking")
axes[2].set_xlabel("Time (s)")
add_state_legend(figure)
figure.tight_layout(rect=(0, 0.04, 1, 1))
figure.savefig("results/wafer_transfer_joints.png", dpi=300)


error_scales = [1000, np.degrees(1), 1000]
error_labels = ["R error (mm)", "Theta error (deg)", "Z error (mm)"]

figure, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

for j in range(3):
    shade_states(axes[j])
    axes[j].plot(time, error[:, j] * error_scales[j])
    axes[j].set_ylabel(error_labels[j])
    axes[j].grid()

axes[0].set_title("Wafer Pick-Transfer-Place: Tracking Error")
axes[2].set_xlabel("Time (s)")
add_state_legend(figure)
figure.tight_layout(rect=(0, 0.04, 1, 1))
figure.savefig("results/wafer_transfer_error.png", dpi=300)


force_labels = ["F_r (N)", "T_theta (N*m)", "F_z (N)"]

figure, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

for j in range(3):
    shade_states(axes[j])
    axes[j].plot(time, force[:, j])
    axes[j].axhline(force_max[j], color="k", linestyle=":", linewidth=1)
    axes[j].axhline(force_min[j], color="k", linestyle=":", linewidth=1)
    axes[j].set_ylabel(force_labels[j])
    axes[j].grid()

axes[0].set_title("Wafer Pick-Transfer-Place: Control Effort (dotted = limits)")
axes[2].set_xlabel("Time (s)")
add_state_legend(figure)
figure.tight_layout(rect=(0, 0.04, 1, 1))
figure.savefig("results/wafer_transfer_effort.png", dpi=300)


figure, axis = plt.subplots(figsize=(7, 7))

angles = np.linspace(-np.pi, np.pi, 400)
r_limits = safety.joint_limits[0]

axis.fill(
    r_limits.position_max * np.cos(angles),
    r_limits.position_max * np.sin(angles),
    color="0.93",
    label="Reachable workspace"
)
axis.fill(
    r_limits.position_min * np.cos(angles),
    r_limits.position_min * np.sin(angles),
    color="white"
)

for zone in safety.forbidden_zones:

    wedge_angles = np.linspace(zone.theta_min, zone.theta_max, 50)
    outline_r = np.concatenate([
        np.full(50, zone.r_max),
        np.full(50, zone.r_min)
    ])
    outline_theta = np.concatenate([wedge_angles, wedge_angles[::-1]])

    axis.fill(
        outline_r * np.cos(outline_theta),
        outline_r * np.sin(outline_theta),
        color="tab:red",
        alpha=0.4,
        label=f"Forbidden zone: {zone.name}"
    )

held = np.array([state in ("TRANSFER", "PLACE") for state in states])

x = position[:, 0] * np.cos(position[:, 1])
y = position[:, 0] * np.sin(position[:, 1])

# Empty and loaded paths overlap (the arm returns along the same line), so
# draw the empty path wide underneath and the loaded path thin on top.
# NaN breaks the line where the other phase takes over.
axis.plot(np.where(held, np.nan, x), np.where(held, np.nan, y),
          color="tab:blue", linewidth=6, alpha=0.4, label="Arm tip (empty)")
axis.plot(np.where(held, x, np.nan), np.where(held, y, np.nan),
          color="tab:green", linewidth=1.8, label="Arm tip (carrying wafer)")

for station in (pick_station, place_station):
    axis.plot(
        station.r * np.cos(station.theta),
        station.r * np.sin(station.theta),
        "ks",
        markersize=8
    )
    axis.annotate(
        station.name,
        (station.r * np.cos(station.theta), station.r * np.sin(station.theta)),
        textcoords="offset points",
        xytext=(8, 8)
    )

axis.plot(0, 0, "k+", markersize=12)
axis.set_aspect("equal")
axis.set_xlabel("x (m)")
axis.set_ylabel("y (m)")
axis.set_title("Wafer Pick-Transfer-Place: Top View")
axis.legend(loc="lower left", fontsize=8)
axis.grid()
figure.tight_layout()
figure.savefig("results/wafer_transfer_top_view.png", dpi=300)


print()
print("Saved results/wafer_transfer_joints.png")
print("Saved results/wafer_transfer_error.png")
print("Saved results/wafer_transfer_effort.png")
print("Saved results/wafer_transfer_top_view.png")

plt.show()
