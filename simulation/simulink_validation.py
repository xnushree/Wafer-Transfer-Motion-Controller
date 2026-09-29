"""Cross-check the Python simulation against the Simulink model.

Reads the CSV files written by scripts/export_validation_data.m in the
MATLAB project and compares them with two Python runs of the same move:

  Python (original)  the loop used in controller_comparison.py:
                     semi-implicit Euler plant, error uses the reference at
                     step i and the position from step i-1
  Python (matched)   the same model with Simulink's timing: error at sample k
                     uses q(t_k), the force is held over [t_k, t_k+1) and the
                     plant is advanced with one RK4 step (Simulink ode4)

If "matched" agrees with Simulink to rounding error, the two models are the
same and every difference in "original" comes from the discretisation.
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.controller import PIDController
from src.robot_dynamics import CylindricalRobotDynamics
from src.trajectory import quintic_trajectory


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The repository's matlab/ snapshot, or else the separate MATLAB project
MATLAB_RESULTS = os.path.join(PROJECT_ROOT, "matlab", "results")
if not os.path.exists(os.path.join(MATLAB_RESULTS, "simulink_validation_pid.csv")):
    MATLAB_RESULTS = os.path.join(
        PROJECT_ROOT, "..", "Wafer-Transfer-Motion-Control-Robot-MATLAB", "results"
    )

duration = 2.0
num_points = 1000
dt = duration / (num_points - 1)

force_min = np.array([-100.0, -20.0, 0.0])
force_max = np.array([100.0, 20.0, 100.0])

gains = [(300.0, 20.0, 30.0), (100.0, 5.0, 10.0), (300.0, 20.0, 30.0)]

robot = CylindricalRobotDynamics(
    radial_mass=5.0,
    radial_damping=1.0,
    rotational_inertia=0.5,
    rotational_damping=0.1,
    vertical_mass=4.0,
    vertical_damping=1.0
)

inertia = np.array([
    robot.radial_mass,
    robot.rotational_inertia,
    robot.vertical_mass
])
damping = np.array([
    robot.radial_damping,
    robot.rotational_damping,
    robot.vertical_damping
])
gravity_force = np.array([0.0, 0.0, robot.vertical_mass * robot.gravity])


trajectories = [
    quintic_trajectory(start, end, duration, num_points)
    for start, end in [(0.1, 0.4), (0.0, np.pi / 2), (0.1, 0.25)]
]
time = trajectories[0][0]
desired = np.column_stack([trajectory[1] for trajectory in trajectories])
desired_velocity = np.column_stack([trajectory[2] for trajectory in trajectories])
desired_acceleration = np.column_stack(
    [trajectory[3] for trajectory in trajectories]
)


def feedforward(i, use_feedforward):

    # Gravity compensation is always on, as in controller_comparison.py
    ff = gravity_force.copy()

    if use_feedforward:
        ff += inertia * desired_acceleration[i] + damping * desired_velocity[i]

    return ff


def plant_acceleration(force, velocity):

    return np.array([
        robot.radial_acceleration(force[0], velocity[0]),
        robot.rotational_acceleration(force[1], velocity[1]),
        robot.vertical_acceleration(force[2], velocity[2]),
    ])


def simulate_original(use_feedforward):

    controllers = [PIDController(*gain) for gain in gains]

    position = np.zeros((num_points, 3))
    velocity = np.zeros((num_points, 3))
    force = np.zeros((num_points, 3))

    position[0] = desired[0]

    for i in range(1, num_points):

        pid = np.array([
            controllers[j].update(desired[i, j], position[i - 1, j], dt)
            for j in range(3)
        ])

        force[i] = np.clip(
            pid + feedforward(i, use_feedforward),
            force_min,
            force_max
        )

        velocity[i] = velocity[i - 1] + plant_acceleration(
            force[i],
            velocity[i - 1]
        ) * dt
        position[i] = position[i - 1] + velocity[i] * dt

    force[0] = force[1]

    return position, force


def simulate_matched(use_feedforward):

    controllers = [PIDController(*gain) for gain in gains]

    position = np.zeros((num_points, 3))
    velocity = np.zeros((num_points, 3))
    force = np.zeros((num_points, 3))

    position[0] = desired[0]

    for k in range(num_points):

        pid = np.array([
            controllers[j].update(desired[k, j], position[k, j], dt)
            for j in range(3)
        ])

        force[k] = np.clip(
            pid + feedforward(k, use_feedforward),
            force_min,
            force_max
        )

        if k == num_points - 1:
            break

        # Force held constant over the step (Simulink ode4 + ZOH)
        position[k + 1], velocity[k + 1] = robot.rk4_step(
            position[k],
            velocity[k],
            force[k],
            dt
        )

    return position, force


def load_simulink(name):

    path = os.path.join(MATLAB_RESULTS, f"simulink_validation_{name}.csv")

    if not os.path.exists(path):
        print(f"Missing {path}")
        print("Run matlab/scripts/export_validation_data.m first.")
        sys.exit(1)

    data = np.genfromtxt(path, delimiter=",", names=True)

    return {
        "time": data["t"],
        "position": np.column_stack([data["r"], data["theta"], data["z"]]),
        "reference": np.column_stack(
            [data["r_ref"], data["theta_ref"], data["z_ref"]]
        ),
        "force": np.column_stack([data["F_r"], data["T_theta"], data["F_z"]]),
    }


def tracking_metrics(position):

    error = desired - position

    return (
        np.sqrt(np.mean(error ** 2, axis=0)),
        np.max(np.abs(error), axis=0),
    )


# Units for printing: metres -> mm, radians -> degrees
scale = np.array([1000.0, np.degrees(1.0), 1000.0])
units = ["mm", "deg", "mm"]
axis_names = ["R", "Theta", "Z"]

rows = []
curves = {}

print()
print("Python vs Simulink cross-check (2 s quintic move, nominal plant)")
print("=================================================================")

for case, use_feedforward, label in [
    ("pid", False, "PID"),
    ("pid_ff", True, "PID + FF"),
]:

    simulink = load_simulink(case)

    if not np.allclose(simulink["time"], time, atol=1e-9):
        print("Simulink time grid differs from the Python grid")
        sys.exit(1)

    reference_difference = np.max(np.abs(simulink["reference"] - desired))

    runs = {
        "Simulink": simulink["position"],
        "Python (original)": simulate_original(use_feedforward)[0],
        "Python (matched)": simulate_matched(use_feedforward)[0],
    }
    curves[label] = runs

    print()
    print(f"{label}")
    print(f"  Reference trajectories agree to {reference_difference:.1e}")
    print(
        f"  {'':<20}"
        + "".join(f"{name + ' RMSE':>14}" for name in axis_names)
        + "".join(f"{name + ' max':>13}" for name in axis_names)
    )

    for name, position in runs.items():

        rmse, max_error = tracking_metrics(position)

        print(
            f"  {name:<20}"
            + "".join(
                f"{rmse[j] * scale[j]:>10.3f} {units[j]:<3}" for j in range(3)
            )
            + "".join(
                f"{max_error[j] * scale[j]:>9.3f} {units[j]:<3}"
                for j in range(3)
            )
        )

        rows.append([label, name] + list(rmse * scale) + list(max_error * scale))

    for name in ("Python (original)", "Python (matched)"):

        difference = np.max(np.abs(runs[name] - runs["Simulink"]), axis=0)

        print(
            f"  Max |{name} - Simulink| position: "
            + ", ".join(
                f"{axis_names[j]} {difference[j] * scale[j]:.2e} {units[j]}"
                for j in range(3)
            )
        )


#  Save metrics

os.makedirs("results", exist_ok=True)

header = (
    "controller,run,"
    "r_rmse_mm,theta_rmse_deg,z_rmse_mm,"
    "r_max_mm,theta_max_deg,z_max_mm"
)

with open("results/simulink_validation.csv", "w") as file:
    file.write(header + "\n")
    for row in rows:
        file.write(
            f"{row[0]},{row[1]}," + ",".join(f"{value:.6f}" for value in row[2:])
            + "\n"
        )


#  Plot tracking error of each run, per axis

figure, axes = plt.subplots(3, 2, sharex=True, figsize=(11, 8))
styles = {
    "Simulink": dict(linewidth=3, alpha=0.5),
    "Python (original)": dict(linestyle="--", linewidth=1.2),
    "Python (matched)": dict(linestyle=":", linewidth=1.8, color="k"),
}

for column, (label, runs) in enumerate(curves.items()):

    for j in range(3):

        for name, position in runs.items():
            axes[j, column].plot(
                time,
                (desired[:, j] - position[:, j]) * scale[j],
                label=name,
                **styles[name]
            )

        axes[j, column].set_ylabel(f"{axis_names[j]} error ({units[j]})")
        axes[j, column].grid()

    axes[0, column].set_title(f"{label}: tracking error")
    axes[2, column].set_xlabel("Time (s)")

axes[0, 0].legend(fontsize=8)
figure.suptitle("Python vs Simulink Cross-Check")
figure.tight_layout()
figure.savefig("results/simulink_validation.png", dpi=300)

print()
print("Saved results/simulink_validation.csv")
print("Saved results/simulink_validation.png")

plt.show()
