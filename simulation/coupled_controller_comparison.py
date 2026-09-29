"""PID vs PID + feedforward vs computed torque, on decoupled and coupled plants.

Move: r 0.1 -> 0.4 m, theta 0 -> 90 deg, z 0.1 -> 0.25 m in 2 s (quintic),
then a 0.5 s hold. Extending and rotating at the same time is the case where
the coupling matters most.

Controllers (all sampled every 2 ms, force held between samples):
  A  PID                 baseline gains, gravity compensation
  B  PID + feedforward   baseline gains + feedforward from the DECOUPLED model
                         (constant rotational inertia 0.5 kg m^2, no coupling)
  C  Computed torque     the same gains divided by each axis' nominal inertia,
                         applied through the full COUPLED model M(q), C(q, q_dot)

On the decoupled plant B and C are almost the same controller, so any
difference on the coupled plant comes from modelling the coupling, not from
stronger gains. Results describe this model only.
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.computed_torque_controller import ComputedTorqueController
from src.controller import PIDController
from src.coupled_dynamics import CoupledCylindricalDynamics
from src.feedforward_controller import FeedforwardController
from src.trajectory import quintic_trajectory


move_time = 2.0
hold_time = 0.5
dt = 0.002

masses = np.array([5.0, 0.5, 4.0])        # nominal m_r, J_0, m_z
damping = np.array([1.0, 0.1, 1.0])
gravity = 9.81
baseline_gains = [(300.0, 20.0, 30.0), (100.0, 5.0, 10.0), (300.0, 20.0, 30.0)]

force_min = np.array([-100.0, -20.0, 0.0])
force_max = np.array([100.0, 20.0, 100.0])


def make_plant(coupled):

    return CoupledCylindricalDynamics(
        radial_mass=masses[0],
        radial_damping=damping[0],
        turret_inertia=masses[1],
        rotational_damping=damping[1],
        vertical_mass=masses[2],
        vertical_damping=damping[2],
        gravity=gravity,
        coupled=coupled
    )


#  Reference: quintic move, then hold

n_move = int(round(move_time / dt)) + 1
n_total = int(round((move_time + hold_time) / dt)) + 1
time = np.arange(n_total) * dt

desired = np.zeros((n_total, 3))
desired_v = np.zeros((n_total, 3))
desired_a = np.zeros((n_total, 3))

for j, (start, end) in enumerate([(0.1, 0.4), (0.0, np.pi / 2), (0.1, 0.25)]):
    _, p, v, a = quintic_trajectory(start, end, move_time, n_move)
    desired[:n_move, j], desired_v[:n_move, j], desired_a[:n_move, j] = p, v, a
    desired[n_move:, j] = end


def make_controller(name):

    if name == "PID":
        pids = [PIDController(*g) for g in baseline_gains]

        def command(k, q, v):
            u = np.array([pids[j].update(desired[k, j], q[j], dt) for j in range(3)])
            return u + [0.0, 0.0, masses[2] * gravity]

        return command

    if name == "PID + FF":
        ffs = [
            FeedforwardController(*baseline_gains[0], mass=masses[0], damping=damping[0]),
            FeedforwardController(*baseline_gains[1], mass=masses[1], damping=damping[1]),
            FeedforwardController(*baseline_gains[2], mass=masses[2], damping=damping[2],
                                  gravity=gravity),
        ]

        def command(k, q, v):
            return np.array([
                ffs[j].update(desired[k, j], q[j], desired_v[k, j], desired_a[k, j],
                              v[j], dt)
                for j in range(3)
            ])

        return command

    if name == "Computed torque":
        gains = [tuple(g / m for g in gain) for gain, m in zip(baseline_gains, masses)]
        controller = ComputedTorqueController(make_plant(coupled=True), gains)

        def command(k, q, v):
            return controller.update(desired[k], q, desired_a[k], v, dt)

        return command

    raise ValueError(name)


def simulate(controller_name, coupled):

    plant = make_plant(coupled)
    command = make_controller(controller_name)

    q = np.zeros((n_total, 3))
    v = np.zeros((n_total, 3))
    u = np.zeros((n_total, 3))
    q[0] = desired[0]

    for k in range(n_total):
        u[k] = np.clip(command(k, q[k], v[k]), force_min, force_max)
        if k < n_total - 1:
            q[k + 1], v[k + 1] = plant.rk4_step(q[k], v[k], u[k], dt)

    return q, u


controllers = ["PID", "PID + FF", "Computed torque"]
scale = np.array([1000.0, np.degrees(1.0), 1000.0])
units = ["mm", "deg", "mm"]
axis_names = ["R", "Theta", "Z"]

results = {}
rows = []

print()
print("Controller comparison on decoupled and coupled plants (2 s move + 0.5 s hold)")
print("============================================================================")

for plant_name, coupled in [("Decoupled plant", False), ("Coupled plant", True)]:

    print()
    print(plant_name)
    print(f"  {'Controller':<17}{'Axis':<7}{'RMSE':>12}{'Max':>12}"
          f"{'Final':>12}{'Peak |u|':>11}{'Saturated':>11}")

    for name in controllers:

        q, u = simulate(name, coupled)
        results[(plant_name, name)] = (q, u)

        error = desired - q
        move = slice(0, n_move)

        for j in range(3):
            rmse = np.sqrt(np.mean(error[move, j] ** 2)) * scale[j]
            peak = np.max(np.abs(error[move, j])) * scale[j]
            final = abs(error[-1, j]) * scale[j]
            peak_u = np.max(np.abs(u[:, j] - (masses[2] * gravity if j == 2 else 0)))
            saturated = bool(np.any(np.isclose(u[:, j], force_min[j]) |
                                    np.isclose(u[:, j], force_max[j])))

            print(f"  {name if j == 0 else '':<17}{axis_names[j]:<7}"
                  f"{rmse:>8.4f} {units[j]:<3}{peak:>8.4f} {units[j]:<3}"
                  f"{final:>8.4f} {units[j]:<3}{peak_u:>9.2f}  {'yes' if saturated else 'no':>8}")

            rows.append([plant_name, name, axis_names[j], rmse, peak, final, peak_u,
                         saturated])

print()
print("Peak |u| is N (R, Z) or N*m (theta); for Z it excludes the constant m_z*g.")


#  Save metrics

os.makedirs("results", exist_ok=True)

with open("results/coupled_controller_comparison.csv", "w") as file:
    file.write("plant,controller,axis,rmse,max_error,final_error,peak_effort,saturated\n")
    for row in rows:
        file.write(",".join(str(value) if not isinstance(value, float)
                            else f"{value:.6f}" for value in row) + "\n")


#  Plot tracking error on both plants

figure, axes = plt.subplots(3, 2, sharex=True, figsize=(11, 8))
styles = {
    "PID": dict(color="tab:gray", linewidth=1.2),
    "PID + FF": dict(color="tab:orange", linewidth=1.5),
    "Computed torque": dict(color="tab:blue", linewidth=1.8),
}

for column, plant_name in enumerate(["Decoupled plant", "Coupled plant"]):
    for j in range(3):
        for name in controllers:
            q, _ = results[(plant_name, name)]
            axes[j, column].plot(time, (desired[:, j] - q[:, j]) * scale[j],
                                 label=name, **styles[name])
        axes[j, column].axvline(move_time, color="k", linestyle=":", linewidth=1)
        axes[j, column].set_ylabel(f"{axis_names[j]} error ({units[j]})")
        axes[j, column].grid()
    axes[0, column].set_title(plant_name)
    axes[2, column].set_xlabel("Time (s)")

axes[0, 0].legend(fontsize=8)
figure.suptitle("Tracking error: PID vs PID + feedforward vs computed torque "
                "(dotted line = end of move)")
figure.tight_layout()
figure.savefig("results/coupled_controller_comparison.png", dpi=300)


# Zoomed view without PID, where the feedforward controllers differ

figure, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
for j in range(3):
    for name in ["PID + FF", "Computed torque"]:
        q, _ = results[("Coupled plant", name)]
        axes[j].plot(time, (desired[:, j] - q[:, j]) * scale[j], label=name,
                     **styles[name])
    axes[j].axvline(move_time, color="k", linestyle=":", linewidth=1)
    axes[j].set_ylabel(f"{axis_names[j]} error ({units[j]})")
    axes[j].grid()
axes[0].set_title("Coupled plant: PID + feedforward vs computed torque")
axes[0].legend(fontsize=8)
axes[2].set_xlabel("Time (s)")
figure.tight_layout()
figure.savefig("results/coupled_ff_vs_computed_torque.png", dpi=300)

print("Saved results/coupled_controller_comparison.csv")
print("Saved results/coupled_controller_comparison.png")
print("Saved results/coupled_ff_vs_computed_torque.png")

plt.show()
