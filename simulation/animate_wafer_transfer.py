"""Animated GIF of the pick-transfer-place sequence.

Draws the same simplified robot as the ROS 2 URDF (base, rotating column,
Z carriage, telescoping arm, fork), the two stations with support pins, the
forbidden "pillar" zone and the wafer, following the simulated joint motion
(PID + feedforward, same controller timing as the Simulink model).

    python simulation/animate_wafer_transfer.py

Saves results/wafer_transfer_animation.gif. Rendering takes about a minute.
"""

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.feedforward_controller import FeedforwardController
from src.robot_dynamics import CylindricalRobotDynamics
from src.safety import SafetyMonitor
from src.wafer_transfer import Station, WaferTransferSequence


FPS = 15
HOLD_BEFORE = 0.6       # s shown before the motion starts
HOLD_AFTER = 1.2        # s shown after it ends
WAFER_RADIUS = 0.10     # 200 mm wafer

COLORS = {
    "graphite": (0.16, 0.17, 0.19),
    "housing": (0.80, 0.82, 0.85),
    "aluminium": (0.62, 0.65, 0.69),
    "ceramic": (0.95, 0.95, 0.93),
    "accent": (0.10, 0.42, 0.78),
    "plate": (0.22, 0.23, 0.26),
    "pin": (0.75, 0.77, 0.80),
    "wafer": (0.30, 0.36, 0.50),
    "zone": (0.85, 0.22, 0.22),
    "trail": (0.25, 0.78, 1.00),
    "background": (0.125, 0.137, 0.165),
}


#  Simulate the sequence (same loop as wafer_transfer_simulation.py)

robot = CylindricalRobotDynamics(5.0, 1.0, 0.5, 0.1, 4.0, 1.0)
safety = SafetyMonitor()
pick = Station("Load port", 0.4, 0.0, 0.10)
place = Station("Process chamber", 0.4, np.pi / 2, 0.25)

sequence = WaferTransferSequence(pick, place, safety)
if not sequence.run():
    sys.exit(f"Sequence refused: {sequence.fault}")

plan_time, plan_q, plan_v, plan_a, plan_states = sequence.timeline()

dt = 0.002
time = np.arange(0.0, plan_time[-1] + dt / 2, dt)

def resample(values):
    return np.column_stack([np.interp(time, plan_time, values[:, j]) for j in range(3)])

desired, desired_v, desired_a = resample(plan_q), resample(plan_v), resample(plan_a)

controllers = [
    FeedforwardController(300.0, 20.0, 30.0, mass=5.0, damping=1.0),
    FeedforwardController(100.0, 5.0, 10.0, mass=0.5, damping=0.1),
    FeedforwardController(300.0, 20.0, 30.0, mass=4.0, damping=1.0, gravity=9.81),
]
force_min, force_max = np.array([-100.0, -20.0, 0.0]), np.array([100.0, 20.0, 100.0])

q = np.zeros((len(time), 3))
v = np.zeros((len(time), 3))
q[0] = desired[0]

for k in range(len(time) - 1):
    u = np.clip([
        controllers[j].update(desired[k, j], q[k, j], desired_v[k, j],
                              desired_a[k, j], v[k, j], dt)
        for j in range(3)
    ], force_min, force_max)
    q[k + 1], v[k + 1] = robot.rk4_step(q[k], v[k], u, dt)

state_index = np.clip(np.searchsorted(plan_time, time, side="right") - 1,
                      0, len(plan_states) - 1)
states = [plan_states[i] for i in state_index]


def wafer_location(state, fork_z):
    # Picked up once the fork rises to the wafer, set down once it drops below
    if state == "PICK":
        return "pick"
    if state == "LIFT":
        return "fork" if fork_z >= pick.z else "pick"
    if state == "TRANSFER":
        return "fork"
    if state == "PLACE":
        return "fork" if fork_z >= place.z else "place"
    return "place"


#  Geometry helpers

def box(center, size, yaw=0.0):
    # Faces of a box rotated by yaw about the world z axis through the origin
    cx, cy, cz = center
    sx, sy, sz = np.array(size) / 2
    corners = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    corners += [cx, cy, cz]
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    p = corners @ rotation.T
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return [p[f] for f in faces]


def cylinder(center, radius, height, n=28):
    cx, cy, cz = center
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    bottom = np.column_stack([cx + radius * np.cos(a), cy + radius * np.sin(a),
                              np.full(n, cz - height / 2)])
    top = bottom + [0, 0, height]
    sides = [[bottom[i], bottom[(i + 1) % n], top[(i + 1) % n], top[i]] for i in range(n)]
    return [bottom, top] + sides


def wedge(theta_min, theta_max, r_min, r_max, z_min, z_max, n=12):
    a = np.linspace(theta_min, theta_max, n)
    def ring(r, z):
        return np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n, z)])
    faces = []
    for z in (z_min, z_max):
        faces.append(np.vstack([ring(r_min, z), ring(r_max, z)[::-1]]))
    for r in (r_min, r_max):
        lo, hi = ring(r, z_min), ring(r, z_max)
        faces += [[lo[i], lo[i + 1], hi[i + 1], hi[i]] for i in range(n - 1)]
    for ang in (theta_min, theta_max):
        c, s = np.cos(ang), np.sin(ang)
        faces.append([[r_min * c, r_min * s, z_min], [r_max * c, r_max * s, z_min],
                      [r_max * c, r_max * s, z_max], [r_min * c, r_min * s, z_max]])
    return faces


def rotate_offset(theta, x, y):
    # Point (x, y) in the arm frame -> world
    return x * np.cos(theta) - y * np.sin(theta), x * np.sin(theta) + y * np.cos(theta)


#  Figure

figure = plt.figure(figsize=(7.2, 5.4), dpi=100, facecolor=COLORS["background"])
axis = figure.add_axes([0, 0, 1, 1], projection="3d", facecolor=COLORS["background"])


def scene_faces(r, theta, z, location):

    # Every solid in one list, so matplotlib depth-sorts them together
    faces, colors = [], []

    def add(polys, color, alpha=1.0):
        faces.extend(polys)
        colors.extend([(*color, alpha)] * len(polys))

    # Stations: base plate + four support pins
    for station in (pick, place):
        add(box((station.r, 0, station.z - 0.030), (0.24, 0.24, 0.012), station.theta),
            COLORS["plate"])
        for px, py in [(0.055, 0.065), (0.055, -0.065), (-0.055, 0.065), (-0.055, -0.065)]:
            add(box((station.r + px, py, station.z - 0.012), (0.008, 0.008, 0.024),
                    station.theta), COLORS["pin"])

    # Fixed base
    add(cylinder((0, 0, 0.006), 0.115, 0.012), COLORS["graphite"])
    add(cylinder((0, 0, 0.029), 0.088, 0.034), COLORS["housing"])

    # Rotating column, Z carriage, telescoping arm and fork (arm frame -> theta)
    add(box((-0.050, 0, 0.215), (0.050, 0.090, 0.310), theta), COLORS["housing"])
    add(box((-0.050, 0, 0.375), (0.058, 0.098, 0.012), theta), COLORS["graphite"])
    add(box((-0.012, 0, z - 0.004), (0.030, 0.100, 0.060), theta), COLORS["accent"])
    add(box((0.045, 0, z - 0.020), (0.090, 0.056, 0.020), theta), COLORS["housing"])
    add(box((r - 0.115, 0, z - 0.012), (0.150, 0.040, 0.012), theta), COLORS["aluminium"])
    add(box((r - 0.042, 0, z - 0.006), (0.024, 0.074, 0.012), theta), COLORS["graphite"])
    for side in (0.024, -0.024):
        add(box((r + 0.020, side, z - 0.0015), (0.100, 0.014, 0.003), theta),
            COLORS["ceramic"])

    # Wafer
    if location == "fork":
        wx, wy, wz = r * np.cos(theta), r * np.sin(theta), z + 0.003
    else:
        station = pick if location == "pick" else place
        wx, wy, wz = (station.r * np.cos(station.theta),
                      station.r * np.sin(station.theta), station.z + 0.0015)
    # Slightly see-through, so the fork reads as underneath it
    add(cylinder((wx, wy, wz), WAFER_RADIUS, 0.003, n=36), COLORS["wafer"], alpha=0.8)

    # Forbidden zone (translucent)
    for zone in safety.forbidden_zones:
        add(wedge(zone.theta_min, zone.theta_max, zone.r_min, zone.r_max,
                  zone.z_min, zone.z_max), COLORS["zone"], alpha=0.16)

    return faces, colors


def draw_static():

    # Floor grid
    for g in np.arange(-0.3, 0.61, 0.05):
        axis.plot([g, g], [-0.3, 0.6], [0, 0], color=(0.35, 0.37, 0.42), lw=0.3, alpha=0.5)
        axis.plot([-0.3, 0.6], [g, g], [0, 0], color=(0.35, 0.37, 0.42), lw=0.3, alpha=0.5)

    for zone in safety.forbidden_zones:
        mid, rad = (zone.theta_min + zone.theta_max) / 2, (zone.r_min + zone.r_max) / 2
        axis.text(rad * np.cos(mid), rad * np.sin(mid), zone.z_max + 0.04,
                  "FORBIDDEN\n" + zone.name, color=(1.0, 0.55, 0.55), fontsize=8,
                  ha="center")

    for station in (pick, place):
        x, y = station.r * np.cos(station.theta), station.r * np.sin(station.theta)
        axis.text(x, y, station.z + 0.07, station.name.replace(" ", "\n"),
                  color=(0.88, 0.88, 0.88), fontsize=8, ha="center")


dynamic = []


def draw_frame(index):

    for artist in dynamic:
        artist.remove()
    dynamic.clear()

    k = frame_samples[index]
    r, theta, z = q[k]
    state = states[k]
    if index < hold_frames:
        state = "IDLE"
    if index >= len(frame_samples) - after_frames:
        state = "COMPLETE"

    location = wafer_location(state if state != "IDLE" else "PICK", z)
    if state == "COMPLETE":
        location = "place"

    faces, colors = scene_faces(r, theta, z, location)
    solids = Poly3DCollection(faces, facecolors=colors,
                              edgecolors=[(c[0] * 0.8, c[1] * 0.8, c[2] * 0.8, c[3])
                                          for c in colors],
                              linewidths=0.25)
    axis.add_collection3d(solids)
    dynamic.append(solids)

    # Arm-tip trail
    past = q[: k + 1: 20]
    trail, = axis.plot(past[:, 0] * np.cos(past[:, 1]), past[:, 0] * np.sin(past[:, 1]),
                       past[:, 2], color=COLORS["trail"], lw=1.2, alpha=0.9)
    dynamic.append(trail)

    held = "wafer on fork" if location == "fork" else f"wafer at {location}"
    label = figure.text(0.03, 0.94, f"{state}", color="white", fontsize=15,
                        fontweight="bold")
    detail = figure.text(0.03, 0.895,
                         f"t = {time[k]:4.1f} s   |   {held}   |   "
                         f"r = {r * 1000:3.0f} mm   θ = {np.degrees(theta):3.0f}°   "
                         f"z = {z * 1000:3.0f} mm",
                         color=(0.8, 0.82, 0.86), fontsize=9)
    footer = figure.text(0.03, 0.03,
                         "R-θ-Z wafer-transfer robot · simulation (PID + feedforward) · "
                         "inspired by semiconductor wafer handlers",
                         color=(0.55, 0.57, 0.62), fontsize=7.5)
    dynamic.extend([label, detail, footer])

    return dynamic


axis.set_xlim(-0.25, 0.55)
axis.set_ylim(-0.25, 0.55)
axis.set_zlim(0.0, 0.40)
axis.set_box_aspect((1, 1, 0.5))
axis.view_init(elev=30, azim=-62)
axis.set_axis_off()
draw_static()

# Frame schedule: short hold, the motion at real speed, short hold
step = int(round(1 / (FPS * dt)))
motion_samples = list(range(0, len(time), step)) + [len(time) - 1]
hold_frames = int(HOLD_BEFORE * FPS)
after_frames = int(HOLD_AFTER * FPS)
frame_samples = [0] * hold_frames + motion_samples + [len(time) - 1] * after_frames

animation = FuncAnimation(figure, draw_frame, frames=len(frame_samples), blit=False)

os.makedirs("results", exist_ok=True)
output = "results/wafer_transfer_animation.gif"
animation.save(output, writer=PillowWriter(fps=FPS))

print(f"Saved {output} ({len(frame_samples)} frames, "
      f"{os.path.getsize(output) / 1e6:.1f} MB)")
