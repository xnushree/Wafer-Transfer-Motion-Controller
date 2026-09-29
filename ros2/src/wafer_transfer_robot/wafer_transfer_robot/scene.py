import numpy as np


def state_at(t, times, states):

    # Sequence state at time t (seconds from the start of the plan)
    if t < 0:
        return "IDLE"

    if t > times[-1]:
        return "COMPLETE"

    index = np.searchsorted(times, t, side="right") - 1

    return states[max(index, 0)]


def wafer_location(state, fork_z, pick_z, place_z):

    # Where the wafer is: "pick", "fork" or "place".
    # During LIFT the fork picks the wafer up once it rises to the wafer's
    # height; during PLACE it leaves the wafer once it drops below it.
    if state in ("IDLE", "PICK", "FAULT"):
        return "pick"

    if state == "LIFT":
        return "fork" if fork_z >= pick_z else "pick"

    if state == "TRANSFER":
        return "fork"

    if state == "PLACE":
        return "fork" if fork_z >= place_z else "place"

    return "place"


def wedge_triangles(theta_min, theta_max, r_min, r_max, z_min, z_max, steps=16):

    # Triangle mesh of a cylindrical wedge (for drawing forbidden zones)
    angles = np.linspace(theta_min, theta_max, steps + 1)

    def point(r, a, z):
        return (r * np.cos(a), r * np.sin(a), z)

    triangles = []

    for a0, a1 in zip(angles[:-1], angles[1:]):
        for z in (z_min, z_max):
            triangles += [
                point(r_min, a0, z), point(r_max, a0, z), point(r_max, a1, z),
                point(r_min, a0, z), point(r_max, a1, z), point(r_min, a1, z),
            ]
        for r in (r_min, r_max):
            triangles += [
                point(r, a0, z_min), point(r, a1, z_min), point(r, a1, z_max),
                point(r, a0, z_min), point(r, a1, z_max), point(r, a0, z_max),
            ]

    for a in (theta_min, theta_max):
        triangles += [
            point(r_min, a, z_min), point(r_max, a, z_min), point(r_max, a, z_max),
            point(r_min, a, z_min), point(r_max, a, z_max), point(r_min, a, z_max),
        ]

    return triangles


def wedge_edges(theta_min, theta_max, r_min, r_max, z_min, z_max, steps=16):

    # Outline of a cylindrical wedge as line segments (pairs of points)
    angles = np.linspace(theta_min, theta_max, steps + 1)

    def point(r, a, z):
        return (r * np.cos(a), r * np.sin(a), z)

    lines = []

    for z in (z_min, z_max):
        for r in (r_min, r_max):
            for a0, a1 in zip(angles[:-1], angles[1:]):
                lines += [point(r, a0, z), point(r, a1, z)]
        for a in (theta_min, theta_max):
            lines += [point(r_min, a, z), point(r_max, a, z)]

    for r in (r_min, r_max):
        for a in (theta_min, theta_max):
            lines += [point(r, a, z_min), point(r, a, z_max)]

    return lines
