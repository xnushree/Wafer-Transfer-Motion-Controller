import os
import sys

import numpy as np

# The ROS package's pure-Python helpers (no rclpy needed)
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "ros2", "src", "wafer_transfer_robot"
    )
)

from wafer_transfer_robot.sampler import (  # noqa: E402
    ControlledStop,
    TrajectorySampler,
    unique_times,
)
from wafer_transfer_robot.scene import state_at, wafer_location  # noqa: E402

from src.trajectory import quintic_trajectory  # noqa: E402


def straight_move():

    time, r, r_velocity, r_acceleration = quintic_trajectory(0.1, 0.4, 1.6, 400)

    zeros = np.zeros_like(r)

    return TrajectorySampler(
        time,
        np.column_stack([r, zeros, np.full_like(r, 0.1)]),
        np.column_stack([r_velocity, zeros, zeros]),
        np.column_stack([r_acceleration, zeros, zeros])
    )


def test_unique_times_drops_repeated_boundary_samples():
    times = np.array([0.0, 0.5, 1.0, 1.0, 1.5])
    values = np.arange(5)

    kept_times, kept_values = unique_times(times, values)

    assert np.all(np.diff(kept_times) > 0)
    assert list(kept_values) == [0, 1, 2, 4]


def test_sampler_holds_end_points_at_rest():
    sampler = straight_move()

    before = sampler.sample(-1.0)
    after = sampler.sample(10.0)

    assert np.allclose(before[0], [0.1, 0.0, 0.1])
    assert np.allclose(after[0], [0.4, 0.0, 0.1])
    assert np.allclose(before[1], 0) and np.allclose(after[2], 0)


def test_controlled_stop_stays_on_path_within_acceleration_limit():
    sampler = straight_move()
    acceleration_max = np.array([1.0, 4.0, 0.5])

    stop = ControlledStop(sampler, 0.8, acceleration_max)

    elapsed = np.linspace(0.0, stop.stop_time + 0.5, 2000)
    samples = [stop.sample(e) for e in elapsed]
    position = np.array([s[0] for s in samples])
    velocity = np.array([s[1] for s in samples])
    acceleration = np.array([s[2] for s in samples])

    # Moves forward only, along r (the planned path), and comes to rest
    assert np.all(np.diff(position[:, 0]) >= -1e-12)
    assert np.allclose(position[:, 1:], [0.0, 0.1])
    assert np.allclose(velocity[-1], 0)

    assert np.max(np.abs(acceleration[:, 0])) <= acceleration_max[0]


def test_state_at_follows_timeline():
    times = np.array([0.0, 1.0, 2.0, 3.0])
    states = ["PICK", "LIFT", "TRANSFER", "TRANSFER"]

    assert state_at(-0.1, times, states) == "IDLE"
    assert state_at(0.5, times, states) == "PICK"
    assert state_at(1.5, times, states) == "LIFT"
    assert state_at(3.5, times, states) == "COMPLETE"


def test_wafer_moves_with_fork_only_between_lift_and_place():
    assert wafer_location("PICK", 0.09, 0.10, 0.25) == "pick"
    assert wafer_location("LIFT", 0.095, 0.10, 0.25) == "pick"
    assert wafer_location("LIFT", 0.11, 0.10, 0.25) == "fork"
    assert wafer_location("TRANSFER", 0.2, 0.10, 0.25) == "fork"
    assert wafer_location("PLACE", 0.26, 0.10, 0.25) == "fork"
    assert wafer_location("PLACE", 0.245, 0.10, 0.25) == "place"
    assert wafer_location("RETRACT", 0.24, 0.10, 0.25) == "place"
