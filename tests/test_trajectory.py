import numpy as np

from src.trajectory import quintic_trajectory


def test_trajectory_start_and_end():

    time, position, velocity, acceleration = quintic_trajectory(
        100,
        400,
        2,
        100
    )

    assert np.isclose(position[0], 100)
    assert np.isclose(position[-1], 400)


def test_trajectory_starts_and_ends_at_zero_velocity():

    time, position, velocity, acceleration = quintic_trajectory(
        100,
        400,
        2,
        100
    )

    assert np.isclose(velocity[0], 0)
    assert np.isclose(velocity[-1], 0)


def test_trajectory_starts_and_ends_at_zero_acceleration():

    time, position, velocity, acceleration = quintic_trajectory(
        100,
        400,
        2,
        100
    )

    assert np.isclose(acceleration[0], 0)
    assert np.isclose(acceleration[-1], 0)