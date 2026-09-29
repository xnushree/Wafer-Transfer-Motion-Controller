import numpy as np


def quintic_trajectory(start, end, duration, num_points=100):

    time = np.linspace(0, duration, num_points)

    tau = time / duration

    position = (
        start
        + (end - start)
        * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    )

    velocity = (
        (end - start) / duration
        * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    )

    acceleration = (
        (end - start) / duration**2
        * (60 * tau - 180 * tau**2 + 120 * tau**3)
    )

    return time, position, velocity, acceleration