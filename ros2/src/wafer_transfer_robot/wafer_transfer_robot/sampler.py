import numpy as np


JOINT_NAMES = ["r_joint", "theta_joint", "z_joint"]


def unique_times(times, *arrays):

    # The planner's timeline repeats the time where two moves meet.
    # ROS trajectories need strictly increasing times, so drop repeats.
    keep = np.concatenate(([True], np.diff(times) > 1e-9))

    return (times[keep],) + tuple(array[keep] for array in arrays)


class TrajectorySampler:

    # Reference position, velocity and acceleration at any time along a
    # planned trajectory. Before the start and after the end it returns
    # the first / last point at rest.

    def __init__(self, times, positions, velocities, accelerations):

        self.times = np.asarray(times, dtype=float)
        self.positions = np.asarray(positions, dtype=float)
        self.velocities = np.asarray(velocities, dtype=float)
        self.accelerations = np.asarray(accelerations, dtype=float)

    @property
    def duration(self):

        return self.times[-1]

    def sample(self, t):

        if t <= self.times[0]:
            return self.positions[0].copy(), np.zeros(3), np.zeros(3)

        if t >= self.times[-1]:
            return self.positions[-1].copy(), np.zeros(3), np.zeros(3)

        def interpolate(values):
            return np.array([
                np.interp(t, self.times, values[:, j]) for j in range(3)
            ])

        return (
            interpolate(self.positions),
            interpolate(self.velocities),
            interpolate(self.accelerations),
        )


class ControlledStop:

    # Brings a moving robot to rest ALONG its planned path (which the safety
    # layer already checked) instead of freezing the reference, which would
    # make the PID brake as hard as it can, overshoot and come back.
    #
    # Plan time tau keeps advancing, but at a rate that eases smoothly from
    # 1 to 0 over stop_time (rate = 1 - smoothstep). Easing in avoids a
    # sudden jump in deceleration. The peak braking deceleration is
    # 1.5 * v / stop_time, so stop_time is chosen to keep it below each
    # joint's acceleration limit divided by the margin.

    def __init__(self, sampler, plan_time, acceleration_max, margin=1.5):

        _, velocity, _ = sampler.sample(plan_time)

        self.sampler = sampler
        self.plan_time = plan_time
        self.stop_time = max(
            1.5 * margin * np.max(np.abs(velocity) / np.asarray(acceleration_max)),
            1e-3
        )

    def sample(self, elapsed):

        T = self.stop_time
        u = min(max(elapsed, 0.0), T) / T

        rate = 1.0 - (3 * u ** 2 - 2 * u ** 3)
        rate_derivative = -(6 * u - 6 * u ** 2) / T

        tau = self.plan_time + T * (u - u ** 3 + u ** 4 / 2)
        position, velocity, acceleration = self.sampler.sample(tau)

        if u >= 1.0:
            return position, np.zeros(3), np.zeros(3)

        return (
            position,
            velocity * rate,
            acceleration * rate ** 2 + velocity * rate_derivative,
        )
