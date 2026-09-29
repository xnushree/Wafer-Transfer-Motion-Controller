import numpy as np


class JointLimits:

    def __init__(
        self,
        name,
        position_min,
        position_max,
        velocity_max,
        acceleration_max
    ):

        self.name = name
        self.position_min = position_min
        self.position_max = position_max
        self.velocity_max = velocity_max
        self.acceleration_max = acceleration_max


def default_joint_limits():

    # Order matches q = [r, theta, z]. SI units: m, rad, m/s, rad/s, m/s^2, rad/s^2
    return [
        JointLimits("r", 0.05, 0.45, 0.5, 1.0),
        JointLimits("theta", -np.pi, np.pi, 2.0, 4.0),
        JointLimits("z", 0.05, 0.30, 0.2, 0.5),
    ]


class ForbiddenZone:

    # A wedge-shaped region in cylindrical coordinates, e.g. a pillar or a
    # neighbouring station. The arm tip must never be inside it.

    def __init__(
        self,
        name,
        theta_min,
        theta_max,
        r_min,
        r_max,
        z_min,
        z_max
    ):

        self.name = name
        self.theta_min = theta_min
        self.theta_max = theta_max
        self.r_min = r_min
        self.r_max = r_max
        self.z_min = z_min
        self.z_max = z_max

    def contains(self, position):

        r, theta, z = position

        # Wrap theta into [-pi, pi] so e.g. 2*pi + 0.1 is treated as 0.1
        theta = np.arctan2(np.sin(theta), np.cos(theta))

        return (
            self.theta_min <= theta <= self.theta_max
            and self.r_min <= r <= self.r_max
            and self.z_min <= z <= self.z_max
        )


def default_forbidden_zones():

    # Example obstacle between 120 and 150 degrees, clear of the reference move
    return [
        ForbiddenZone(
            "pillar",
            np.radians(120),
            np.radians(150),
            0.20,
            0.45,
            0.0,
            0.30
        ),
    ]


class SafetyMonitor:

    def __init__(self, joint_limits=None, forbidden_zones=None):

        if joint_limits is None:
            joint_limits = default_joint_limits()

        if forbidden_zones is None:
            forbidden_zones = default_forbidden_zones()

        self.joint_limits = joint_limits
        self.forbidden_zones = forbidden_zones

    def check_state(self, position, velocity=None, acceleration=None):

        violations = []

        for i, limits in enumerate(self.joint_limits):

            if (
                position[i] < limits.position_min
                or position[i] > limits.position_max
            ):
                violations.append(
                    f"{limits.name} position {position[i]:.4f} outside "
                    f"[{limits.position_min:.4f}, {limits.position_max:.4f}]"
                )

            if velocity is not None and abs(velocity[i]) > limits.velocity_max:
                violations.append(
                    f"{limits.name} velocity {velocity[i]:.4f} exceeds "
                    f"{limits.velocity_max:.4f}"
                )

            if (
                acceleration is not None
                and abs(acceleration[i]) > limits.acceleration_max
            ):
                violations.append(
                    f"{limits.name} acceleration {acceleration[i]:.4f} exceeds "
                    f"{limits.acceleration_max:.4f}"
                )

        for zone in self.forbidden_zones:

            if zone.contains(position):
                violations.append(f"inside forbidden zone '{zone.name}'")

        return violations

    def is_safe(self, position, velocity=None, acceleration=None):

        return len(self.check_state(position, velocity, acceleration)) == 0

    def clamp_position(self, position):

        lower = np.array([limits.position_min for limits in self.joint_limits])
        upper = np.array([limits.position_max for limits in self.joint_limits])

        return np.clip(position, lower, upper)

    def check_trajectory(self, positions, velocities, accelerations):

        for k in range(len(positions)):

            violations = self.check_state(
                positions[k],
                velocities[k],
                accelerations[k]
            )

            if violations:
                return k, violations

        return None, []
