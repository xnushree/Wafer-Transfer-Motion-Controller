# Copied from src/robot_kinematics.py by ros2/sync_core.py - do not edit here.
import numpy as np


class RThetaZRobot:

    def __init__(self, max_radius, min_z, max_z):
        self.max_radius = max_radius
        self.min_z = min_z
        self.max_z = max_z

    def forward_kinematics(self, radius, theta, z):

        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        return np.array([x, y, z])

    def check_limits(self, radius, z):

        if radius < 0 or radius > self.max_radius:
            return False

        if z < self.min_z or z > self.max_z:
            return False

        return True
    
    def inverse_kinematics(self, x, y, z):

        radius = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)

        if not self.check_limits(radius, z):
            raise ValueError("Target position is outside robot limits")

        return np.array([radius, theta, z])