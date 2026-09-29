# Copied from src/feedforward_controller.py by ros2/sync_core.py - do not edit here.
from wafer_transfer_robot.core.controller import PIDController


class FeedforwardController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        mass,
        damping,
        gravity=0.0
    ):

        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd
        )

        self.mass = mass
        self.damping = damping
        self.gravity = gravity

    def update(
        self,
        desired_position,
        actual_position,
        desired_velocity,
        desired_acceleration,
        actual_velocity,
        dt
    ):

        feedback = self.pid.update(
            desired_position,
            actual_position,
            dt
        )

        feedforward = (
            self.mass * desired_acceleration
            + self.damping * desired_velocity
            + self.mass * self.gravity
        )

        output = feedback + feedforward

        return output