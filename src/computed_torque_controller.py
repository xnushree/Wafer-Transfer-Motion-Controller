import numpy as np

from src.controller import PIDController


class ComputedTorqueController:

    # Model-based ("computed torque" / inverse dynamics) control:
    #
    #     a   = q_ddot_desired + PID(q_desired - q)
    #     tau = M(q) a + C(q, q_dot) q_dot + B q_dot + G
    #
    # If the model is exact, the closed loop becomes three independent
    # double integrators driven by the PID correction, whatever the arm's
    # reach or speed. The PID gains are therefore per unit inertia
    # (accelerations, not forces).

    def __init__(self, model, gains):

        # gains: [(kp, ki, kd) for r, theta, z], in acceleration units
        self.model = model
        self.pids = [PIDController(kp, ki, kd) for kp, ki, kd in gains]

    def update(
        self,
        desired_position,
        position,
        desired_acceleration,
        velocity,
        dt
    ):

        correction = np.array([
            pid.update(desired_position[j], position[j], dt)
            for j, pid in enumerate(self.pids)
        ])

        acceleration_command = np.asarray(desired_acceleration) + correction

        return (
            self.model.mass_matrix(position) @ acceleration_command
            + self.model.coriolis_centrifugal(position, velocity)
            + self.model.damping * np.asarray(velocity)
            + self.model.gravity_vector()
        )
