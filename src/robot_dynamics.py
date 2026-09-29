import numpy as np


class CylindricalRobotDynamics:

    def __init__(
        self,
        radial_mass,
        radial_damping,
        rotational_inertia,
        rotational_damping,
        vertical_mass,
        vertical_damping,
        gravity=9.81
    ):

        self.radial_mass = radial_mass
        self.radial_damping = radial_damping

        self.rotational_inertia = rotational_inertia
        self.rotational_damping = rotational_damping

        self.vertical_mass = vertical_mass
        self.vertical_damping = vertical_damping

        self.gravity = gravity

    def radial_acceleration(self, force, velocity):

        acceleration = (
            force - self.radial_damping * velocity
        ) / self.radial_mass

        return acceleration

    def rotational_acceleration(self, torque, angular_velocity):

        acceleration = (
            torque - self.rotational_damping * angular_velocity
        ) / self.rotational_inertia

        return acceleration

    def vertical_acceleration(self, force, velocity):

        acceleration = (
            force
            - self.vertical_damping * velocity
            - self.vertical_mass * self.gravity
        ) / self.vertical_mass

        return acceleration

    def state_derivative(
        self,
        position,
        velocity,
        radial_force,
        rotational_torque,
        vertical_force
    ):

        radial_acceleration = self.radial_acceleration(
            radial_force,
            velocity[0]
        )

        rotational_acceleration = self.rotational_acceleration(
            rotational_torque,
            velocity[1]
        )

        vertical_acceleration = self.vertical_acceleration(
            vertical_force,
            velocity[2]
        )

        acceleration = np.array([
            radial_acceleration,
            rotational_acceleration,
            vertical_acceleration
        ])

        return np.concatenate((velocity, acceleration))

    def rk4_step(self, position, velocity, force, dt):

        # Advance one time step with the force held constant (zero-order
        # hold), using fourth-order Runge-Kutta. This is what Simulink's
        # fixed-step ode4 solver does, so results match the Simulink model.

        def derivative(state):
            return self.state_derivative(state[:3], state[3:], *force)

        state = np.concatenate((position, velocity))

        k1 = derivative(state)
        k2 = derivative(state + dt / 2 * k1)
        k3 = derivative(state + dt / 2 * k2)
        k4 = derivative(state + dt * k3)

        state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        return state[:3], state[3:]