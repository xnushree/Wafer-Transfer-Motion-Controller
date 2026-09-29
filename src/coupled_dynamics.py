import numpy as np


class CoupledCylindricalDynamics:

    # Rigid-body dynamics of an R-theta-Z arm in the form
    #
    #     M(q) q_ddot + C(q, q_dot) q_dot + B q_dot + G = tau,   q = [r, theta, z]
    #
    # The radial stage is modelled as a point mass m_r at radius r, so it
    # adds m_r * r^2 to the inertia about the theta axis, and rotation and
    # extension couple through centrifugal and Coriolis terms:
    #
    #     m_r (r_ddot - r theta_dot^2)                    + b_r r_dot       = F_r
    #     (J_0 + m_r r^2) theta_ddot + 2 m_r r r_dot theta_dot + b_theta theta_dot = T_theta
    #     m_z z_ddot                                      + b_z z_dot + m_z g = F_z
    #
    # J_0 is the inertia of the turret and column alone. The point-mass
    # assumption is an upper bound on the coupling (a real arm spreads its
    # mass along its length). With coupled=False the model reduces to the
    # decoupled CylindricalRobotDynamics with rotational inertia J_0.

    def __init__(
        self,
        radial_mass,
        radial_damping,
        turret_inertia,
        rotational_damping,
        vertical_mass,
        vertical_damping,
        gravity=9.81,
        coupled=True
    ):

        self.radial_mass = radial_mass
        self.turret_inertia = turret_inertia
        self.vertical_mass = vertical_mass
        self.gravity = gravity
        self.coupled = coupled

        self.damping = np.array([radial_damping, rotational_damping, vertical_damping])

    def mass_matrix(self, position):

        r = position[0]
        rotational_inertia = self.turret_inertia

        if self.coupled:
            rotational_inertia += self.radial_mass * r ** 2

        return np.diag([self.radial_mass, rotational_inertia, self.vertical_mass])

    def coriolis_centrifugal(self, position, velocity):

        # The C(q, q_dot) q_dot vector
        if not self.coupled:
            return np.zeros(3)

        r = position[0]
        r_dot, theta_dot = velocity[0], velocity[1]

        return np.array([
            -self.radial_mass * r * theta_dot ** 2,
            2.0 * self.radial_mass * r * r_dot * theta_dot,
            0.0,
        ])

    def gravity_vector(self):

        return np.array([0.0, 0.0, self.vertical_mass * self.gravity])

    def acceleration(self, position, velocity, force):

        rhs = (
            np.asarray(force, dtype=float)
            - self.coriolis_centrifugal(position, velocity)
            - self.damping * velocity
            - self.gravity_vector()
        )

        # M is diagonal, so solving M q_ddot = rhs is an element-wise divide
        return rhs / np.diag(self.mass_matrix(position))

    def rk4_step(self, position, velocity, force, dt):

        # One step with the force held constant (zero-order hold)
        def derivative(state):
            return np.concatenate((state[3:], self.acceleration(state[:3], state[3:], force)))

        state = np.concatenate((position, velocity))

        k1 = derivative(state)
        k2 = derivative(state + dt / 2 * k1)
        k3 = derivative(state + dt / 2 * k2)
        k4 = derivative(state + dt * k3)

        state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        return state[:3], state[3:]
