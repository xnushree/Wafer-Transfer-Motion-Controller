import numpy as np

from src.robot_dynamics import CylindricalRobotDynamics


def create_robot():

    return CylindricalRobotDynamics(
        radial_mass=5.0,
        radial_damping=1.0,
        rotational_inertia=0.5,
        rotational_damping=0.1,
        vertical_mass=4.0,
        vertical_damping=1.0
    )


def test_radial_acceleration():

    robot = create_robot()

    acceleration = robot.radial_acceleration(
        force=10.0,
        velocity=0.0
    )

    assert acceleration == 2.0


def test_radial_damping():

    robot = create_robot()

    acceleration = robot.radial_acceleration(
        force=10.0,
        velocity=2.0
    )

    assert acceleration == 1.6


def test_rotational_acceleration():

    robot = create_robot()

    acceleration = robot.rotational_acceleration(
        torque=1.0,
        angular_velocity=0.0
    )

    assert acceleration == 2.0


def test_vertical_gravity():

    robot = create_robot()

    acceleration = robot.vertical_acceleration(
        force=robot.vertical_mass * robot.gravity,
        velocity=0.0
    )

    assert np.isclose(acceleration, 0.0)


def test_state_derivative():

    robot = create_robot()

    position = np.array([
        0.2,
        0.5,
        0.1
    ])

    velocity = np.array([
        1.0,
        2.0,
        0.5
    ])

    derivative = robot.state_derivative(
        position=position,
        velocity=velocity,
        radial_force=10.0,
        rotational_torque=1.0,
        vertical_force=robot.vertical_mass * robot.gravity
    )

    assert len(derivative) == 6

    assert np.allclose(
        derivative[:3],
        velocity
    )

def test_rk4_step_matches_analytical_solution():

    # Constant 1 N on the R axis from rest: exact solution is
    # r_dot = (F/b)(1 - exp(-t/tau)), r = r0 + (F/b)(t - tau(1 - exp(-t/tau)))
    robot = create_robot()

    position = np.array([0.1, 0.0, 0.1])
    velocity = np.zeros(3)
    force = [1.0, 0.0, robot.vertical_mass * robot.gravity]

    dt = 0.002
    for _ in range(1000):
        position, velocity = robot.rk4_step(position, velocity, force, dt)

    t = 1000 * dt
    tau = robot.radial_mass / robot.radial_damping

    assert np.isclose(velocity[0], 1.0 - np.exp(-t / tau), atol=1e-10)
    assert np.isclose(
        position[0],
        0.1 + t - tau * (1.0 - np.exp(-t / tau)),
        atol=1e-10
    )
    assert np.isclose(position[2], 0.1, atol=1e-12)
