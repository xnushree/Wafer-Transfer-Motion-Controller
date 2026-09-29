import numpy as np

from src.computed_torque_controller import ComputedTorqueController
from src.coupled_dynamics import CoupledCylindricalDynamics
from src.robot_dynamics import CylindricalRobotDynamics
from src.trajectory import quintic_trajectory


def create_model(coupled=True, damping=True):

    b = 1.0 if damping else 0.0

    return CoupledCylindricalDynamics(
        radial_mass=5.0,
        radial_damping=b,
        turret_inertia=0.5,
        rotational_damping=0.1 * b,
        vertical_mass=4.0,
        vertical_damping=b,
        coupled=coupled
    )


def test_inertia_grows_with_reach():
    model = create_model()

    near = model.mass_matrix([0.1, 0.0, 0.1])[1, 1]
    far = model.mass_matrix([0.4, 0.0, 0.1])[1, 1]

    assert np.isclose(near, 0.5 + 5.0 * 0.1 ** 2)
    assert np.isclose(far, 0.5 + 5.0 * 0.4 ** 2)


def test_uncoupled_model_matches_decoupled_dynamics():
    coupled_off = create_model(coupled=False)
    decoupled = CylindricalRobotDynamics(5.0, 1.0, 0.5, 0.1, 4.0, 1.0)

    position = np.array([0.3, 0.7, 0.2])
    velocity = np.array([0.2, 1.1, -0.1])
    force = np.array([3.0, 1.5, 40.0])

    expected = decoupled.state_derivative(position, velocity, *force)[3:]

    assert np.allclose(coupled_off.acceleration(position, velocity, force), expected)


def test_spinning_arm_is_pushed_outwards():
    # Centrifugal effect: with no force, a rotating arm accelerates outwards
    model = create_model()

    acceleration = model.acceleration(
        [0.3, 0.0, 0.1],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, model.vertical_mass * model.gravity]
    )

    assert np.isclose(acceleration[0], 0.3 * 2.0 ** 2)


def test_angular_momentum_is_conserved_without_torque():
    # (J_0 + m r^2) theta_dot stays constant when the arm extends freely
    model = create_model(damping=False)

    position = np.array([0.1, 0.0, 0.1])
    velocity = np.array([0.3, 2.0, 0.0])
    force = [0.0, 0.0, model.vertical_mass * model.gravity]

    def momentum(q, v):
        return model.mass_matrix(q)[1, 1] * v[1]

    start = momentum(position, velocity)

    for _ in range(500):
        position, velocity = model.rk4_step(position, velocity, force, 0.002)

    assert position[0] > 0.3
    assert np.isclose(momentum(position, velocity), start, rtol=1e-8)


def test_computed_torque_tracks_exact_model_closely():
    model = create_model()
    gains = [(60.0, 4.0, 6.0), (200.0, 10.0, 20.0), (75.0, 5.0, 7.5)]
    controller = ComputedTorqueController(model, gains)

    dt = 0.002
    n = 1000
    moves = [(0.1, 0.4), (0.0, np.pi / 2), (0.1, 0.25)]
    trajectories = [quintic_trajectory(a, b, 2.0, n) for a, b in moves]
    desired = np.column_stack([t[1] for t in trajectories])
    desired_acceleration = np.column_stack([t[3] for t in trajectories])

    position, velocity = desired[0].copy(), np.zeros(3)
    worst = np.zeros(3)

    for k in range(n - 1):
        force = controller.update(
            desired[k], position, desired_acceleration[k], velocity, dt
        )
        position, velocity = model.rk4_step(position, velocity, force, dt)
        worst = np.maximum(worst, np.abs(desired[k + 1] - position))

    # Sub-0.1 mm / sub-0.01 degree while the arm extends and rotates together
    assert worst[0] < 1e-4 and worst[2] < 1e-4
    assert np.degrees(worst[1]) < 0.01
