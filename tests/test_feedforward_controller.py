import numpy as np

from src.feedforward_controller import FeedforwardController


def test_feedforward_zero_motion():

    controller = FeedforwardController(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        mass=5.0,
        damping=1.0
    )

    output = controller.update(
        desired_position=1.0,
        actual_position=1.0,
        desired_velocity=0.0,
        desired_acceleration=0.0,
        actual_velocity=0.0,
        dt=0.01
    )

    assert output == 0.0


def test_feedforward_acceleration():

    controller = FeedforwardController(
        kp=0.0,
        ki=0.0,
        kd=0.0,
        mass=5.0,
        damping=1.0
    )

    output = controller.update(
        desired_position=1.0,
        actual_position=1.0,
        desired_velocity=0.0,
        desired_acceleration=2.0,
        actual_velocity=0.0,
        dt=0.01
    )

    assert output == 10.0


def test_feedforward_gravity():

    controller = FeedforwardController(
        kp=0.0,
        ki=0.0,
        kd=0.0,
        mass=4.0,
        damping=1.0,
        gravity=9.81
    )

    output = controller.update(
        desired_position=1.0,
        actual_position=1.0,
        desired_velocity=0.0,
        desired_acceleration=0.0,
        actual_velocity=0.0,
        dt=0.01
    )

    assert np.isclose(
        output,
        4.0 * 9.81
    )