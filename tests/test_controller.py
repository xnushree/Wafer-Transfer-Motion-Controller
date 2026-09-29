from src.controller import PIDController


def test_pid_zero_error():

    controller = PIDController(1.0, 0.1, 0.01)

    output = controller.update(
        desired=100,
        actual=100,
        dt=0.01
    )

    assert output == 0


def test_pid_positive_error():

    controller = PIDController(1.0, 0.0, 0.0)

    output = controller.update(
        desired=100,
        actual=90,
        dt=0.01
    )

    assert output == 10


def test_pid_integral_action():

    controller = PIDController(0.0, 1.0, 0.0)

    output = controller.update(
        desired=100,
        actual=90,
        dt=0.1
    )

    assert output == 1