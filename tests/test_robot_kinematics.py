import numpy as np

from src.robot_kinematics import RThetaZRobot


def test_forward_kinematics():
    robot = RThetaZRobot(500, 0, 300)

    position = robot.forward_kinematics(300, np.pi / 2, 100)

    assert np.allclose(position, [0, 300, 100])


def test_inverse_kinematics():
    robot = RThetaZRobot(500, 0, 300)

    joints = robot.inverse_kinematics(0, 300, 100)

    assert np.allclose(joints, [300, np.pi / 2, 100])


def test_forward_inverse_round_trip():
    robot = RThetaZRobot(500, 0, 300)

    joints = [300, np.pi / 2, 100]

    position = robot.forward_kinematics(*joints)

    calculated_joints = robot.inverse_kinematics(*position)

    assert np.allclose(calculated_joints, joints)


def test_radius_limit():
    robot = RThetaZRobot(500, 0, 300)

    try:
        robot.inverse_kinematics(600, 0, 100)
        assert False
    except ValueError:
        assert True


def test_z_limit():
    robot = RThetaZRobot(500, 0, 300)

    try:
        robot.inverse_kinematics(300, 0, 400)
        assert False
    except ValueError:
        assert True