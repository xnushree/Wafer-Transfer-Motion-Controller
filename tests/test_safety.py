import numpy as np

from src.safety import SafetyMonitor
from src.trajectory import quintic_trajectory


def reference_move(duration):

    moves = [(0.1, 0.4), (0.0, np.pi / 2), (0.1, 0.25)]

    trajectories = [
        quintic_trajectory(start, end, duration, 1000)
        for start, end in moves
    ]

    position = np.column_stack([trajectory[1] for trajectory in trajectories])
    velocity = np.column_stack([trajectory[2] for trajectory in trajectories])
    acceleration = np.column_stack([trajectory[3] for trajectory in trajectories])

    return position, velocity, acceleration


def test_safe_state_has_no_violations():
    monitor = SafetyMonitor()

    assert monitor.is_safe([0.2, 0.5, 0.15], [0.1, 1.0, 0.1], [0.5, 2.0, 0.2])


def test_position_outside_workspace_is_detected():
    monitor = SafetyMonitor()

    violations = monitor.check_state([0.5, 0.0, 0.15])

    assert len(violations) == 1
    assert "r position" in violations[0]


def test_velocity_limit_is_detected():
    monitor = SafetyMonitor()

    violations = monitor.check_state([0.2, 0.0, 0.15], velocity=[0.0, -2.5, 0.0])

    assert len(violations) == 1
    assert "theta velocity" in violations[0]


def test_acceleration_limit_is_detected():
    monitor = SafetyMonitor()

    violations = monitor.check_state(
        [0.2, 0.0, 0.15],
        acceleration=[0.0, 0.0, 0.6]
    )

    assert len(violations) == 1
    assert "z acceleration" in violations[0]


def test_clamp_position_pulls_command_inside_limits():
    monitor = SafetyMonitor()

    clamped = monitor.clamp_position([0.5, 4.0, 0.0])

    assert np.allclose(clamped, [0.45, np.pi, 0.05])


def test_reference_move_is_safe():
    monitor = SafetyMonitor()

    index, violations = monitor.check_trajectory(*reference_move(2.0))

    assert index is None
    assert violations == []


def test_rushed_move_is_rejected():
    monitor = SafetyMonitor()

    index, violations = monitor.check_trajectory(*reference_move(0.5))

    assert index is not None
    assert any("acceleration" in violation for violation in violations)


def test_position_inside_forbidden_zone_is_detected():
    monitor = SafetyMonitor()

    violations = monitor.check_state([0.3, np.radians(135), 0.15])

    assert violations == ["inside forbidden zone 'pillar'"]


def test_short_arm_passes_under_forbidden_zone():
    monitor = SafetyMonitor()

    assert monitor.is_safe([0.1, np.radians(135), 0.15])


def test_move_through_forbidden_zone_is_rejected():
    monitor = SafetyMonitor()

    time, theta, theta_velocity, theta_acceleration = quintic_trajectory(
        0.0,
        np.radians(170),
        3.0,
        1000
    )

    position = np.column_stack([
        np.full_like(theta, 0.3),
        theta,
        np.full_like(theta, 0.15)
    ])
    velocity = np.column_stack([
        np.zeros_like(theta),
        theta_velocity,
        np.zeros_like(theta)
    ])
    acceleration = np.column_stack([
        np.zeros_like(theta),
        theta_acceleration,
        np.zeros_like(theta)
    ])

    index, violations = monitor.check_trajectory(position, velocity, acceleration)

    assert index is not None
    assert np.degrees(theta[index]) >= 120
    assert "inside forbidden zone 'pillar'" in violations
