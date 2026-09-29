import numpy as np

from src.safety import SafetyMonitor
from src.wafer_transfer import Station, WaferTransferSequence


def reference_sequence(place_theta=np.pi / 2):

    pick = Station("load_port", 0.4, 0.0, 0.10)
    place = Station("process_chamber", 0.4, place_theta, 0.25)

    return WaferTransferSequence(pick, place)


def test_sequence_completes():
    sequence = reference_sequence()

    assert sequence.run()
    assert sequence.state == "COMPLETE"
    assert sequence.fault is None


def test_states_run_in_order():
    sequence = reference_sequence()
    sequence.run()

    order = []
    for segment in sequence.segments:
        if not order or order[-1] != segment["state"]:
            order.append(segment["state"])

    assert order == ["PICK", "LIFT", "TRANSFER", "PLACE", "RETRACT"]


def test_wafer_is_held_only_between_lift_and_place():
    sequence = reference_sequence()
    sequence.run()

    for segment in sequence.segments:
        expected = segment["state"] in ("TRANSFER", "PLACE")
        assert segment["wafer_held"] == expected

    assert not sequence.wafer_held


def test_moves_are_continuous_and_end_at_place_station():
    sequence = reference_sequence()
    sequence.run()

    for previous, current in zip(sequence.segments, sequence.segments[1:]):
        assert np.allclose(previous["position"][-1], current["position"][0])

    final = sequence.segments[-1]["position"][-1]

    assert np.allclose(final, [0.1, np.pi / 2, 0.24])


def test_every_move_respects_limits():
    sequence = reference_sequence()
    sequence.run()

    time, position, velocity, acceleration, states = sequence.timeline()

    index, violations = SafetyMonitor().check_trajectory(
        position,
        velocity,
        acceleration
    )

    assert index is None


def test_rotation_happens_only_when_retracted():
    sequence = reference_sequence(place_theta=np.radians(170))
    sequence.run()

    time, position, velocity, acceleration, states = sequence.timeline()

    rotating = np.abs(velocity[:, 1]) > 1e-9

    assert np.all(position[rotating, 0] <= 0.1 + 1e-9)


def test_retract_first_passes_under_pillar():
    # The place station at 170 degrees lies beyond the pillar (120-150 deg).
    # Rotating while retracted keeps the arm under it.
    sequence = reference_sequence(place_theta=np.radians(170))

    assert sequence.run()


def test_station_inside_forbidden_zone_faults():
    sequence = reference_sequence(place_theta=np.radians(135))

    assert not sequence.run()
    assert sequence.state == "FAULT"
    assert "TRANSFER" in sequence.fault
    assert "pillar" in sequence.fault
