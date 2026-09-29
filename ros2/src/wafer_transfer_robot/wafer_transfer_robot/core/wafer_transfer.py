# Copied from src/wafer_transfer.py by ros2/sync_core.py - do not edit here.
import numpy as np

from wafer_transfer_robot.core.safety import SafetyMonitor
from wafer_transfer_robot.core.trajectory import quintic_trajectory


STATES = [
    "IDLE",
    "PICK",
    "LIFT",
    "TRANSFER",
    "PLACE",
    "RETRACT",
    "COMPLETE",
]


class Station:

    # A place where a wafer rests, given in robot coordinates.
    # z is the height of the wafer's underside on the station.

    def __init__(self, name, r, theta, z):

        self.name = name
        self.r = r
        self.theta = theta
        self.z = z


class WaferTransferSequence:

    def __init__(
        self,
        pick_station,
        place_station,
        safety_monitor=None,
        start_position=(0.1, 0.0, 0.1),
        home_radius=0.1,
        approach_clearance=0.01,
        lift_height=0.02,
        time_margin=1.2,
        num_points=200
    ):

        if safety_monitor is None:
            safety_monitor = SafetyMonitor()

        self.pick = pick_station
        self.place = place_station
        self.safety = safety_monitor
        self.start_position = np.array(start_position, dtype=float)
        self.home_radius = home_radius
        self.approach_clearance = approach_clearance
        self.lift_height = lift_height
        self.time_margin = time_margin
        self.num_points = num_points

        self.state = "IDLE"
        self.wafer_held = False
        self.fault = None
        self.segments = []

    def waypoints(self):

        # Each state is a list of joint targets [r, theta, z].
        # The arm always retracts to home_radius before rotating, so it
        # swings with a short reach and passes under nearby obstacles.
        pick = self.pick
        place = self.place
        home = self.home_radius
        below = self.approach_clearance
        above = self.lift_height

        return {
            "PICK": [
                [home, pick.theta, pick.z - below],
                [pick.r, pick.theta, pick.z - below],
            ],
            "LIFT": [
                [pick.r, pick.theta, pick.z + above],
            ],
            "TRANSFER": [
                [home, pick.theta, pick.z + above],
                [home, place.theta, place.z + above],
                [place.r, place.theta, place.z + above],
            ],
            "PLACE": [
                [place.r, place.theta, place.z - below],
            ],
            "RETRACT": [
                [home, place.theta, place.z - below],
            ],
        }

    def move_duration(self, start, end):

        # Shortest quintic duration that keeps every joint inside its
        # velocity and acceleration limits, times a safety margin.
        # For a quintic: peak velocity = 1.875 * distance / T
        #                peak acceleration = 5.7735 * distance / T^2
        distance = np.abs(np.asarray(end) - np.asarray(start))

        velocity_max = np.array([
            limits.velocity_max for limits in self.safety.joint_limits
        ])
        acceleration_max = np.array([
            limits.acceleration_max for limits in self.safety.joint_limits
        ])

        time_for_velocity = 1.875 * distance / velocity_max
        time_for_acceleration = np.sqrt(5.7735 * distance / acceleration_max)

        duration = self.time_margin * max(
            time_for_velocity.max(),
            time_for_acceleration.max()
        )

        # Round up to the next 0.1 s
        return np.ceil(duration * 10) / 10

    def plan_move(self, start, end):

        duration = self.move_duration(start, end)

        joints = [
            quintic_trajectory(start[i], end[i], duration, self.num_points)
            for i in range(3)
        ]

        return {
            "time": joints[0][0],
            "position": np.column_stack([joint[1] for joint in joints]),
            "velocity": np.column_stack([joint[2] for joint in joints]),
            "acceleration": np.column_stack([joint[3] for joint in joints]),
        }

    def run(self):

        position = self.start_position.copy()
        waypoints = self.waypoints()

        for state in STATES[1:-1]:

            self.state = state

            for target in waypoints[state]:

                target = np.array(target, dtype=float)

                if np.allclose(target, position):
                    continue

                move = self.plan_move(position, target)

                index, violations = self.safety.check_trajectory(
                    move["position"],
                    move["velocity"],
                    move["acceleration"]
                )

                if violations:
                    self.fault = f"{state}: " + "; ".join(violations)
                    self.state = "FAULT"
                    return False

                move["state"] = state
                move["wafer_held"] = self.wafer_held
                self.segments.append(move)

                position = target

            # The gripper acts at the end of these states
            if state == "LIFT":
                self.wafer_held = True

            if state == "PLACE":
                self.wafer_held = False

        self.state = "COMPLETE"

        return True

    def timeline(self):

        # Join all segments into one continuous time history
        time = []
        position = []
        velocity = []
        acceleration = []
        states = []

        offset = 0.0

        for segment in self.segments:

            time.append(segment["time"] + offset)
            position.append(segment["position"])
            velocity.append(segment["velocity"])
            acceleration.append(segment["acceleration"])
            states += [segment["state"]] * len(segment["time"])

            offset += segment["time"][-1]

        return (
            np.concatenate(time),
            np.vstack(position),
            np.vstack(velocity),
            np.vstack(acceleration),
            states,
        )
