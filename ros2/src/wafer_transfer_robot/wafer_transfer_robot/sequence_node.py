"""Wafer pick-transfer-place state machine.

Plans IDLE -> PICK -> LIFT -> TRANSFER -> PLACE -> RETRACT -> COMPLETE with
WaferTransferSequence (the same planner as the Python simulation) and
publishes the whole plan once on /robot/planned_trajectory, time-stamped
with its start time. The safety node must accept it before the controller
sees it.

While the robot moves it publishes the current state on /robot/wafer_state
and draws the stations and the wafer in RViz (/robot/markers). The wafer is
drawn on the fork only while the fork is actually carrying it.
"""

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from wafer_transfer_robot.core.safety import SafetyMonitor
from wafer_transfer_robot.core.wafer_transfer import Station, WaferTransferSequence
from wafer_transfer_robot.sampler import JOINT_NAMES, unique_times
from wafer_transfer_robot.scene import state_at, wafer_location


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

WAFER_DIAMETER = 0.20     # m (200 mm wafer)
WAFER_THICKNESS = 0.002   # m, drawn thicker than a real wafer to be visible


class SequenceNode(Node):

    def __init__(self):

        super().__init__("sequence_node")

        defaults = {
            "pick_r": 0.4, "pick_theta_deg": 0.0, "pick_z": 0.10,
            "place_r": 0.4, "place_theta_deg": 90.0, "place_z": 0.25,
            "start_delay": 2.0,
            "unsafe_planner": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = {name: self.get_parameter(name).value for name in defaults}

        self.pick = Station(
            "Load port", p["pick_r"], np.radians(p["pick_theta_deg"]), p["pick_z"]
        )
        self.place = Station(
            "Process chamber", p["place_r"], np.radians(p["place_theta_deg"]),
            p["place_z"]
        )
        self.start_delay = p["start_delay"]

        # unsafe_planner:=true simulates a planner bug: it plans without the
        # forbidden zones, so only the independent safety node can stop it
        planner_safety = None
        if p["unsafe_planner"]:
            planner_safety = SafetyMonitor(forbidden_zones=[])
            self.get_logger().warn(
                "unsafe_planner is on: planning WITHOUT forbidden zones"
            )

        self.sequence = WaferTransferSequence(
            self.pick,
            self.place,
            safety_monitor=planner_safety
        )
        self.planned = self.sequence.run()

        if self.planned:
            times, positions, velocities, accelerations, states = (
                self.sequence.timeline()
            )
            states = np.array(states)
            (
                self.times, self.positions, self.velocities,
                self.accelerations, self.states
            ) = unique_times(times, positions, velocities, accelerations, states)

            self.get_logger().info(
                f"Planned {len(self.sequence.segments)} moves, "
                f"{self.times[-1]:.1f} s in total"
            )
        else:
            self.get_logger().error(f"Planning FAULT: {self.sequence.fault}")

        self.start_time = None
        self.last_state = None
        self.fault = None
        self.last_location = "pick"
        self.fork_z = p["pick_z"]

        self.plan_pub = self.create_publisher(
            JointTrajectory, "/robot/planned_trajectory", LATCHED
        )
        self.state_pub = self.create_publisher(String, "/robot/wafer_state", 10)
        self.fault_pub = self.create_publisher(String, "/robot/fault", LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, "/robot/markers", 10)

        self.create_subscription(
            JointState,
            "/robot/joint_state",
            self.on_joint_state,
            10
        )

        # A fault from anywhere (e.g. the safety node rejecting the plan)
        # stops the sequence
        self.create_subscription(String, "/robot/fault", self.on_fault, LATCHED)

        self.create_timer(0.05, self.tick)

    def on_fault(self, msg):

        if self.fault is None:
            self.fault = msg.data
            self.get_logger().error(f"Sequence stopped: {msg.data}")

    def on_joint_state(self, msg):

        self.fork_z = msg.position[msg.name.index("z_joint")]

    def publish_plan(self, now):

        start = now + Duration(seconds=self.start_delay)

        msg = JointTrajectory()
        msg.header.stamp = start.to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINT_NAMES

        for k, t in enumerate(self.times):
            point = JointTrajectoryPoint()
            point.positions = self.positions[k].tolist()
            point.velocities = self.velocities[k].tolist()
            point.accelerations = self.accelerations[k].tolist()
            nanoseconds = round(t * 1e9)
            point.time_from_start = DurationMsg(
                sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000
            )
            msg.points.append(point)

        self.plan_pub.publish(msg)
        self.start_time = start

        self.get_logger().info(
            f"Published plan, motion starts in {self.start_delay:.1f} s"
        )

    def tick(self):

        now = self.get_clock().now()

        # Wait for the simulator's clock before scheduling anything
        if now.nanoseconds == 0:
            return

        if self.start_time is None:
            if self.planned:
                self.publish_plan(now)
            else:
                self.fault_pub.publish(
                    String(data=f"Planning FAULT: {self.sequence.fault}")
                )
                self.start_time = now

        if not self.planned or self.fault is not None:
            state = "FAULT"
        else:
            t = (now - self.start_time).nanoseconds * 1e-9
            state = state_at(t, self.times, self.states)

        if state == "FAULT":
            # The wafer stays wherever it was when the sequence stopped
            location = self.last_location
        else:
            location = wafer_location(state, self.fork_z, self.pick.z, self.place.z)
        self.last_location = location

        if state != self.last_state:
            self.get_logger().info(f"State: {state}")
            self.last_state = state

        self.state_pub.publish(
            String(data=f"{state} | wafer at {location}")
        )
        self.publish_markers(location)

    def station_markers(self, station, k):

        # A station is a base plate with four support pins that hold the
        # wafer from below. The pins sit outside the fork's path (|y| > 3 cm
        # in the station's own frame) so the fork can slide in between them.
        c, s = np.cos(station.theta), np.sin(station.theta)
        x, y = station.r * c, station.r * s

        plate = Marker()
        plate.header.frame_id = "base_link"
        plate.ns = "station_plates"
        plate.id = k
        plate.type = Marker.CUBE
        plate.pose.position.x = x
        plate.pose.position.y = y
        # Plate top sits 2.4 cm under the wafer, below the fork's approach
        plate.pose.position.z = station.z - 0.030
        plate.pose.orientation.z = float(np.sin(station.theta / 2))
        plate.pose.orientation.w = float(np.cos(station.theta / 2))
        plate.scale.x = plate.scale.y = 0.24
        plate.scale.z = 0.012
        plate.color.r, plate.color.g, plate.color.b = 0.22, 0.23, 0.26
        plate.color.a = 1.0

        markers = [plate]
        pin_height = 0.024

        for n, (px, py) in enumerate(
            [(0.055, 0.065), (0.055, -0.065), (-0.055, 0.065), (-0.055, -0.065)]
        ):
            pin = Marker()
            pin.header.frame_id = "base_link"
            pin.ns = "station_pins"
            pin.id = 4 * k + n
            pin.type = Marker.CYLINDER
            pin.pose.position.x = x + px * c - py * s
            pin.pose.position.y = y + px * s + py * c
            pin.pose.position.z = station.z - pin_height / 2
            pin.pose.orientation.w = 1.0
            pin.scale.x = pin.scale.y = 0.008
            pin.scale.z = pin_height
            pin.color.r, pin.color.g, pin.color.b = 0.75, 0.77, 0.80
            pin.color.a = 1.0
            markers.append(pin)

        label = Marker()
        label.header.frame_id = "base_link"
        label.ns = "station_labels"
        label.id = k
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = station.z + 0.07
        label.pose.orientation.w = 1.0
        label.scale.z = 0.028
        label.color.r = label.color.g = label.color.b = 0.88
        label.color.a = 1.0
        # RViz spaces words far apart, so stack them on separate lines
        label.text = station.name.replace(" ", "\n")
        markers.append(label)

        return markers

    def publish_markers(self, location):

        markers = MarkerArray()
        markers.markers += self.station_markers(self.pick, 0)
        markers.markers += self.station_markers(self.place, 1)

        wafer = Marker()
        wafer.ns = "wafer"
        wafer.id = 0
        wafer.type = Marker.CYLINDER
        wafer.scale.x = wafer.scale.y = WAFER_DIAMETER
        wafer.scale.z = WAFER_THICKNESS
        wafer.pose.orientation.w = 1.0
        # Polished silicon: dark blue-grey
        wafer.color.r, wafer.color.g, wafer.color.b = 0.30, 0.36, 0.50
        wafer.color.a = 1.0

        if location == "fork":
            # Carried: ride on the fork, so it moves with the robot's TF
            wafer.header.frame_id = "fork_link"
            wafer.pose.position.z = 0.002 + WAFER_THICKNESS / 2
        else:
            station = self.pick if location == "pick" else self.place
            wafer.header.frame_id = "base_link"
            wafer.pose.position.x = station.r * np.cos(station.theta)
            wafer.pose.position.y = station.r * np.sin(station.theta)
            wafer.pose.position.z = station.z + WAFER_THICKNESS / 2

        markers.markers.append(wafer)
        self.marker_pub.publish(markers)


def main():

    rclpy.init()
    node = SequenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
