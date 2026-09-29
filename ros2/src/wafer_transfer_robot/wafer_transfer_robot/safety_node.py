"""Independent safety layer.

Checks every planned trajectory (position, velocity, acceleration limits
and forbidden zones) before it can reach the controller: only accepted plans
are forwarded from /robot/planned_trajectory to /robot/safe_trajectory.
It also watches the measured joint state and reports violations on
/robot/fault, and draws the forbidden zones in RViz.

The planner already checks its own moves; this node checks them again on
purpose, so a bug in the planner cannot command an unsafe motion.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from wafer_transfer_robot.core.safety import SafetyMonitor
from wafer_transfer_robot.sampler import JOINT_NAMES
from wafer_transfer_robot.scene import wedge_edges, wedge_triangles


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class SafetyNode(Node):

    def __init__(self):

        super().__init__("safety_node")

        self.monitor = SafetyMonitor()
        self.state_faulted = False

        self.safe_pub = self.create_publisher(
            JointTrajectory, "/robot/safe_trajectory", LATCHED
        )
        self.fault_pub = self.create_publisher(String, "/robot/fault", LATCHED)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/robot/safety_markers", 10
        )

        self.create_subscription(
            JointTrajectory,
            "/robot/planned_trajectory",
            self.on_plan,
            LATCHED
        )
        self.create_subscription(
            JointState,
            "/robot/joint_state",
            self.on_state,
            10
        )

        self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f"Safety layer active: {len(self.monitor.forbidden_zones)} forbidden zone(s)"
        )

    def on_plan(self, msg):

        order = [msg.joint_names.index(name) for name in JOINT_NAMES]

        def column(field):
            return np.array([
                [getattr(point, field)[i] for i in order] for point in msg.points
            ])

        index, violations = self.monitor.check_trajectory(
            column("positions"),
            column("velocities"),
            column("accelerations")
        )

        if violations:
            t = msg.points[index].time_from_start
            text = (
                f"Plan REJECTED at t = {t.sec + t.nanosec * 1e-9:.2f} s: "
                + "; ".join(violations)
            )
            self.get_logger().error(text)
            self.fault_pub.publish(String(data=text))
            return

        self.get_logger().info(
            f"Plan accepted ({len(msg.points)} points checked)"
        )
        self.safe_pub.publish(msg)

    def on_state(self, msg):

        index = [msg.name.index(name) for name in JOINT_NAMES]
        position = [msg.position[i] for i in index]

        violations = self.monitor.check_state(position)

        if violations and not self.state_faulted:
            text = "Measured state unsafe: " + "; ".join(violations)
            self.get_logger().error(text)
            self.fault_pub.publish(String(data=text))

        self.state_faulted = bool(violations)

    def publish_markers(self):

        markers = MarkerArray()

        for k, zone in enumerate(self.monitor.forbidden_zones):

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.ns = "forbidden_zones"
            marker.id = k
            marker.type = Marker.TRIANGLE_LIST
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.color.r, marker.color.g, marker.color.b = 0.85, 0.22, 0.22
            marker.color.a = 0.22
            marker.points = [
                Point(x=x, y=y, z=z)
                for x, y, z in wedge_triangles(
                    zone.theta_min, zone.theta_max,
                    zone.r_min, zone.r_max,
                    zone.z_min, zone.z_max
                )
            ]
            markers.markers.append(marker)

            # Crisp outline so the zone reads clearly from any angle
            outline = Marker()
            outline.header.frame_id = "base_link"
            outline.ns = "forbidden_zone_edges"
            outline.id = k
            outline.type = Marker.LINE_LIST
            outline.scale.x = 0.002
            outline.pose.orientation.w = 1.0
            outline.color.r, outline.color.g, outline.color.b = 1.0, 0.35, 0.35
            outline.color.a = 0.9
            outline.points = [
                Point(x=x, y=y, z=z)
                for x, y, z in wedge_edges(
                    zone.theta_min, zone.theta_max,
                    zone.r_min, zone.r_max,
                    zone.z_min, zone.z_max
                )
            ]
            markers.markers.append(outline)

            label = Marker()
            label.header.frame_id = "base_link"
            label.ns = "forbidden_zone_labels"
            label.id = k
            label.type = Marker.TEXT_VIEW_FACING
            middle = (zone.theta_min + zone.theta_max) / 2
            radius = (zone.r_min + zone.r_max) / 2
            label.pose.position.x = radius * np.cos(middle)
            label.pose.position.y = radius * np.sin(middle)
            label.pose.position.z = zone.z_max + 0.04
            label.pose.orientation.w = 1.0
            label.scale.z = 0.028
            label.color.r, label.color.g, label.color.b = 1.0, 0.55, 0.55
            label.color.a = 1.0
            # RViz spaces words far apart, so put them on separate lines
            label.text = f"FORBIDDEN\n{zone.name}"
            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main():

    rclpy.init()
    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
