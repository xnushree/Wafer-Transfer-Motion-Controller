"""Forward kinematics: joint state -> end-effector (fork) pose.

Publishes /robot/ee_pose (PoseStamped in base_link) and /robot/ee_path, a
trail of recent fork positions for RViz.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState

from wafer_transfer_robot.core.robot_kinematics import RThetaZRobot
from wafer_transfer_robot.sampler import JOINT_NAMES


class KinematicsNode(Node):

    def __init__(self):

        super().__init__("kinematics_node")

        self.declare_parameter("path_every_n", 5)
        self.declare_parameter("path_length", 3000)

        self.path_every_n = self.get_parameter("path_every_n").value
        self.path_length = self.get_parameter("path_length").value

        self.robot = RThetaZRobot(max_radius=0.45, min_z=0.05, max_z=0.30)

        self.pose_pub = self.create_publisher(PoseStamped, "/robot/ee_pose", 10)
        self.path_pub = self.create_publisher(Path, "/robot/ee_path", 10)

        self.path = Path()
        self.path.header.frame_id = "base_link"
        self.count = 0

        self.create_subscription(
            JointState,
            "/robot/joint_state",
            self.on_state,
            10
        )

    def on_state(self, msg):

        index = [msg.name.index(name) for name in JOINT_NAMES]
        r, theta, z = [msg.position[i] for i in index]

        x, y, z = self.robot.forward_kinematics(r, theta, z)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # The fork points along the arm, i.e. rotated by theta about z
        pose.pose.orientation.z = float(np.sin(theta / 2))
        pose.pose.orientation.w = float(np.cos(theta / 2))

        self.pose_pub.publish(pose)

        self.count += 1
        if self.count % self.path_every_n == 0:
            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-self.path_length:]
            self.path_pub.publish(self.path)


def main():

    rclpy.init()
    node = KinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
