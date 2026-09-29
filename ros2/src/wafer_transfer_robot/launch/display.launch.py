"""Check the URDF on its own: move each joint with sliders in a small GUI.

    ros2 launch wafer_transfer_robot display.launch.py

Needs the joint_state_publisher_gui package.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    share = get_package_share_directory("wafer_transfer_robot")

    with open(os.path.join(share, "urdf", "wafer_robot.urdf")) as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "wafer_transfer.rviz")]
        ),
    ])
