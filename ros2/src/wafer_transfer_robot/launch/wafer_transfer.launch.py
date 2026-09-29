"""Full simulation: robot, safety layer, controller, sequence and RViz.

    ros2 launch wafer_transfer_robot wafer_transfer.launch.py
    ros2 launch wafer_transfer_robot wafer_transfer.launch.py place_theta_deg:=135.0

place_theta_deg:=135.0 puts the process chamber inside the forbidden zone,
so the planner refuses and the robot stays still.

Adding unsafe_planner:=true makes the planner ignore the forbidden zones (a
simulated planner bug); the independent safety node then rejects the plan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    share = get_package_share_directory("wafer_transfer_robot")

    with open(os.path.join(share, "urdf", "wafer_robot.urdf")) as file:
        robot_description = file.read()

    rviz_config = os.path.join(share, "rviz", "wafer_transfer.rviz")

    sim_time = {"use_sim_time": True}

    def robot_node(executable, parameters=()):
        return Node(
            package="wafer_transfer_robot",
            executable=executable,
            output="screen",
            parameters=[sim_time, *parameters]
        )

    return LaunchDescription([

        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("place_theta_deg", default_value="90.0"),
        DeclareLaunchArgument("unsafe_planner", default_value="false"),

        # Owns the simulation clock, so it does not use sim time itself
        Node(
            package="wafer_transfer_robot",
            executable="robot_sim_node",
            output="screen"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, sim_time],
            remappings=[("joint_states", "/robot/joint_state")]
        ),

        robot_node("safety_node"),
        robot_node("controller_node"),
        robot_node("kinematics_node"),
        robot_node(
            "sequence_node",
            [{
                "place_theta_deg": ParameterValue(
                    LaunchConfiguration("place_theta_deg"),
                    value_type=float
                ),
                "unsafe_planner": ParameterValue(
                    LaunchConfiguration("unsafe_planner"),
                    value_type=bool
                ),
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[sim_time],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
    ])
