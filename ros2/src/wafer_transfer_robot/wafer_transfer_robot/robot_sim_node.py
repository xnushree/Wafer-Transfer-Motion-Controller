"""Simulated R-theta-Z robot: integrates the dynamics and owns the clock.

Every dt (wall time) it applies the latest force command for one step,
advances simulation time, publishes /clock and a time-stamped
/robot/joint_state. All other nodes run with use_sim_time:=true, so they
share this clock and the controller can match each measurement to the
reference at exactly the time it was taken.
"""

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from wafer_transfer_robot.core.robot_dynamics import CylindricalRobotDynamics
from wafer_transfer_robot.sampler import JOINT_NAMES


class RobotSimNode(Node):

    def __init__(self):

        super().__init__("robot_sim_node")

        self.declare_parameter("dt", 0.01)
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("initial_position", [0.1, 0.0, 0.1])
        self.declare_parameter("force_min", [-100.0, -20.0, 0.0])
        self.declare_parameter("force_max", [100.0, 20.0, 100.0])

        self.dt = self.get_parameter("dt").value
        real_time_factor = self.get_parameter("real_time_factor").value
        self.force_min = np.array(self.get_parameter("force_min").value)
        self.force_max = np.array(self.get_parameter("force_max").value)

        self.robot = CylindricalRobotDynamics(
            radial_mass=5.0,
            radial_damping=1.0,
            rotational_inertia=0.5,
            rotational_damping=0.1,
            vertical_mass=4.0,
            vertical_damping=1.0
        )

        self.position = np.array(
            self.get_parameter("initial_position").value,
            dtype=float
        )
        self.velocity = np.zeros(3)

        # Hold the Z axis against gravity until the controller takes over
        self.force = np.array(
            [0.0, 0.0, self.robot.vertical_mass * self.robot.gravity]
        )

        self.steps = 0

        self.clock_pub = self.create_publisher(ClockMsg, "/clock", 10)
        self.state_pub = self.create_publisher(JointState, "/robot/joint_state", 10)

        self.create_subscription(
            Float64MultiArray,
            "/robot/joint_command",
            self.on_command,
            10
        )

        # Wall-clock timer: this node is the source of simulation time
        self.create_timer(
            self.dt / real_time_factor,
            self.step,
            clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        self.get_logger().info(
            f"Simulating at {1.0 / self.dt:.0f} Hz, start position "
            f"{np.round(self.position, 3).tolist()}"
        )

    def on_command(self, msg):

        if len(msg.data) == 3:
            # The actuator limit is physical, so the plant enforces it too
            self.force = np.clip(np.array(msg.data), self.force_min, self.force_max)

    def step(self):

        self.position, self.velocity = self.robot.rk4_step(
            self.position,
            self.velocity,
            self.force,
            self.dt
        )
        self.steps += 1

        stamp = Time(nanoseconds=round(self.steps * self.dt * 1e9)).to_msg()

        clock = ClockMsg()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        state = JointState()
        state.header.stamp = stamp
        state.name = JOINT_NAMES
        state.position = self.position.tolist()
        state.velocity = self.velocity.tolist()
        state.effort = self.force.tolist()
        self.state_pub.publish(state)


def main():

    rclpy.init()
    node = RobotSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
