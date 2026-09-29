"""PID + feedforward joint controller.

Runs once per /robot/joint_state message. It looks up the reference at the
measurement's own timestamp (not "now"), so message delays do not show up
as a fake tracking error. Until a safe trajectory arrives it holds the
first measured position.

On any /robot/fault it makes a controlled stop: if the robot is moving it
slows down along the planned (already safety-checked) path within the
acceleration limits, then holds; otherwise it holds where it is.

Publishes /robot/joint_command (forces [F_r, T_theta, F_z]) and
/robot/tracking_error ([r, theta, z] desired - actual) for rqt_plot.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from wafer_transfer_robot.core.feedforward_controller import FeedforwardController
from wafer_transfer_robot.core.safety import default_joint_limits
from wafer_transfer_robot.sampler import (
    JOINT_NAMES,
    ControlledStop,
    TrajectorySampler,
)


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


def seconds(stamp):

    return Time.from_msg(stamp).nanoseconds * 1e-9


class ControllerNode(Node):

    def __init__(self):

        super().__init__("controller_node")

        self.declare_parameter("dt", 0.01)
        self.declare_parameter("force_min", [-100.0, -20.0, 0.0])
        self.declare_parameter("force_max", [100.0, 20.0, 100.0])

        self.dt = self.get_parameter("dt").value
        self.force_min = np.array(self.get_parameter("force_min").value)
        self.force_max = np.array(self.get_parameter("force_max").value)

        # Same gains and nominal model as the Python and Simulink studies
        self.controllers = [
            FeedforwardController(kp=300.0, ki=20.0, kd=30.0, mass=5.0, damping=1.0),
            FeedforwardController(kp=100.0, ki=5.0, kd=10.0, mass=0.5, damping=0.1),
            FeedforwardController(
                kp=300.0, ki=20.0, kd=30.0, mass=4.0, damping=1.0, gravity=9.81
            ),
        ]

        self.sampler = None
        self.start_time = None
        self.hold_position = None
        self.faulted = False
        self.stop = None
        self.stop_start = None

        self.acceleration_max = np.array([
            limits.acceleration_max for limits in default_joint_limits()
        ])

        self.command_pub = self.create_publisher(
            Float64MultiArray, "/robot/joint_command", 10
        )
        self.error_pub = self.create_publisher(
            Float64MultiArray, "/robot/tracking_error", 10
        )

        self.create_subscription(
            JointTrajectory,
            "/robot/safe_trajectory",
            self.on_trajectory,
            LATCHED
        )
        self.create_subscription(
            JointState,
            "/robot/joint_state",
            self.on_state,
            10
        )
        self.create_subscription(String, "/robot/fault", self.on_fault, LATCHED)

    def on_fault(self, msg):

        if not self.faulted:
            self.faulted = True
            self.get_logger().error(f"Fault - controlled stop: {msg.data}")

    def on_trajectory(self, msg):

        order = [msg.joint_names.index(name) for name in JOINT_NAMES]

        times = np.array([
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in msg.points
        ])

        def column(field):
            return np.array([
                [getattr(point, field)[i] for i in order] for point in msg.points
            ])

        self.sampler = TrajectorySampler(
            times,
            column("positions"),
            column("velocities"),
            column("accelerations")
        )
        self.start_time = seconds(msg.header.stamp)

        self.get_logger().info(
            f"Following trajectory: {len(times)} points, "
            f"{self.sampler.duration:.1f} s, starting at t = {self.start_time:.2f} s"
        )

    def on_state(self, msg):

        index = [msg.name.index(name) for name in JOINT_NAMES]
        position = np.array([msg.position[i] for i in index])
        velocity = np.array([msg.velocity[i] for i in index])

        t = seconds(msg.header.stamp)

        moving = (
            self.sampler is not None
            and self.start_time <= t <= self.start_time + self.sampler.duration
        )

        if self.faulted and self.stop is None and moving:
            self.stop = ControlledStop(
                self.sampler,
                t - self.start_time,
                self.acceleration_max
            )
            self.stop_start = t
            self.get_logger().info(
                f"Stopping along the path in {self.stop.stop_time:.2f} s"
            )

        if self.stop is not None:
            reference = self.stop.sample(t - self.stop_start)
        elif not self.faulted and self.sampler is not None and t >= self.start_time:
            reference = self.sampler.sample(t - self.start_time)
        else:
            if self.hold_position is None:
                self.hold_position = position.copy()
            reference = (self.hold_position, np.zeros(3), np.zeros(3))

        desired, desired_velocity, desired_acceleration = reference

        command = np.array([
            self.controllers[j].update(
                desired[j],
                position[j],
                desired_velocity[j],
                desired_acceleration[j],
                velocity[j],
                self.dt
            )
            for j in range(3)
        ])

        command = np.clip(command, self.force_min, self.force_max)

        self.command_pub.publish(Float64MultiArray(data=command.tolist()))
        self.error_pub.publish(
            Float64MultiArray(data=(desired - position).tolist())
        )


def main():

    rclpy.init()
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
