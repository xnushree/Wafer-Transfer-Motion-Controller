# Development helper for offline_check/run_system.py - not part of the ROS package.
"""Minimal in-process stand-ins for rclpy and ROS messages, enough to run the
wafer_transfer_robot nodes together on a synchronous fake bus."""
import sys
import types

BUS = {}          # topic -> list of callbacks
SIM_NS = [0]      # shared simulation time (ns)
TIMERS = []       # (node, period_s, callback, steady, next_fire)
LOG = []


class M:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = M()
        setattr(self, name, value)
        return value


def msg_class(_class_name, **defaults):
    def init(self, **kw):
        for k, v in defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(_class_name, (M,), {"__init__": init})


# ---- rclpy.time / duration / clock
class Time:
    def __init__(self, seconds=0, nanoseconds=0, clock_type=None):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    @staticmethod
    def from_msg(msg):
        return Time(nanoseconds=msg.sec * 1_000_000_000 + msg.nanosec)

    def to_msg(self):
        return M(sec=self.nanoseconds // 1_000_000_000,
                 nanosec=self.nanoseconds % 1_000_000_000)

    def __add__(self, other):
        return Time(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other):
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)


class Duration:
    def __init__(self, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)


class ClockType:
    STEADY_TIME = "steady"
    ROS_TIME = "ros"


class Clock:
    def __init__(self, clock_type=ClockType.ROS_TIME):
        self.clock_type = clock_type

    def now(self):
        return Time(nanoseconds=SIM_NS[0])


class Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, level, text):
        LOG.append((SIM_NS[0] * 1e-9, self.name, level, text))

    def info(self, text): self._log("INFO", text)
    def warn(self, text): self._log("WARN", text)
    def error(self, text): self._log("ERROR", text)


class Parameter:
    def __init__(self, value):
        self.value = value


class Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        for callback in list(BUS.get(self.topic, [])):
            callback(msg)


class Node:
    OVERRIDES = {}

    def __init__(self, name):
        self._name = name
        self._params = {}

    def declare_parameter(self, name, value):
        self._params[name] = Node.OVERRIDES.get((self._name, name), value)

    def get_parameter(self, name):
        return Parameter(self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        return Publisher(topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        BUS.setdefault(topic, []).append(callback)
        # transient-local replay
        if topic in LATCHED_STORE:
            callback(LATCHED_STORE[topic])

    def create_timer(self, period, callback, clock=None):
        steady = clock is not None and clock.clock_type == ClockType.STEADY_TIME
        TIMERS.append([self, period, callback, steady, period])

    def get_logger(self):
        return Logger(self._name)

    def get_clock(self):
        return Clock()

    def destroy_node(self):
        pass


LATCHED_STORE = {}


def install():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda node: None
    rclpy.try_shutdown = lambda: None
    sys.modules["rclpy"] = rclpy

    for sub, attrs in {
        "rclpy.node": {"Node": Node},
        "rclpy.time": {"Time": Time},
        "rclpy.duration": {"Duration": Duration},
        "rclpy.clock": {"Clock": Clock, "ClockType": ClockType},
        "rclpy.qos": {
            "QoSProfile": lambda **k: M(**k),
            "DurabilityPolicy": M(TRANSIENT_LOCAL="tl"),
            "ReliabilityPolicy": M(RELIABLE="rel"),
        },
    }.items():
        module = types.ModuleType(sub)
        for k, v in attrs.items():
            setattr(module, k, v)
        sys.modules[sub] = module
        setattr(rclpy, sub.split(".")[1], module)

    header = lambda: M(stamp=M(sec=0, nanosec=0), frame_id="")
    messages = {
        "std_msgs.msg": {
            "String": msg_class("String", data=""),
            "Float64MultiArray": msg_class("Float64MultiArray", data=list),
        },
        "sensor_msgs.msg": {
            "JointState": msg_class("JointState", header=header, name=list,
                                    position=list, velocity=list, effort=list),
        },
        "trajectory_msgs.msg": {
            "JointTrajectory": msg_class("JointTrajectory", header=header,
                                         joint_names=list, points=list),
            "JointTrajectoryPoint": msg_class(
                "JointTrajectoryPoint", positions=list, velocities=list,
                accelerations=list, time_from_start=lambda: M(sec=0, nanosec=0)),
        },
        "builtin_interfaces.msg": {
            "Duration": msg_class("Duration", sec=0, nanosec=0),
        },
        "rosgraph_msgs.msg": {"Clock": msg_class("ClockMsg", clock=None)},
        "geometry_msgs.msg": {
            "Point": msg_class("Point", x=0.0, y=0.0, z=0.0),
            "PoseStamped": msg_class("PoseStamped", header=header),
        },
        "nav_msgs.msg": {"Path": msg_class("Path", header=header, poses=list)},
        "visualization_msgs.msg": {
            "Marker": type("Marker", (msg_class("MarkerBase", header=header,
                                                points=list, text=""),), {
                "CUBE": 1, "CYLINDER": 3, "LINE_LIST": 5, "TEXT_VIEW_FACING": 9,
                "TRIANGLE_LIST": 11}),
            "MarkerArray": msg_class("MarkerArray", markers=list),
        },
    }
    for module_name, attrs in messages.items():
        package = module_name.split(".")[0]
        sys.modules.setdefault(package, types.ModuleType(package))
        module = types.ModuleType(module_name)
        for k, v in attrs.items():
            setattr(module, k, v)
        sys.modules[module_name] = module


def latch(publisher_topics):
    """Make publishes on these topics replay to late subscribers."""
    original = Publisher.publish

    def publish(self, msg):
        if self.topic in publisher_topics:
            LATCHED_STORE[self.topic] = msg
        original(self, msg)

    Publisher.publish = publish
