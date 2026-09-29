"""Run all five ROS 2 nodes together WITHOUT ROS, on a fake message bus.

A development check for machines without ROS 2 (e.g. Windows). Run it from
the Python project folder with the project's .venv:

    python ros2/offline_check/run_system.py                 # normal run, 90 deg
    python ros2/offline_check/run_system.py 170             # chamber beyond the pillar
    python ros2/offline_check/run_system.py 135             # planner refuses
    python ros2/offline_check/run_system.py 135 unsafe      # safety node rejects
    python ros2/offline_check/run_system.py 90 inject 6.0   # fault at t = 6 s

It checks the node logic and message flow, not ROS itself (QoS, real
timing, launch files); run the real system in ROS 2 for that.
"""
import os
import sys

import numpy as np

import fake_ros

fake_ros.install()
fake_ros.latch({
    "/robot/planned_trajectory", "/robot/safe_trajectory", "/robot/fault"
})

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "src", "wafer_transfer_robot"
    )
)

place_theta = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
fake_ros.Node.OVERRIDES[("sequence_node", "place_theta_deg")] = place_theta
unsafe = len(sys.argv) > 2 and sys.argv[2] == "unsafe"
inject = len(sys.argv) > 2 and sys.argv[2] == "inject"
INJECT_T = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
if unsafe:
    fake_ros.Node.OVERRIDES[("sequence_node", "unsafe_planner")] = True

from wafer_transfer_robot.robot_sim_node import RobotSimNode
from wafer_transfer_robot.controller_node import ControllerNode
from wafer_transfer_robot.safety_node import SafetyNode
from wafer_transfer_robot.kinematics_node import KinematicsNode
from wafer_transfer_robot.sequence_node import SequenceNode

# Collect outputs
record = {"state": [], "error": [], "wafer": [], "fault": [], "joint": [],
          "markers": None, "safety_markers": None, "pose": None}
fake_ros.BUS.setdefault("/robot/joint_state", []).append(
    lambda m: record["joint"].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                      list(m.position), list(m.effort))))
fake_ros.BUS.setdefault("/robot/tracking_error", []).append(
    lambda m: record["error"].append(list(m.data)))
fake_ros.BUS.setdefault("/robot/wafer_state", []).append(
    lambda m: record["state"].append((fake_ros.SIM_NS[0] * 1e-9, m.data)))
fake_ros.BUS.setdefault("/robot/fault", []).append(
    lambda m: record["fault"].append(m.data))
fake_ros.BUS.setdefault("/robot/markers", []).append(
    lambda m: record.__setitem__("markers", m))
fake_ros.BUS.setdefault("/robot/safety_markers", []).append(
    lambda m: record.__setitem__("safety_markers", m))
fake_ros.BUS.setdefault("/robot/ee_pose", []).append(
    lambda m: record.__setitem__("pose", m))

# Start order like a launch file (arbitrary): sim first
sim = RobotSimNode()
safety = SafetyNode()
controller = ControllerNode()
kinematics = KinematicsNode()
sequence = SequenceNode()

# Main loop: fire the sim's steady timer; sim-time timers fire from /clock
fake_ros.BUS.setdefault("/clock", []).append(
    lambda m: fake_ros.SIM_NS.__setitem__(0, m.clock.sec * 1_000_000_000 + m.clock.nanosec))

sim_timer = [t for t in fake_ros.TIMERS if t[3]][0]
other_timers = [t for t in fake_ros.TIMERS if not t[3]]

end_time = 16.0
fault_pub = fake_ros.Publisher("/robot/fault")
injected = False
while fake_ros.SIM_NS[0] * 1e-9 < end_time:
    if inject and not injected and fake_ros.SIM_NS[0] * 1e-9 >= INJECT_T:
        injected = True
        fault_pub.publish(fake_ros.sys.modules["std_msgs.msg"].String(data="TEST e-stop"))
        print("Injected fault at", record["joint"][-1][0], "joints", [round(x, 4) for x in record["joint"][-1][1]])
    sim_timer[2]()
    now = fake_ros.SIM_NS[0] * 1e-9
    for timer in other_timers:
        if now + 1e-9 >= timer[4]:
            timer[2]()
            timer[4] += timer[1]

# ---- Report
print(f"place_theta_deg = {place_theta}")
for t, node, level, text in fake_ros.LOG:
    print(f"  [{t:6.2f}] {level:<5} {node}: {text}")

error = np.array(record["error"])
print(f"Max |error|: R {abs(error[:,0]).max()*1e3:.3f} mm, "
      f"theta {np.degrees(abs(error[:,1]).max()):.3f} deg, Z {abs(error[:,2]).max()*1e3:.3f} mm")
print(f"Final error: R {abs(error[-1,0])*1e3:.4f} mm, Z {abs(error[-1,2])*1e3:.4f} mm")
joint = record["joint"]
print(f"Final joints: {np.round(joint[-1][1], 4)}")
effort = np.array([j[2] for j in joint])
print(f"Peak |F_r| {abs(effort[:,0]).max():.2f}, |T| {abs(effort[:,1]).max():.2f}, F_z {effort[:,2].min():.1f}..{effort[:,2].max():.1f}")

transitions = []
for t, s in record["state"]:
    if not transitions or transitions[-1][1] != s:
        transitions.append((t, s))
print("Wafer-state transitions:")
for t, s in transitions:
    print(f"  {t:6.2f} s  {s}")
print("Faults:", record["fault"])
if record["safety_markers"]:
    print("Safety markers:", [(m.ns, m.type, len(m.points)) for m in record["safety_markers"].markers])
if record["markers"]:
    print("Scene markers:", [(m.ns, m.header.frame_id) for m in record["markers"].markers])
if record["pose"]:
    p = record["pose"].pose.position
    print(f"Last ee_pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")

if inject:
    e = np.array(record["error"]); j = record["joint"]
    r = np.array([x[1][0] for x in j]); tt = np.array([x[0] for x in j])
    v = np.gradient(r, tt); a = np.gradient(v, tt)
    after = tt >= INJECT_T
    print(f"After fault: max |error| R {abs(e[after,0]).max()*1e3:.3f} mm, "
          f"peak |acc R| {abs(a[after]).max():.3f} m/s^2 (limit 1.0), "
          f"min r {r[after].min():.4f}, final r {r[-1]:.4f}, max reverse speed {max(v[after].max(), 0)*1e3:.3f} mm/s")
if inject:
    th = np.array([x[1][1] for x in j]); z = np.array([x[1][2] for x in j])
    for name, q, lim in [("theta", th, 4.0), ("z", z, 0.5)]:
        vq = np.gradient(q, tt); aq = np.gradient(vq, tt)
        print(f"  {name}: peak |acc| after fault {abs(aq[after]).max():.3f} (limit {lim}), final {q[-1]:.4f}")
    print(f"  max |error| after fault: theta {np.degrees(abs(e[after,1]).max()):.3f} deg, z {abs(e[after,2]).max()*1e3:.3f} mm")
