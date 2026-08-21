"""Publish the twin's live state on ROS 2.

The README claims the AMR route is authored as a first-class `BasisCurves` path
rather than a baked mesh, "so a nav stack can consume it". This makes good on
that: the curve is read straight off the stage and published as a
`nav_msgs/Path`, which is exactly what a nav stack subscribes to.

Alongside it the four articulations stream `sensor_msgs/JointState` while the
authored timeline plays, and simulation time goes out on `/clock` so anything
downstream can run on sim time rather than wall time.

rclpy comes from the copy the Isaac Sim ROS 2 bridge ships for its own Python -
the system ROS 2 is built for 3.12 and Isaac runs 3.11, so importing the system
one would fail. The `ros2` CLI still talks to this over DDS, which is how
verify_ros2.sh checks the topics from outside the simulator.
"""

import argparse
import json
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--stage", default="/workspace/home/ubuntu/factory-twin/output/factory_twin.usda")
parser.add_argument("--report", default="/workspace/home/ubuntu/factory-twin/output/ros2_report.json")
parser.add_argument("--seconds", type=float, default=90.0, help="how long to keep publishing")
parser.add_argument("--rate", type=float, default=30.0, help="joint state publish rate (Hz)")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
sim_app.update()

# The bridge ships an rclpy built against Isaac's interpreter; use that one.
BRIDGE_RCLPY = "/root/isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
if os.path.isdir(BRIDGE_RCLPY):
    sys.path.insert(0, BRIDGE_RCLPY)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from pxr import Usd, UsdGeom  # noqa: E402
import omni.usd, omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402

FRAME = "factory_world"
open_stage(args.stage)
stage_file = Usd.Stage.Open(args.stage)
TPS = float(stage_file.GetTimeCodesPerSecond() or 24.0)
START_TC, END_TC = int(stage_file.GetStartTimeCode()), int(stage_file.GetEndTimeCode())

dt = 1.0 / args.rate
world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
world.reset()
stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()

arms = Articulation(prim_paths_expr="/World/Workstations/Robot_*", name="arms")
world.scene.add(arms)
world.reset()
dof_names = list(arms.dof_names)
n_robots = int(arms.count)
joint_names = [f"robot_{r:02d}_{d}" for r in range(n_robots) for d in dof_names]
print(f"publishing {len(joint_names)} joints from {n_robots} articulations", flush=True)

# --- the AMR route, straight off the stage ---------------------------------
curve = UsdGeom.BasisCurves(stage.GetPrimAtPath("/World/AMR_Path"))
pts = curve.GetPointsAttr().Get() if curve else None
route = [(float(p[0]), float(p[1]), float(p[2])) for p in pts] if pts else []
print(f"AMR route: {len(route)} vertices from /World/AMR_Path", flush=True)

rclpy.init()
node = Node("factory_twin")
latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)
pub_js = node.create_publisher(JointState, "/factory/joint_states", 10)
pub_path = node.create_publisher(Path, "/factory/amr_route", latched)
pub_clock = node.create_publisher(Clock, "/clock", 10)

def stamp(t):
    return RosTime(sec=int(t), nanosec=int((t - int(t)) * 1e9))

path_msg = Path(header=Header(frame_id=FRAME))
for x, y, z in route:
    ps = PoseStamped(header=Header(frame_id=FRAME))
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    ps.pose.orientation.w = 1.0
    path_msg.poses.append(ps)

timeline.set_time_codes_per_second(TPS)
timeline.set_start_time(START_TC / TPS)
timeline.set_end_time(END_TC / TPS)
timeline.set_looping(True)          # keep the arms moving for the whole window
timeline.set_current_time(START_TC / TPS)
world.reset()
arms.initialize()
if not timeline.is_playing():
    timeline.play()

n_js = n_clock = n_path = 0
sim_t = 0.0
next_path = 0.0
print(f"streaming for {args.seconds}s at {args.rate} Hz", flush=True)
while sim_t < args.seconds:
    world.step(render=True)
    sim_t += dt
    st = stamp(sim_t)

    pub_clock.publish(Clock(clock=st)); n_clock += 1

    q = np.array(arms.get_joint_positions()).reshape(-1)
    v = np.array(arms.get_joint_velocities()).reshape(-1)
    msg = JointState(header=Header(stamp=st, frame_id=FRAME))
    msg.name = joint_names
    msg.position = [float(x) for x in q]
    msg.velocity = [float(x) for x in v]
    pub_js.publish(msg); n_js += 1

    if sim_t >= next_path:
        path_msg.header.stamp = st
        pub_path.publish(path_msg); n_path += 1
        next_path = sim_t + 1.0

    rclpy.spin_once(node, timeout_sec=0.0)
    if abs(sim_t % 10.0) < dt:
        print(f"  t={sim_t:5.1f}s  joint_states={n_js}  clock={n_clock}  "
              f"path={n_path}", flush=True)

report = {
    "topics": {"/factory/joint_states": "sensor_msgs/JointState",
               "/factory/amr_route": "nav_msgs/Path",
               "/clock": "rosgraph_msgs/Clock"},
    "joint_names": joint_names,
    "n_joints": len(joint_names),
    "route_vertices": len(route),
    "rate_hz": args.rate,
    "seconds": args.seconds,
    "published": {"joint_states": n_js, "clock": n_clock, "amr_route": n_path},
    "rclpy_from": BRIDGE_RCLPY,
}
os.makedirs(os.path.dirname(args.report), exist_ok=True)
with open(args.report, "w") as fh:
    json.dump(report, fh, indent=2)
print(f"\npublished {n_js} joint_states, {n_clock} clock, {n_path} path messages", flush=True)
print(f"report written to {args.report}", flush=True)

timeline.stop()
node.destroy_node()
rclpy.shutdown()
sim_app.close()
