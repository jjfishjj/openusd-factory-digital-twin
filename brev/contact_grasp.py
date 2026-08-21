"""Drive the line with real physics: a moving belt and contact-based grasping.

The scene as authored fakes both halves of the pick cycle. The workpieces are
kinematic bodies whose transforms are time-sampled to ride the belt, and the
grasp is baked - the carried part is authored *inside* the gripper because USD
has no time-varying parenting. Nothing is actually transported and nothing is
actually held.

This script replaces both with the real thing, which is why it has to be a
runtime script rather than more USD:

  * the conveyor bed becomes a kinematic body with PhysxSurfaceVelocityAPI, so
    parts are carried by friction against a moving surface;
  * each arm gets an Isaac Sim SurfaceGripper, so the part is held by a real
    constraint created on contact and released on command.

The line indexes: the belt runs until all four parts are at their stations,
stops, the four arms pick in parallel, place into their bins, and the belt
resumes. Reports what was actually gripped and delivered.
"""

import argparse
import json
import math
import os

parser = argparse.ArgumentParser()
parser.add_argument("--stage", default="/workspace/home/ubuntu/factory-twin/output/factory_twin.usda")
parser.add_argument("--report", default="/workspace/home/ubuntu/factory-twin/output/contact_grasp_report.json")
parser.add_argument("--belt-speed", type=float, default=0.6,
                    help="m/s along +Y. The authored visual loop uses 2.5; a "
                         "physically transported part needs a speed the arms "
                         "can index against.")
parser.add_argument("--grip-distance", type=float, default=0.12)
parser.add_argument("--dt", type=float, default=1.0 / 120.0)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Gf, Sdf  # noqa: E402
import omni.usd, omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.core.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.robot.surface_gripper import GripperView  # noqa: E402
from isaacsim.robot.surface_gripper._surface_gripper import GripperStatus  # noqa: E402
from usd.schema.isaac import robot_schema  # noqa: E402

# Joint poses of the authored pick cycle (build_scene._CYCLE).
REST = (-20.0, 90.0)
PICK = (-80.0, 10.0)
PLACE = (70.0, 0.0)
YAWS = [5.0, -8.0, 10.0, 0.0]          # config: robots.poses[i][0]
STATION_Y = [-7.5, -2.5, 2.5, 7.5]
N = 4
BELT_TOP, PART = 0.875, 0.30
# The suction face. link2's +Z runs along the forearm, which at the PICK pose
# points out and *up* - measured fwd [-0.863, -0.076, +0.499] - while the part
# sits 0.27 m below it. Suction along +Z therefore misses entirely: the first
# run measured along=-0.004, lateral=0.310, i.e. the part square to the side of
# the ray. link2's -X is what actually points at the part, so the attachment
# joint is placed on the gripper's -X face and its frame rotated -90 deg about
# Y, which carries the joint's +Z onto link2's -X.
AP_POS = Gf.Vec3f(-0.12, 0.0, 0.42)     # -X face of the 0.24 x 0.14 x 0.14 gripper
AP_ROT = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))

report = {"belt_speed_mps": args.belt_speed, "grip_distance_m": args.grip_distance}

open_stage(args.stage)
world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=args.dt)
world.reset()
stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
timeline.stop()

# --- the baked choreography is what we are replacing -----------------------
for path in ("/World/Workpieces", "/World/DropParts"):
    p = stage.GetPrimAtPath(path)
    if p and p.IsValid():
        p.SetActive(False)
print("deactivated the baked workpieces and drop parts", flush=True)

# --- the belt actually moves now -------------------------------------------
# The authored bed is a Cube carrying its dimensions in xformOp:scale, and a
# scaled prim must never be a rigid body: PhysX scales the surface velocity by
# it too. Applying RigidBodyAPI straight to the bed sends parts down the line at
# belt_speed * 10 (the bed's scale.y) through a collider squashed to 0.075 of
# its thickness. So build the drive body the same way physics.py builds a link -
# an unscaled Xform for the body, the scale on a child collision shape - and
# leave the authored bed as visual geometry only.
bed = stage.GetPrimAtPath("/World/Conveyor/Bed")
UsdPhysics.CollisionAPI.Apply(bed).CreateCollisionEnabledAttr(False)

drive = UsdGeom.Xform.Define(stage, "/World/ConveyorDrive")
drive.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))
drive_prim = drive.GetPrim()
rb = UsdPhysics.RigidBodyAPI.Apply(drive_prim)
rb.CreateKinematicEnabledAttr(True)
UsdPhysics.MassAPI.Apply(drive_prim).CreateMassAttr(1000.0)

geo = UsdGeom.Cube.Define(stage, "/World/ConveyorDrive/geo")
geo.GetPrim().CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3).Set(
    Gf.Vec3f(0.6, 10.0, 0.075))
geo.GetPrim().CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(
    ["xformOp:scale"])
UsdPhysics.CollisionAPI.Apply(geo.GetPrim())
UsdGeom.Imageable(geo).MakeInvisible()          # the authored bed is the visual

sv = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(drive_prim)
sv.CreateSurfaceVelocityAttr(Gf.Vec3f(0.0, args.belt_speed, 0.0))
sv.CreateSurfaceVelocityEnabledAttr(True)

# grip needs friction: the default material is too slick to carry a part
mat = UsdShade.Material.Define(stage, "/World/Materials/BeltPhysics")
pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
pm.CreateStaticFrictionAttr(0.9)
pm.CreateDynamicFrictionAttr(0.8)
pm.CreateRestitutionAttr(0.0)
print(f"belt: unscaled kinematic body + surface velocity +Y {args.belt_speed} m/s", flush=True)

def belt(on):
    sv.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, args.belt_speed if on else 0.0, 0.0))

# --- live parts: dynamic bodies, one per station, 2 m upstream -------------
UsdGeom.Xform.Define(stage, "/World/LiveParts")
part_paths = []
for k in range(N):
    path = f"/World/LiveParts/Part_{k:02d}"
    prim = UsdGeom.Cube.Define(stage, path).GetPrim()
    prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(
        Gf.Vec3d(0.0, -9.5 + 0.4 * k, BELT_TOP + PART / 2))
    prim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(PART / 2, PART / 2, PART / 2))
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(
        ["xformOp:translate", "xformOp:scale"])
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1.0)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")
    part_paths.append(path)
UsdShade.MaterialBindingAPI.Apply(geo.GetPrim()).Bind(
    mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
    materialPurpose="physics")
print(f"{N} dynamic parts parked upstream; calibration places them", flush=True)

# --- a suction gripper per arm ---------------------------------------------
UsdGeom.Scope.Define(stage, "/World/Grippers")
grip_paths = []
for k in range(N):
    base = f"/World/Workstations/Robot_{k:02d}"
    ap_path = f"{base}/gripper_attach"
    ap = stage.DefinePrim(ap_path, "PhysicsJoint")
    j = UsdPhysics.Joint(ap)
    j.CreateBody0Rel().SetTargets([Sdf.Path(f"{base}/link2")])
    j.CreateLocalPos0Attr(AP_POS)
    j.CreateLocalRot0Attr(AP_ROT)
    j.CreateExcludeFromArticulationAttr(True)
    robot_schema.ApplyAttachmentPointAPI(ap)
    ap.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set("Z")
    ap.GetAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name).Set(0.01)

    gpath = f"/World/Grippers/Gripper_{k:02d}"
    robot_schema.CreateSurfaceGripper(stage, gpath)
    stage.GetPrimAtPath(gpath).GetRelationship(
        robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([Sdf.Path(ap_path)])
    grip_paths.append(gpath)
print(f"{N} surface grippers authored", flush=True)

# --- the authored trajectory has to stop fighting us -----------------------
# Isaac re-syncs time-sampled drive targets from USD to PhysX every step, so a
# commanded target is overwritten unless the time samples are cleared first.
drive_attrs = {}
for k in range(N):
    base = f"/World/Workstations/Robot_{k:02d}"
    for name in ("joint0_yaw", "joint1_shoulder", "joint2_elbow"):
        a = stage.GetPrimAtPath(f"{base}/{name}").GetAttribute(
            "drive:angular:physics:targetPosition")
        a.Clear()
        drive_attrs[(k, name)] = a
    drive_attrs[(k, "joint0_yaw")].Set(YAWS[k])
print("cleared the authored drive time samples; the script commands the arms now", flush=True)

def command(pose):
    for k in range(N):
        drive_attrs[(k, "joint1_shoulder")].Set(float(pose[0]))
        drive_attrs[(k, "joint2_elbow")].Set(float(pose[1]))

command(REST)
world.reset()

arms = Articulation(prim_paths_expr="/World/Workstations/Robot_*", name="arms")
parts = RigidPrim(prim_paths_expr="/World/LiveParts/Part_*", name="parts")
arms.initialize()
parts.initialize()
grippers = GripperView(paths="/World/Grippers/Gripper_*")
grippers.set_surface_gripper_properties(
    max_grip_distance=[args.grip_distance] * N,
    coaxial_force_limit=[500.0] * N,
    shear_force_limit=[500.0] * N,
    retry_interval=[2.0] * N)
timeline.play()

def run(seconds):
    for _ in range(int(seconds / args.dt)):
        world.step(render=False)

def part_y():
    return np.array(parts.get_world_poses()[0])[:, 1]

def arm_err(pose):
    q = np.degrees(np.array(arms.get_joint_positions()))
    return float(np.abs(q[:, 1:] - np.array(pose)).max())

def settle(pose, limit_s=8.0, tol=8.0):
    command(pose)
    t = 0.0
    while t < limit_s:
        run(0.1); t += 0.1
        if arm_err(pose) < tol:
            return round(t, 2)
    return None

PART_TOP = BELT_TOP + PART            # 1.175
FACE_GAP = 0.035                      # how far above the part the face should stop

def suction_frame(k):
    """World position and forward direction of robot k's suction face."""
    m = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(f"/World/Workstations/Robot_{k:02d}/link2"))
    local = Gf.Matrix4d()
    local.SetTransform(Gf.Rotation(Gf.Quatd(AP_ROT)), Gf.Vec3d(AP_POS))
    jm = local * m
    ap = np.array(jm.Transform(Gf.Vec3d(0, 0, 0)))
    fwd = np.array(jm.TransformDir(Gf.Vec3d(0, 0, 1)))
    return ap, fwd / (np.linalg.norm(fwd) or 1.0)

def ray_hit(ap, fwd, z):
    """Where the suction ray crosses a horizontal plane, and how far along."""
    if abs(fwd[2]) < 1e-6:
        return None, None
    t = (z - ap[2]) / fwd[2]
    return ap + fwd * t, t

print("\n=== CALIBRATE ===", flush=True)
# The authored PICK pose (-80, 10) was built for a baked grasp, where the part
# is teleported into the gripper and no approach clearance is ever needed. Under
# a real gripper it puts the suction face at z~1.11 - *below* the 1.175 top of
# the part - so the arm swipes the part off the belt instead of descending onto
# it. Rather than hand-tune a replacement, sweep candidates and measure: keep
# the pose whose face sits just above the part with its ray pointing down the
# belt centre line.
CANDIDATES = [(sh, el) for sh in range(-80, -66, 2) for el in (10, 14, 18, 22)]
best, scan = None, []
for pose in CANDIDATES:
    settle(pose, limit_s=2.5, tol=8.0)
    aps, fwds, hits = [], [], []
    for k in range(N):
        ap, fwd = suction_frame(k)
        hit, t = ray_hit(ap, fwd, PART_TOP)
        aps.append(ap); fwds.append(fwd); hits.append((hit, t))
    face_z = float(np.mean([a[2] for a in aps]))
    hit_x = float(np.mean([h[0][0] for h in hits if h[0] is not None]))
    reach = float(np.mean([h[1] for h in hits if h[1] is not None]))
    # What matters is how far the face stops above the part along its own ray,
    # and whether that ray lands on the belt centre line - not the face height
    # on its own. Scoring on face height alone picked a pose 0.16 m clear of the
    # part, past any honest suction range.
    ok = face_z > PART_TOP + 0.01 and 0.0 < reach < 0.30
    score = abs(reach - FACE_GAP) + 2.0 * abs(hit_x) + (0.0 if ok else 10.0)
    scan.append({"pose": pose, "face_z": round(face_z, 3), "hit_x": round(hit_x, 3),
                 "reach_m": round(reach, 3), "usable": ok, "score": round(score, 4)})
    print(f"  ({pose[0]:+4d},{pose[1]:+3d})  face_z {face_z:.3f}  hit_x {hit_x:+.3f}  "
          f"reach {reach:+.3f}  {'ok' if ok else '--'}", flush=True)
    if best is None or score < best[0]:
        best = (score, pose)
APPROACH = best[1]
report["calibration"] = {"candidates": scan, "chosen_pose_deg": list(APPROACH),
                         "authored_pick_pose_deg": list(PICK)}
print(f"  chosen approach pose: shoulder {APPROACH[0]}, elbow {APPROACH[1]} "
      f"(authored PICK was {PICK})", flush=True)

# Aim each part at where that robot's ray actually lands on the belt. This is
# what the station-centre assumption got wrong: the arms carry a per-robot yaw
# (5, -8, 10, 0 deg), so each reach lands at its own y, up to 0.24 m off centre.
settle(APPROACH, limit_s=4.0, tol=8.0)
targets = []
for k in range(N):
    ap, fwd = suction_frame(k)
    hit, _ = ray_hit(ap, fwd, BELT_TOP + PART / 2)
    targets.append(hit)
    print(f"  Robot_{k:02d} aims at x {hit[0]:+.3f}  y {hit[1]:+.3f}  "
          f"(station centre {STATION_Y[k]:+.1f})", flush=True)
report["aim_targets"] = [t.round(4).tolist() for t in targets]
settle(REST, limit_s=4.0, tol=8.0)

# park each part the same distance upstream of its own aim point
D = 1.2
pos = np.array([[t[0], t[1] - D, BELT_TOP + PART / 2] for t in targets])
orn = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (N, 1))
parts.set_world_poses(pos, orn)
run(0.75)

log = []
print("\n=== INDEXING ===", flush=True)
belt(True)
y0 = part_y().copy()
target_y = np.array([t[1] for t in targets])
t = 0.0
while t < 25.0:
    run(0.1); t += 0.1
    if np.abs(part_y() - target_y).max() < 0.06:
        break
belt(False)
run(0.5)
travel = part_y() - y0
offset = part_y() - target_y
z_now = np.array(parts.get_world_poses()[0])[:, 2]
print(f"  indexed in {t:.2f}s; travelled {travel.round(3)} m", flush=True)
print(f"  offset from aim point: {offset.round(3)} m   z {z_now.round(3)}", flush=True)
report["indexing"] = {"seconds": round(t, 2), "travel_m": travel.round(4).tolist(),
                      "offset_from_aim_m": offset.round(4).tolist(),
                      "z": z_now.round(4).tolist(),
                      "transported": bool((travel > 0.5).all())}

print("\n=== PICK ===", flush=True)
tA = settle(APPROACH, limit_s=8.0, tol=8.0)
print(f"  arms at approach pose after {tA}s (err {arm_err(APPROACH):.2f} deg)", flush=True)
ppos = np.array(parts.get_world_poses()[0])
geom = []
for k in range(N):
    ap, fwd = suction_frame(k)
    delta = ppos[k] - ap
    along = float(np.dot(delta, fwd))
    lateral = float(np.linalg.norm(delta - along * fwd))
    geom.append({"attach_point": ap.round(3).tolist(), "forward": fwd.round(3).tolist(),
                 "part": ppos[k].round(3).tolist(),
                 "distance_m": round(float(np.linalg.norm(delta)), 3),
                 "along_forward_m": round(along, 3), "lateral_m": round(lateral, 3)})
    print(f"  Robot_{k:02d} face {ap.round(3)} | part {ppos[k].round(3)} | "
          f"along {along:+.3f} lateral {lateral:.3f}", flush=True)
report["pick_geometry"] = geom

# Size the suction range to the approach we actually achieved. The ray lands on
# the part, and the part is always nearer along it than the belt underneath, so
# this cannot re-create the earlier failure of gripping /World/ConveyorDrive -
# that happened only when a 0.45 m lateral miss let the ray past the part.
need = max(g["along_forward_m"] for g in geom) + 0.05
reach_m = float(min(max(need, 0.10), 0.60))
grippers.set_surface_gripper_properties(
    max_grip_distance=[reach_m] * N, coaxial_force_limit=[500.0] * N,
    shear_force_limit=[500.0] * N, retry_interval=[2.0] * N)
report["pick"] = {}
report["grip_range_m"] = round(reach_m, 3)
print(f"  suction range set to {reach_m:.3f} m (max along {need - 0.05:.3f})", flush=True)

grippers.apply_gripper_action([0.5] * N)
run(1.0)
status = [GripperStatus(s).name for s in grippers.get_surface_gripper_status()]
gripped = grippers.get_gripped_objects()
z_at_grip = np.array(parts.get_world_poses()[0])[:, 2]
for k in range(N):
    print(f"  Robot_{k:02d}: {status[k]:8s} gripped={list(gripped[k])}", flush=True)
right = sum(1 for k, g in enumerate(gripped) if f"/World/LiveParts/Part_{k:02d}" in list(g))
report["pick"].update({"reach_seconds": tA, "status": status,
                  "gripped": [list(g) for g in gripped],
                  "n_gripped": sum(1 for g in gripped if g),
                  "n_gripped_correct_part": right})
print(f"  gripped the right part: {right}/{N}", flush=True)

print("\n=== CARRY ===", flush=True)
tB = settle(PLACE)

still = grippers.get_gripped_objects()
z_carry = np.array(parts.get_world_poses()[0])[:, 2]
print(f"  arms at PLACE pose after {tB}s", flush=True)
print(f"  part z at grip {z_at_grip.round(3)} -> carried {z_carry.round(3)}", flush=True)
print(f"  still held: {[len(g) for g in still]}", flush=True)
report["carry"] = {"reach_seconds": tB, "still_held": [list(g) for g in still],
                   "n_still_held": sum(1 for g in still if g),
                   "z_at_grip": z_at_grip.round(4).tolist(),
                   "z_carried": z_carry.round(4).tolist(),
                   "lifted": bool((z_carry > BELT_TOP + PART / 2 + 0.1).all())}

print("\n=== RELEASE ===", flush=True)
grippers.apply_gripper_action([-0.5] * N)
run(2.5)
settle(REST)
run(1.0)
pos = np.array(parts.get_world_poses()[0])
bins = []
for k in range(N):
    b = stage.GetPrimAtPath(f"/World/Bins/Bin_{k:02d}")
    if b and b.IsValid():
        bins.append(np.array(UsdGeom.XformCache().GetLocalToWorldTransform(b).ExtractTranslation()))
    else:
        bins.append(np.array([np.nan] * 3))
delivered = []
for k in range(N):
    d = float(np.linalg.norm(pos[k][:2] - bins[k][:2])) if not np.isnan(bins[k][0]) else float("nan")
    delivered.append(d)
    print(f"  Part_{k:02d} final {pos[k].round(3)}  bin {bins[k].round(3)}  "
          f"xy distance {d:.3f} m", flush=True)
report["release"] = {"final_pos": pos.round(4).tolist(),
                     "bin_pos": [b.round(4).tolist() for b in bins],
                     "xy_distance_to_bin_m": [round(d, 4) for d in delivered],
                     "n_delivered": sum(1 for d in delivered if d == d and d < 0.75)}

checks = {
    "belt physically transported every part": report["indexing"]["transported"],
    "parts indexed onto each arm's aim point (<0.06 m)":
        max(abs(o) for o in report["indexing"]["offset_from_aim_m"]) < 0.06,
    "a usable approach pose was found": any(c["usable"] for c in report["calibration"]["candidates"]),
    "arms reached the approach pose": tA is not None,
    "every gripper gripped its own part": report["pick"]["n_gripped_correct_part"] == N,
    "grip survived the carry": report["carry"]["n_still_held"] == N,
    "parts were lifted off the belt": report["carry"]["lifted"],
    "parts delivered to their bins": report["release"]["n_delivered"] == N,
}
report["verdict"] = checks
report["passed"] = all(checks.values())
print("\n=== VERDICT ===", flush=True)
for k, v in checks.items():
    print(f"  [{'PASS' if v else 'FAIL'}] {k}", flush=True)
print(f"\n  OVERALL: {'PASS' if report['passed'] else 'FAIL'} "
      f"({sum(checks.values())}/{len(checks)})", flush=True)

timeline.stop()
os.makedirs(os.path.dirname(args.report), exist_ok=True)
with open(args.report, "w") as fh:
    json.dump(report, fh, indent=2)
print(f"\nreport written to {args.report}", flush=True)
sim_app.close()
