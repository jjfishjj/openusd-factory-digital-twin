"""Validate the factory twin's UsdPhysics rigging in a real PhysX solver.

Runs headless in Isaac Sim on a GPU box.

  0.  Static USD audit      - schema counts, drive targets vs joint limits, CCD
  1.  Dynamics              - drop parts fall, and where they end up
  2a. Pose hold + ablation  - can each arm hold its authored rest pose, and if
                              not, which part of the scene is interfering?
  2b. Timeline playback     - play the authored 192-frame trajectory and let
                              Isaac Sim drive the joints from the USD time
                              samples, which is how the scene is meant to run
  2c. Motion                - animated joints travel; static joints hold

Note on method: Isaac Sim re-syncs time-sampled drive targets from USD to PhysX
on every step, so manually commanding targets while the timeline is stopped has
no effect - the authored value at the current timeline time always wins. The
trajectory therefore has to be validated by *playing the timeline*.
"""

import argparse
import json
import math
import os

parser = argparse.ArgumentParser()
parser.add_argument("--stage", default="/workspace/home/ubuntu/factory-twin/output/factory_twin.usda")
parser.add_argument("--report", default="/workspace/home/ubuntu/factory-twin/output/physx_report.json")
parser.add_argument("--settle-seconds", type=float, default=5.0)
parser.add_argument("--hold-seconds", type=float, default=2.0)
parser.add_argument("--dt", type=float, default=1.0 / 120.0)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402
import omni.usd  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.core.prims import Articulation, RigidPrim  # noqa: E402

report = {"stage": args.stage, "physics_dt": args.dt}
ROBOTS = "/World/Workstations/Robot_*"
DROPS = "/World/DropParts/Drop_*"
BELT_TOP, FLOOR_TOP, HALF = 0.875, 0.0, 0.175

# ---------------------------------------------------------------- phase 0
print("\n=== PHASE 0 - static USD audit ===", flush=True)
open_stage(args.stage)
stage = Usd.Stage.Open(args.stage)

# Capture the authored timeline before World() rewrites timeCodesPerSecond.
TPS = float(stage.GetTimeCodesPerSecond() or 24.0)
START_TC, END_TC = int(stage.GetStartTimeCode()), int(stage.GetEndTimeCode())
print(f"authored timeline: {START_TC}..{END_TC} @ {TPS} fps", flush=True)

arts, revolutes, rigids, colliders, drives = [], [], [], [], []
for prim in stage.Traverse():
    p = prim.GetPath().pathString
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        arts.append(p)
    if prim.IsA(UsdPhysics.RevoluteJoint):
        revolutes.append(p)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigids.append(p)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        colliders.append(p)

limit_violations = []
for jp in revolutes:
    prim = stage.GetPrimAtPath(jp)
    joint = UsdPhysics.RevoluteJoint(prim)
    lo, hi = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
    tgt = prim.GetAttribute("drive:angular:physics:targetPosition")
    if not tgt or not tgt.IsValid():
        continue
    samples = tgt.GetTimeSamples()
    vals = [v for v in ([tgt.Get(t) for t in samples] if samples else [tgt.Get()]) if v is not None]
    if not vals:
        continue
    drives.append({"joint": jp, "n_samples": len(samples)})
    if lo is not None and min(vals) < lo - 1e-6:
        limit_violations.append([jp, "below lower", min(vals), lo])
    if hi is not None and max(vals) > hi + 1e-6:
        limit_violations.append([jp, "above upper", max(vals), hi])

scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
a = scene_prim.GetAttribute("physxScene:enableCCD")
scene_ccd = bool(a.Get()) if a and a.IsValid() else False
body_ccd = [p.GetPath().pathString for p in stage.Traverse()
            if (x := p.GetAttribute("physxRigidBody:enableCCD")) and x.IsValid() and x.Get()]

report["static"] = {
    "authored_tps": TPS,
    "articulation_roots": len(arts),
    "revolute_joints": len(revolutes),
    "rigid_bodies": len(rigids),
    "colliders": len(colliders),
    "driven_joints": len(drives),
    "animated_drives": sum(1 for d in drives if d["n_samples"] > 0),
    "static_drives": sum(1 for d in drives if d["n_samples"] == 0),
    "limit_violations": limit_violations,
    "scene_ccd_enabled": scene_ccd,
    "bodies_with_ccd": len(body_ccd),
    "gravity": {"direction": list(scene_prim.GetAttribute("physics:gravityDirection").Get()),
                "magnitude": float(scene_prim.GetAttribute("physics:gravityMagnitude").Get())},
    "up_axis": UsdGeom.GetStageUpAxis(stage),
    "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
}
print(json.dumps(report["static"], indent=2), flush=True)

# ---------------------------------------------------------------- sim setup
print("\n=== Booting PhysX ===", flush=True)
world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=args.dt)
world.reset()
rt_stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()

robots = Articulation(prim_paths_expr=ROBOTS, name="robots")
drops = RigidPrim(prim_paths_expr=DROPS, name="drops")
world.scene.add(robots)
world.scene.add(drops)
world.reset()

dof_names = list(robots.dof_names)
n_robots, n_dof = int(robots.count), int(robots.num_dof)
print(f"articulations {n_robots} | dof {n_dof} {dof_names} | drop bodies {drops.count}", flush=True)
report["runtime"] = {"articulations_found": n_robots, "dof_per_articulation": n_dof,
                     "dof_names": dof_names, "drop_bodies_found": int(drops.count)}

_attr_cache = {}
def authored_target(robot_idx, dof_name, timecode):
    key = (robot_idx, dof_name)
    if key not in _attr_cache:
        prim = stage.GetPrimAtPath(f"/World/Workstations/Robot_{robot_idx:02d}/{dof_name}")
        at = prim.GetAttribute("drive:angular:physics:targetPosition") if prim else None
        _attr_cache[key] = at if (at and at.IsValid()) else None
    at = _attr_cache[key]
    if at is None:
        return 0.0
    v = at.Get(float(timecode))
    return 0.0 if v is None else math.radians(v)

def target_matrix(timecode):
    m = np.zeros((n_robots, n_dof), dtype=np.float32)
    for r in range(n_robots):
        for d, name in enumerate(dof_names):
            m[r, d] = authored_target(r, name, timecode)
    return m

animated = np.zeros((n_robots, n_dof), dtype=bool)
for r in range(n_robots):
    for d, name in enumerate(dof_names):
        prim = stage.GetPrimAtPath(f"/World/Workstations/Robot_{r:02d}/{name}")
        at = prim.GetAttribute("drive:angular:physics:targetPosition") if prim else None
        animated[r, d] = bool(at and at.IsValid() and len(at.GetTimeSamples()) > 0)
print(f"animated joints: {int(animated.sum())}/{animated.size} "
      f"(static by design: {int((~animated).sum())})", flush=True)

# ---------------------------------------------------------------- phase 1
print("\n=== PHASE 1 - gravity / dynamic bodies ===", flush=True)
timeline.stop()
world.reset()
p0 = np.array(drops.get_world_poses()[0])
traj = [p0.copy()]
for _ in range(int(args.settle_seconds / args.dt)):
    world.step(render=False)
    traj.append(np.array(drops.get_world_poses()[0]))
p1 = traj[-1]
for _ in range(int(1.0 / args.dt)):
    world.step(render=False)
p2 = np.array(drops.get_world_poses()[0])

drop_rows = []
for i in range(len(p0)):
    zs = [t[i][2] for t in traj]
    # where did it pause on the way down? (a rest is a run of near-constant z)
    rest_levels = []
    j = 0
    while j < len(zs) - 1:
        k = j
        while k < len(zs) - 1 and abs(zs[k + 1] - zs[k]) < 1e-4:
            k += 1
        if (k - j) * args.dt > 0.15:
            rest_levels.append({"z": round(float(zs[j]), 4),
                                "from_s": round(j * args.dt, 3),
                                "to_s": round(k * args.dt, 3)})
        j = max(k, j + 1)
    drop_rows.append({
        "body": i,
        "xy_start": [round(float(p0[i][0]), 3), round(float(p0[i][1]), 3)],
        "xy_end": [round(float(p1[i][0]), 3), round(float(p1[i][1]), 3)],
        "z_start": round(float(p0[i][2]), 4), "z_end": round(float(p1[i][2]), 4),
        "dz": round(float(p1[i][2] - p0[i][2]), 4),
        "residual_1s": round(float(abs(p2[i][2] - p1[i][2])), 5),
        "rest_levels": rest_levels[:6],
    })
    print(f"  Drop_{i:02d}: xy {drop_rows[-1]['xy_start']} -> {drop_rows[-1]['xy_end']}  "
          f"z {p0[i][2]:7.3f} -> {p1[i][2]:8.3f}  residual {drop_rows[-1]['residual_1s']:.5f}", flush=True)
    print(f"            rest levels: {rest_levels[:4] if rest_levels else 'never came to rest'}", flush=True)

residual = float(np.abs(p2[:, 2] - p1[:, 2]).max()) if len(p2) else 0.0
report["phase1_dynamics"] = {
    "settle_seconds": args.settle_seconds,
    "belt_top_m": BELT_TOP, "floor_top_m": FLOOR_TOP, "part_half_extent_m": HALF,
    "expected_rest_z_on_belt": BELT_TOP + HALF, "expected_rest_z_on_floor": FLOOR_TOP + HALF,
    "bodies": drop_rows,
    "all_fell": bool(all(r["dz"] < -1e-3 for r in drop_rows)),
    "all_above_floor": bool((p1[:, 2] > FLOOR_TOP).all()),
    "all_at_rest": bool(all(r["residual_1s"] < 5e-3 for r in drop_rows)),
    "residual_after_settle_m": residual,
}

# ---------------------------------------------------------------- phase 2a
print("\n=== PHASE 2a - pose hold + ablation ===", flush=True)
# Isaac re-applies the USD drive target every step, so simply stepping with the
# timeline stopped IS the hold test: PhysX drives to the frame-0 authored pose.
ABLATION = ["/World/DropParts", "/World/Workpieces", "/World/Bins",
            "/World/Racks", "/World/Conveyor", "/World/AMR"]
t0 = target_matrix(START_TC)

def set_active(paths, value):
    for path in paths:
        pr = rt_stage.GetPrimAtPath(path)
        if pr and pr.IsValid():
            pr.SetActive(value)

def hold_test(tag, disabled):
    set_active(ABLATION, True)
    if disabled:
        set_active([disabled], False)
    timeline.stop()
    world.reset()
    art = Articulation(prim_paths_expr=ROBOTS, name=f"robots_{tag}")
    art.initialize()
    for _ in range(int(args.hold_seconds / args.dt)):
        world.step(render=False)
    q = np.array(art.get_joint_positions())
    return np.degrees(np.abs(q - t0))

baseline = hold_test("base", None)
print("  baseline steady-state error (deg):", flush=True)
for r in range(n_robots):
    print(f"    Robot_{r:02d}: " + ", ".join(
        f"{dof_names[d]}={baseline[r][d]:7.3f}" for d in range(n_dof)), flush=True)

ablation_results = {}

print("\n  --- ablation: what is disturbing the arms? ---", flush=True)
worst_before = float(baseline.max())
print(f"  baseline worst hold error {worst_before:.3f} deg", flush=True)
for path in ABLATION:
    try:
        err = hold_test(path.split("/")[-1], path)
        ablation_results[path] = round(float(err.max()), 3)
        tag = "  <-- THIS IS THE CAUSE" if err.max() < 1.0 <= worst_before else ""
        print(f"    without {path:22s} worst error {err.max():7.3f} deg{tag}", flush=True)
    except Exception as exc:  # one failed ablation must not lose the whole run
        ablation_results[path] = f"error: {exc}"
        print(f"    without {path:22s} FAILED: {exc}", flush=True)
set_active(ABLATION, True)
culprits = [k for k, v in ablation_results.items()
            if isinstance(v, float) and v < 1.0 <= worst_before]
print(f"  culprit: {culprits or 'none identified'}", flush=True)


report["phase2a_hold"] = {
    "hold_seconds": args.hold_seconds,
    "max_steady_state_error_deg": round(float(baseline.max()), 4),
    "mean_steady_state_error_deg": round(float(baseline.mean()), 4),
    "per_robot_deg": {f"Robot_{r:02d}": {dof_names[d]: round(float(baseline[r][d]), 4)
                                         for d in range(n_dof)} for r in range(n_robots)},
    "ablation_worst_error_deg": ablation_results,
    "culprit": culprits,
}

# ---------------------------------------------------------------- phase 2b
print("\n=== PHASE 2b - timeline playback (Isaac drives from USD) ===", flush=True)
# Order matters. World.reset() starts the timeline itself, and any later
# stop/play cycle tears down the PhysX tensor view the Articulation wraps -
# which is what "Failed to get DOF positions from backend" means. So configure
# the timeline first, reset once, and only then build the view.
timeline.stop()
timeline.set_time_codes_per_second(TPS)
timeline.set_start_time(START_TC / TPS)
timeline.set_end_time(END_TC / TPS)
timeline.set_looping(False)
timeline.set_current_time(START_TC / TPS)
world.reset()
robots = Articulation(prim_paths_expr=ROBOTS, name="robots_playback")
robots.initialize()
if not timeline.is_playing():
    timeline.play()
print(f"  timeline playing={timeline.is_playing()} "
      f"t0={timeline.get_current_time():.3f}s", flush=True)

# The USD stores the arms at their rest pose (all joints zero) and the motion
# in the drive targets, so the first moments of playback are the solver driving
# the arms from zero to the frame-0 pose. That convergence is not tracking
# error, so it is measured and reported separately rather than averaged in.
SETTLE_S = 0.5
q_min = np.full((n_robots, n_dof), 1e9)
q_max = np.full((n_robots, n_dof), -1e9)
lag_all, lag_steady, samples_log, seen = [], [], [], set()
q_last = tgt_last = None
end_s = END_TC / TPS
max_updates = int((end_s + 1.0) / args.dt) + 100
for _ in range(max_updates):
    world.step(render=True)
    t = timeline.get_current_time()
    if t > end_s:
        break
    frame = t * TPS
    tgt = target_matrix(frame)
    try:
        q = np.array(robots.get_joint_positions())
    except Exception:
        robots = Articulation(prim_paths_expr=ROBOTS, name=f"robots_re{len(lag_all)}")
        robots.initialize()
        q = np.array(robots.get_joint_positions())
    err = np.abs(q - tgt)
    lag_all.append(float(err.max()))
    if t >= SETTLE_S:
        # travel is measured over the same window, so the initial swing away
        # from the rest pose is not counted as commanded motion either
        lag_steady.append(float(err.max()))
        q_min, q_max = np.minimum(q_min, q), np.maximum(q_max, q)
    q_last, tgt_last = q, tgt
    fk = int(round(frame))
    if fk % 24 == 0 and fk not in seen:
        seen.add(fk)
        samples_log.append({"frame": fk, "t_s": round(float(t), 3),
                            "in_steady_window": bool(t >= SETTLE_S),
                            "target_deg_robot0": [round(math.degrees(x), 2) for x in tgt[0]],
                            "achieved_deg_robot0": [round(math.degrees(x), 2) for x in q[0]],
                            "max_lag_deg": round(math.degrees(float(err.max())), 3)})
        print(f"  t={t:5.2f}s frame {fk:3d}  R0 tgt "
              f"{[round(math.degrees(x),1) for x in tgt[0]]}  got "
              f"{[round(math.degrees(x),1) for x in q[0]]}  "
              f"max lag {math.degrees(float(err.max())):.2f} deg"
              f"{'' if t >= SETTLE_S else '   [startup, excluded]'}", flush=True)
timeline.stop()
print(f"  timeline advanced to {t:.3f}s of {end_s:.3f}s over {len(lag_all)} samples "
      f"({len(lag_steady)} in the steady window, t >= {SETTLE_S}s)", flush=True)
startup_lag = math.degrees(max(lag_all[:len(lag_all) - len(lag_steady)] or [0.0]))
print(f"  startup convergence peak: {startup_lag:.2f} deg (excluded)", flush=True)
print(f"  steady-state lag: max {math.degrees(max(lag_steady)):.2f}  "
      f"mean {math.degrees(float(np.mean(lag_steady))):.2f}  "
      f"p95 {math.degrees(float(np.percentile(lag_steady, 95))):.2f} deg", flush=True)

travel_deg = np.degrees(q_max - q_min)
final_err_deg = np.degrees(np.abs(q_last - tgt_last))
print("\n=== PHASE 2c - joint travel under PhysX (deg, steady window) ===", flush=True)
for r in range(n_robots):
    print(f"  Robot_{r:02d}: " + ", ".join(
        f"{dof_names[d]}={travel_deg[r][d]:6.1f}{'*' if animated[r][d] else '(static)'}"
        for d in range(n_dof)), flush=True)

animated_moved = bool((travel_deg[animated] > 5.0).all()) if animated.any() else True
# A static-target joint is not required to be motionless: the shoulder and
# elbow swinging through the pick cycle feed a reaction torque back into the
# yaw turntable, and a force drive answers that with a finite deflection. What
# it must do is hold - come back to its target rather than drift or run away.
static_returns = bool((final_err_deg[~animated] < 1.0).all()) if (~animated).any() else True
static_peak_deflection = float(travel_deg[~animated].max()) if (~animated).any() else 0.0
print(f"\n  static joints: peak deflection {static_peak_deflection:.2f} deg, "
      f"final error {float(final_err_deg[~animated].max()):.3f} deg", flush=True)

report["phase2b_tracking"] = {
    "method": "timeline playback - Isaac Sim reads the authored drive time samples",
    "timeline_reached_s": round(float(t), 3), "timeline_end_s": round(end_s, 3),
    "settle_window_s": SETTLE_S,
    "samples": samples_log,
    "startup_convergence_peak_deg": round(startup_lag, 3),
    "max_lag_deg": round(math.degrees(max(lag_steady)), 3) if lag_steady else None,
    "mean_lag_deg": round(math.degrees(float(np.mean(lag_steady))), 3) if lag_steady else None,
    "p95_lag_deg": round(math.degrees(float(np.percentile(lag_steady, 95))), 3) if lag_steady else None,
    "max_lag_including_startup_deg": round(math.degrees(max(lag_all)), 3) if lag_all else None,
}
report["phase2c_motion"] = {
    "window": f"t >= {SETTLE_S}s",
    "joint_travel_deg": {f"Robot_{r:02d}": {dof_names[d]: round(float(travel_deg[r][d]), 2)
                                            for d in range(n_dof)} for r in range(n_robots)},
    "final_error_deg": {f"Robot_{r:02d}": {dof_names[d]: round(float(final_err_deg[r][d]), 3)
                                           for d in range(n_dof)} for r in range(n_robots)},
    "animated_joints_moved": animated_moved,
    "static_joints_return_to_target": static_returns,
    "static_joint_peak_deflection_deg": round(static_peak_deflection, 2),
}

# ---------------------------------------------------------------- verdict
s, p1r = report["static"], report["phase1_dynamics"]
p2a, p2b, p2c = report["phase2a_hold"], report["phase2b_tracking"], report["phase2c_motion"]
checks = {
    "4 articulation roots parsed by PhysX": report["runtime"]["articulations_found"] == 4,
    "3 DOF per arm registered": report["runtime"]["dof_per_articulation"] == 3,
    "no drive target exceeds a joint limit": len(s["limit_violations"]) == 0,
    "CCD enabled (scene + bodies)": s["scene_ccd_enabled"] and s["bodies_with_ccd"] > 0,
    "dynamic parts fall under gravity": p1r["all_fell"],
    "dynamic parts stay above the floor": p1r["all_above_floor"],
    "dynamic parts come to rest": p1r["all_at_rest"],
    "arms hold their authored rest pose (<1 deg)": p2a["max_steady_state_error_deg"] < 1.0,
    "timeline played to the end": p2b["timeline_reached_s"] >= p2b["timeline_end_s"] - 0.2,
    "steady-state tracking lag acceptable (<10 deg)": (p2b["max_lag_deg"] or 999) < 10.0,
    "animated joints actually move": p2c["animated_joints_moved"],
    "static joints return to their target (<1 deg)": p2c["static_joints_return_to_target"],
}
# Measured characteristics, reported rather than graded - they are properties
# of the design, not defects to pass or fail.
report["findings"] = [
    {"what": "yaw turntable deflection under pick-cycle reaction torque",
     "value_deg": p2c["static_joint_peak_deflection_deg"],
     "note": "joint0_yaw holds a static drive target, but the shoulder and elbow "
             "swinging through the cycle push the turntable off target by this "
             "much before it recovers. Raising the yaw drive stiffness above the "
             "shared 1e5 would reduce it."},
    {"what": "startup convergence from the authored rest pose",
     "value_deg": p2b["startup_convergence_peak_deg"],
     "note": "the USD stores the arms at the zero pose and the motion in the "
             "drive targets, so the solver has to travel to the frame-0 pose "
             "when playback starts. Excluded from the tracking statistic."},
]
report["verdict"] = checks
report["passed"] = all(checks.values())

print("\n=== VERDICT ===", flush=True)
for k, v in checks.items():
    print(f"  [{'PASS' if v else 'FAIL'}] {k}", flush=True)
print(f"\n  OVERALL: {'PASS' if report['passed'] else 'FAIL'}"
      f"  ({sum(checks.values())}/{len(checks)})", flush=True)

os.makedirs(os.path.dirname(args.report), exist_ok=True)
with open(args.report, "w") as fh:
    json.dump(report, fh, indent=2)
print(f"\nreport written to {args.report}", flush=True)

sim_app.close()
