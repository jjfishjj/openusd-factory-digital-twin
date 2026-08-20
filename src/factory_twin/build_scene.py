"""Assemble a full factory production-line digital twin as an OpenUSD stage.

The whole layout is data-driven from a YAML config (see config/factory.yaml):
edit the numbers, re-run, get a different plant.

Run:  factory-twin-build --config config/factory.yaml --out output/factory_twin.usda

Layout (Z-up, metres — Isaac Sim / robotics convention):

    +Y  ── material flow direction ──▶
    │
    │   [Rack]  [Rack]  [Rack]        <- inbound storage (-X)
    │   ══════ conveyor belt ══════   <- centre, animated parts ride +Y
    │      R    R    R    R           <- robot-arm articulations (+X)
    │              [AMR path]         <- AMR loop feeding the line
    └──── floor ────────────────────

Workpieces are kinematic rigid bodies driven by time-sampled transforms, so the
conveyor flow both animates (readable without a GPU) and interacts with PhysX in
Isaac Sim. A few loose dynamic parts drop onto the belt under gravity there too.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from . import assets, kinematics, physics

# ---------------------------------------------------------------------------
# Arm pick-and-place cycle (shared by the arm drives AND the carried part, so
# the grasp stays exact). Keyframes are (phase u, shoulder deg, elbow deg);
# yaw is per-robot and added separately. FK-verified gripper targets:
#   PICK  -> belt  (x≈0.1, z≈1.0)      PLACE -> +X bin (x≈3.5, z≈1.2)
# ---------------------------------------------------------------------------
_CYCLE = [(0.00, -20, 90), (0.20, -80, 10), (0.40, -20, 90),
          (0.60, 70, 0), (0.80, -20, 90), (1.00, -20, 90)]
_GRASP_U = (0.20, 0.60)  # the part is attached to the gripper over this window

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "factory.yaml"


def _wrap(y, span):
    """Wrap y into [-span/2, span/2) — the conveyor 'recycle' at the belt end."""
    return ((y + span / 2) % span) - span / 2


def _add_amr_path(stage, path, points):
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    curve.CreateWidthsAttr([0.15] * len(points))
    curve.SetWidthsInterpolation("vertex")
    return curve


def _robot_y(cfg, i):
    L, rc = cfg["line"]["length"], cfg["robots"]
    return -L / 2 + (i + 0.5) * (L / rc["count"])


def _pose_at(u):
    """Piecewise-linear (shoulder, elbow) angle at cycle phase u∈[0,1]."""
    for j in range(len(_CYCLE) - 1):
        u0, a0, b0 = _CYCLE[j]
        u1, a1, b1 = _CYCLE[j + 1]
        if u0 <= u <= u1:
            t = (u - u0) / (u1 - u0) if u1 > u0 else 0.0
            return a0 + (a1 - a0) * t, b0 + (b1 - b0) * t
    return _CYCLE[-1][1], _CYCLE[-1][2]


def _cycle_u(cfg, i, f, n_frames):
    """Cycle phase of robot i at frame f (one pick-place per loop, phased)."""
    return (f / n_frames + i / cfg["robots"]["count"]) % 1.0


def _gripper_world(cfg, i, u):
    """World-space gripper tip of robot i at cycle phase u, via forward
    kinematics on the SAME joint angles the arm drives use."""
    yaw = cfg["robots"]["poses"][i % len(cfg["robots"]["poses"])][0]
    q1, q2 = _pose_at(u)
    tip = kinematics.arm_fk(yaw, q1, q2)[-1]
    root = Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(cfg["robots"]["offset_x"], _robot_y(cfg, i), 0))
    return root.Transform(Gf.Vec3d(float(tip[0]), float(tip[1]), float(tip[2])))


def _passthrough_parts(stage, cfg, belt_top, m_part, n_frames):
    """Background parts that ride the belt straight through, unpicked."""
    L, fps = cfg["line"]["length"], cfg["animation"]["fps"]
    speed = cfg["workpieces"]["speed"]
    count = cfg["workpieces"].get("passthrough", 0)
    for i in range(count):
        p = f"/World/Workpieces/Pass_{i:02d}"
        body = UsdGeom.Xform.Define(stage, p).GetPrim()
        physics.make_kinematic_body(body, mass=2.0)
        trans = UsdGeom.Xformable(body).AddTranslateOp()
        y0 = -L / 2 + i * (L / max(1, count))
        for f in range(n_frames + 1):
            trans.Set(Gf.Vec3d(0, _wrap(y0 + speed * (f / fps), L), belt_top), time=f)
        geo = assets.add_box(stage, f"{p}/geo", (0.4, 0.4, 0.3))
        physics.add_collider(geo)
        assets.bind(geo, m_part)


def _pick_parts(stage, cfg, belt_top, m_pick, m_bin, n_frames):
    """One choreographed part per robot: ride the belt to the station, get
    carried by the gripper (baked to follow the FK during the grasp window),
    and settle into an output bin on the +X side. Kinematic, so the pick reads
    identically in the GPU-free GIF and in Isaac Sim."""
    rc = cfg["robots"]
    part_step = max(1, n_frames // 60)
    approach = 6.0  # metres of belt travel before the pickup
    UsdGeom.Xform.Define(stage, "/World/Bins")
    for i in range(rc["count"]):
        y_i = _robot_y(cfg, i)
        binpos = _gripper_world(cfg, i, _GRASP_U[1])  # release point = bin

        # output bin on the floor beneath the release point
        bin_geo = assets.add_box(stage, f"/World/Bins/Bin_{i:02d}",
                                 (0.8, 0.8, 0.9),
                                 translate=(binpos[0], y_i, 0.45))
        physics.make_static_collider(bin_geo)
        assets.bind(bin_geo, m_bin)

        # the picked part
        p = f"/World/Workpieces/Pick_{i:02d}"
        body = UsdGeom.Xform.Define(stage, p).GetPrim()
        physics.make_kinematic_body(body, mass=2.0)
        trans = UsdGeom.Xformable(body).AddTranslateOp()
        for f in range(0, n_frames + 1, part_step):
            u = _cycle_u(cfg, i, f, n_frames)
            if u < _GRASP_U[0]:                       # riding the belt to the arm
                frac = u / _GRASP_U[0]
                pos = Gf.Vec3d(0.1, y_i - (1 - frac) * approach, belt_top)
            elif u <= _GRASP_U[1]:                    # grasped: follow the gripper
                g = _gripper_world(cfg, i, u)
                pos = Gf.Vec3d(g[0], g[1], g[2] - 0.05)
            else:                                     # released: rest in the bin
                pos = Gf.Vec3d(binpos[0], binpos[1], 1.0)
            trans.Set(pos, time=f)
        geo = assets.add_box(stage, f"{p}/geo", (0.35, 0.35, 0.3))
        physics.add_collider(geo)
        assets.bind(geo, m_pick)


def _drop_parts(stage, cfg, belt_top, m_drop):
    """Loose dynamic rigid bodies suspended above the belt — they fall and settle
    once PhysX simulates (Isaac Sim). Static in the authored USD."""
    L = cfg["line"]["length"]
    n = cfg["physics"].get("dynamic_parts", 0)
    drop_h = cfg["physics"].get("drop_height", 2.0)
    if n <= 0:
        return
    UsdGeom.Xform.Define(stage, "/World/DropParts")
    for i in range(n):
        y = -L / 2 + (i + 1) * (L / (n + 1))
        body = UsdGeom.Xform.Define(stage, f"/World/DropParts/Drop_{i:02d}").GetPrim()
        UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(0, y, belt_top + drop_h))
        physics.make_dynamic_body(body, mass=1.5)
        geo = assets.add_box(stage, f"/World/DropParts/Drop_{i:02d}/geo", (0.35, 0.35, 0.35))
        physics.add_collider(geo)
        assets.bind(geo, m_drop)


def _animate_arms(stage, cfg, n_frames):
    """Author the pick-and-place trajectory on each arm's shoulder/elbow drives,
    from the shared _CYCLE keyframes and phased per robot. Yaw stays constant.
    This is the same joint schedule the carried part follows via FK."""
    rc = cfg["robots"]
    key_step = max(1, n_frames // 48)
    for i in range(rc["count"]):
        base = f"/World/Workstations/Robot_{i:02d}"
        yaw = rc["poses"][i % len(rc["poses"])][0]
        physics.set_drive_target(stage.GetPrimAtPath(f"{base}/joint0_yaw"), yaw)
        j1 = stage.GetPrimAtPath(f"{base}/joint1_shoulder")
        j2 = stage.GetPrimAtPath(f"{base}/joint2_elbow")
        for f in range(0, n_frames + 1, key_step):
            q1, q2 = _pose_at(_cycle_u(cfg, i, f, n_frames))
            physics.set_drive_target(j1, q1, time=f)
            physics.set_drive_target(j2, q2, time=f)


def build(cfg: dict, out_path: str) -> str:
    stage = Usd.Stage.CreateNew(out_path)

    # --- stage metadata: the contract every downstream tool reads first ---
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim("/World"))
    UsdGeom.Xform.Define(stage, "/World")

    L = cfg["line"]["length"]
    floor_w = cfg["line"]["floor_width"]
    belt_h = cfg["conveyor"]["height"]
    belt_top = belt_h + 0.075 + 0.15  # bed top + half part height

    # --- materials ---
    mats = "/World/Materials"
    UsdGeom.Scope.Define(stage, mats)
    m_floor = assets.create_material(stage, f"{mats}/Floor", (0.22, 0.22, 0.25), roughness=0.9)
    m_steel = assets.create_material(stage, f"{mats}/Steel", (0.55, 0.57, 0.6), roughness=0.35, metallic=0.9)
    m_belt = assets.create_material(stage, f"{mats}/Belt", (0.1, 0.1, 0.12), roughness=0.7)
    m_robot = assets.create_material(stage, f"{mats}/Robot", (0.95, 0.55, 0.1), roughness=0.5)
    m_rack = assets.create_material(stage, f"{mats}/Rack", (0.2, 0.35, 0.75), roughness=0.6)
    m_amr = assets.create_material(stage, f"{mats}/AMR", (0.9, 0.85, 0.1), roughness=0.5)
    m_part = assets.create_material(stage, f"{mats}/Part", (0.8, 0.2, 0.2), roughness=0.4)
    m_pick = assets.create_material(stage, f"{mats}/Pick", (0.85, 0.5, 0.85), roughness=0.4)
    m_drop = assets.create_material(stage, f"{mats}/Drop", (0.2, 0.7, 0.35), roughness=0.5)
    m_bin = assets.create_material(stage, f"{mats}/Bin", (0.35, 0.3, 0.22), roughness=0.8)

    # --- physics scene + ground collider ---
    physics.setup_physics_scene(stage).CreateGravityMagnitudeAttr(cfg["physics"]["gravity"])

    floor = assets.build_floor(stage, "/World/Floor", width=floor_w, length=L + 4)
    assets.bind(floor, m_floor)
    physics.make_static_collider(floor)

    # --- conveyor (bed is a static collider so parts ride / land on it) ---
    conv = assets.build_conveyor(stage, "/World/Conveyor", length=L,
                                 width=cfg["conveyor"]["width"], height=belt_h)
    for child in conv.GetChildren():
        if "Leg" in child.GetName():
            assets.bind(child, m_steel)
        else:
            assets.bind(child, m_belt)
            physics.make_static_collider(child)

    # --- timeline metadata (one conveyor loop = one belt-length of travel) ---
    fps = cfg["animation"]["fps"]
    n_frames = max(1, round(fps * L / cfg["workpieces"]["speed"]))
    stage.SetTimeCodesPerSecond(fps)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(n_frames)

    # --- parts: background flow, choreographed picks (+ bins), and drops ---
    UsdGeom.Xform.Define(stage, "/World/Workpieces")
    _passthrough_parts(stage, cfg, belt_top, m_part, n_frames)
    _pick_parts(stage, cfg, belt_top, m_pick, m_bin, n_frames)
    _drop_parts(stage, cfg, belt_top, m_drop)

    # --- robot-arm articulations along the +X side ---
    UsdGeom.Xform.Define(stage, "/World/Workstations")
    rc = cfg["robots"]
    poses = rc["poses"]
    for i in range(rc["count"]):
        y = -L / 2 + (i + 0.5) * (L / rc["count"])
        root = physics.build_robot_arm_articulated(
            stage, f"/World/Workstations/Robot_{i:02d}",
            mats={"steel": m_steel, "robot": m_robot},
            pose=poses[i % len(poses)])
        xf = UsdGeom.Xformable(root)
        xf.AddTranslateOp().Set(Gf.Vec3d(rc["offset_x"], y, 0))
        # no yaw on the root: the arm pitches in the world X-Z plane so its
        # reach toward the belt (-X) is visible in the top-down preview.
    _animate_arms(stage, cfg, n_frames)

    # The choreographed parts are kinematic, so in PhysX they would shove the
    # arms and bulldoze the drop parts. Filter them against the simulated
    # bodies; see physics.isolate_choreographed_parts for the measurements.
    physics.isolate_choreographed_parts(
        stage,
        choreographed=["/World/Workpieces"],
        simulated=["/World/Workstations", "/World/DropParts"],
    )

    # --- storage racks: authored once, stamped as instanceable references ---
    proto = "/World/_Prototypes/Rack"
    UsdGeom.Scope.Define(stage, "/World/_Prototypes")
    rkc = cfg["racks"]
    rack_proto = assets.build_rack(stage, proto, bays=rkc["bays"], levels=rkc["levels"])
    for child in rack_proto.GetChildren():
        assets.bind(child, m_rack)
    stage.GetPrimAtPath("/World/_Prototypes").SetActive(False)

    UsdGeom.Xform.Define(stage, "/World/Racks")
    for i in range(rkc["count"]):
        y = -L / 2 + (i + 0.5) * (L / rkc["count"])
        inst = stage.DefinePrim(f"/World/Racks/Rack_{i:02d}")
        inst.GetReferences().AddInternalReference(Sdf.Path(proto))
        inst.SetInstanceable(True)
        UsdGeom.Xformable(inst).AddTranslateOp().Set(Gf.Vec3d(rkc["offset_x"], y, 0))

    # --- AMR + its route ---
    amr = assets.build_amr(stage, "/World/AMR")
    amr_x = cfg["amr"]["offset_x"]
    UsdGeom.Xformable(amr).AddTranslateOp().Set(Gf.Vec3d(amr_x, -L / 2, 0))
    for child in amr.GetChildren():
        assets.bind(child, m_amr)
    half = L / 2
    _add_amr_path(stage, "/World/AMR_Path", [
        (amr_x, -half, 0.02), (amr_x, half, 0.02),
        (rkc["offset_x"], half, 0.02), (rkc["offset_x"], -half, 0.02),
        (amr_x, -half, 0.02),
    ])

    # --- lighting ---
    UsdGeom.Xform.Define(stage, "/World/Lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(800.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(0.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 25))

    stage.GetRootLayer().documentation = (
        "Factory production-line digital twin (OpenUSD). Z-up, metres. "
        f"Config-driven, {n_frames}-frame conveyor loop. "
        "Generated by factory_twin.build_scene."
    )
    stage.Save()
    return out_path


def load_config(path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="YAML layout config")
    parser.add_argument("--out", default="output/factory_twin.usda",
                        help="Output USD path (.usda text or .usdc binary)")
    args = parser.parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    cfg = load_config(args.config)
    out = build(cfg, args.out)
    stage = Usd.Stage.Open(out)
    n = len(list(stage.Traverse()))
    print(f"✓ wrote {out}  ({n} prims, "
          f"frames 0–{int(stage.GetEndTimeCode())} @ {stage.GetTimeCodesPerSecond():g}fps)")


if __name__ == "__main__":
    main()
