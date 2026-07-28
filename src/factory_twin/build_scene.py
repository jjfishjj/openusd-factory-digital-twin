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

from . import assets, physics

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


def _animate_workpieces(stage, cfg, belt_top, m_part):
    """Kinematic parts riding the belt, driven by time samples so the flow loops
    seamlessly. Returns the frame count authored."""
    L = cfg["line"]["length"]
    count = cfg["workpieces"]["count"]
    speed = cfg["workpieces"]["speed"]
    fps = cfg["animation"]["fps"]

    n_frames = max(1, round(fps * L / speed))  # exactly one belt-length per loop
    stage.SetTimeCodesPerSecond(fps)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(n_frames)

    UsdGeom.Xform.Define(stage, "/World/Workpieces")
    for i in range(count):
        body = UsdGeom.Xform.Define(stage, f"/World/Workpieces/Part_{i:02d}").GetPrim()
        physics.make_kinematic_body(body, mass=2.0)
        trans = UsdGeom.Xformable(body).AddTranslateOp()
        y0 = -L / 2 + i * (L / count)
        for f in range(n_frames + 1):
            y = _wrap(y0 + speed * (f / fps), L)
            trans.Set(Gf.Vec3d(0, y, belt_top), time=f)
        # collision/visual geometry (scale lives on the child, never on a body)
        geo = assets.add_box(stage, f"/World/Workpieces/Part_{i:02d}/geo", (0.4, 0.4, 0.3))
        physics.add_collider(geo)
        assets.bind(geo, m_part)
    return n_frames


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
    m_drop = assets.create_material(stage, f"{mats}/Drop", (0.2, 0.7, 0.35), roughness=0.5)

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

    # --- animated + dynamic parts ---
    n_frames = _animate_workpieces(stage, cfg, belt_top, m_part)
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
        xf.AddRotateZOp().Set(-90)  # face the belt

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
