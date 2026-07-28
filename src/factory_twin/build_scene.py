"""Assemble a full factory production-line digital twin as an OpenUSD stage.

Run:  python -m factory_twin.build_scene --out output/factory_twin.usda

Layout (Z-up, metres — Isaac Sim / robotics convention):

    +Y  ── material flow direction ──▶
    │
    │   [Rack]  [Rack]  [Rack]        <- inbound storage (left)
    │   ══════ conveyor belt ══════   <- centre, runs along +Y
    │      R    R    R    R           <- robot-arm workstations (right)
    │              [AMR path]         <- AMR loop feeding the line
    └──── floor ────────────────────
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from . import assets, physics


def _add_amr_path(stage, path, points):
    """Represent the AMR route as a BasisCurves prim — a first-class, editable
    'ground truth' path an operator (or an Isaac Sim nav stack) can follow."""
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    curve.CreateWidthsAttr([0.15] * len(points))
    curve.SetWidthsInterpolation("vertex")
    return curve


def build(out_path: str) -> str:
    stage = Usd.Stage.CreateNew(out_path)

    # --- stage metadata: the contract every downstream tool reads first ---
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim("/World"))
    UsdGeom.Xform.Define(stage, "/World")

    # --- materials library ---
    mats = "/World/Materials"
    UsdGeom.Scope.Define(stage, mats)
    m_floor = assets.create_material(stage, f"{mats}/Floor", (0.22, 0.22, 0.25), roughness=0.9)
    m_steel = assets.create_material(stage, f"{mats}/Steel", (0.55, 0.57, 0.6), roughness=0.35, metallic=0.9)
    m_belt = assets.create_material(stage, f"{mats}/Belt", (0.1, 0.1, 0.12), roughness=0.7)
    m_robot = assets.create_material(stage, f"{mats}/Robot", (0.95, 0.55, 0.1), roughness=0.5)
    m_rack = assets.create_material(stage, f"{mats}/Rack", (0.2, 0.35, 0.75), roughness=0.6)
    m_amr = assets.create_material(stage, f"{mats}/AMR", (0.9, 0.85, 0.1), roughness=0.5)
    m_part = assets.create_material(stage, f"{mats}/Part", (0.8, 0.2, 0.2), roughness=0.4)

    LINE_LEN = 20.0  # metres of production line

    # --- physics: gravity + a ground collider ---
    physics.setup_physics_scene(stage)

    # --- floor (also a static collider so bodies rest on it) ---
    floor = assets.build_floor(stage, "/World/Floor", width=16, length=LINE_LEN + 4)
    assets.bind(floor, m_floor)
    physics.make_static_collider(floor)

    # --- conveyor down the centre ---
    conv = assets.build_conveyor(stage, "/World/Conveyor", length=LINE_LEN)
    for child in conv.GetChildren():
        assets.bind(child, m_steel if "Leg" in child.GetName() else m_belt)

    # --- workpieces spaced along the belt ---
    parts = UsdGeom.Xform.Define(stage, "/World/Workpieces").GetPrim()
    for i in range(6):
        y = -LINE_LEN / 2 + (i + 0.5) * (LINE_LEN / 6)
        p = assets.build_workpiece(stage, f"/World/Workpieces/Part_{i:02d}",
                                   translate=(0, y, 1.05))
        assets.bind(p, m_part)

    # --- robot-arm workstations along the +X side of the belt ---
    # each is a UsdPhysics articulation (see physics.py) posed slightly
    # differently so the line looks like it is working.
    UsdGeom.Xform.Define(stage, "/World/Workstations")
    poses = [(20, -40, 80), (-15, -55, 60), (30, -30, 90), (0, -50, 70)]
    for i in range(4):
        y = -LINE_LEN / 2 + (i + 0.5) * (LINE_LEN / 4)
        root = physics.build_robot_arm_articulated(
            stage, f"/World/Workstations/Robot_{i:02d}",
            mats={"steel": m_steel, "robot": m_robot}, pose=poses[i])
        xf = UsdGeom.Xformable(root)
        xf.AddTranslateOp().Set(Gf.Vec3d(2.0, y, 0))
        xf.AddRotateZOp().Set(-90)  # turn to face the belt (-X side)

    # --- storage racks defined ONCE as a prototype, then instanced ---
    # This is how digital twins stay light at scale: one authored rack,
    # many cheap instanceable references.
    proto = "/World/_Prototypes/Rack"
    UsdGeom.Scope.Define(stage, "/World/_Prototypes")
    rack_proto = assets.build_rack(stage, proto, bays=3, levels=3)
    for child in rack_proto.GetChildren():
        assets.bind(child, m_rack)
    stage.GetPrimAtPath(proto.rsplit("/", 1)[0]).SetActive(False)  # hide raw prototype

    racks = UsdGeom.Xform.Define(stage, "/World/Racks").GetPrim()
    for i in range(3):
        y = -LINE_LEN / 2 + (i + 0.5) * (LINE_LEN / 3)
        inst = stage.DefinePrim(f"/World/Racks/Rack_{i:02d}")
        inst.GetReferences().AddInternalReference(Sdf.Path(proto))
        inst.SetInstanceable(True)
        UsdGeom.Xformable(inst).AddTranslateOp().Set(Gf.Vec3d(-4.5, y, 0))

    # --- AMR + its route ---
    amr = assets.build_amr(stage, "/World/AMR")
    UsdGeom.Xformable(amr).AddTranslateOp().Set(Gf.Vec3d(-2.2, -LINE_LEN / 2, 0))
    for child in amr.GetChildren():
        assets.bind(child, m_amr)
    half = LINE_LEN / 2
    _add_amr_path(stage, "/World/AMR_Path", [
        (-2.2, -half, 0.02), (-2.2, half, 0.02),
        (-4.5, half, 0.02), (-4.5, -half, 0.02), (-2.2, -half, 0.02),
    ])

    # --- lighting: a dome for ambient fill + a key light for shadows ---
    lights = UsdGeom.Xform.Define(stage, "/World/Lights").GetPrim()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(800.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(0.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 25))

    stage.GetRootLayer().documentation = (
        "Factory production-line digital twin (OpenUSD MVP). "
        "Z-up, metres. Generated by factory_twin.build_scene."
    )
    stage.Save()
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="output/factory_twin.usda",
                        help="Output USD path (.usda text or .usdc binary)")
    args = parser.parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    out = build(args.out)
    stage = Usd.Stage.Open(out)
    n = len(list(stage.Traverse()))
    print(f"✓ wrote {out}  ({n} prims)")


if __name__ == "__main__":
    main()
