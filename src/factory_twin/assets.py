"""Reusable asset builders for the factory digital twin.

Every function takes a USD stage and a prim path, builds one piece of the
factory, and returns the created prim. Geometry is intentionally kept to simple
primitives (cubes, cylinders, capsules) — the goal of this MVP is to show the
OpenUSD *scene structure* of a digital twin, not high-fidelity meshes. Any RTX
machine running Omniverse can open the resulting .usd and swap these placeholder
shapes for real CAD assets later.
"""

from __future__ import annotations

from pxr import Gf, Sdf, UsdGeom, UsdShade


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def create_material(
    stage,
    path: str,
    color: tuple[float, float, float],
    roughness: float = 0.5,
    metallic: float = 0.0,
) -> UsdShade.Material:
    """Create a UsdPreviewSurface material. This is the portable PBR shader
    every USD-aware renderer (incl. Omniverse RTX) understands out of the box."""
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind(prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI(prim).Bind(material)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _set_xform(prim, translate=None, rotate=None, scale=None) -> None:
    """Apply a translate/rotate/scale to any Xformable prim, in that order."""
    xform = UsdGeom.Xformable(prim)
    if translate is not None:
        xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate is not None:  # XYZ euler degrees
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_box(stage, path, size_xyz, translate=(0, 0, 0), rotate=None):
    """A unit cube scaled to size_xyz (metres). Cube default edge length is 2,
    so we halve the scale to get the requested dimensions."""
    cube = UsdGeom.Cube.Define(stage, path)
    sx, sy, sz = size_xyz
    _set_xform(cube.GetPrim(), translate=translate, rotate=rotate,
               scale=(sx / 2.0, sy / 2.0, sz / 2.0))
    return cube.GetPrim()


def add_cylinder(stage, path, radius, height, translate=(0, 0, 0), axis="Z"):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    _set_xform(cyl.GetPrim(), translate=translate)
    return cyl.GetPrim()


# ---------------------------------------------------------------------------
# Factory assets
# ---------------------------------------------------------------------------
def build_floor(stage, path, width, length):
    """Factory floor as a thin box centred on the origin, top face at z=0."""
    return add_box(stage, path, (width, length, 0.1), translate=(0, 0, -0.05))


def build_conveyor(stage, path, length, width=1.2, height=0.8):
    """A conveyor belt: a bed on legs, running along +Y."""
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    add_box(stage, f"{path}/Bed", (width, length, 0.15),
            translate=(0, 0, height))
    # a leg every 2 metres
    n_legs = max(2, int(length // 2))
    for i in range(n_legs):
        y = -length / 2 + (i + 0.5) * (length / n_legs)
        add_box(stage, f"{path}/Leg_{i:02d}", (0.1, 0.1, height),
                translate=(0, y, height / 2))
    return root


def build_robot_arm(stage, path, reach=1.4):
    """A stylised 3-segment robot arm on a pedestal. Segments are separate prims
    so a future step could add UsdPhysics joints and articulate it in Isaac Sim."""
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    add_cylinder(stage, f"{path}/Base", radius=0.35, height=0.4,
                 translate=(0, 0, 0.2))
    add_cylinder(stage, f"{path}/Shoulder", radius=0.18, height=reach * 0.6,
                 translate=(0, 0, 0.4 + reach * 0.3))
    # forearm tilted outward toward the line
    forearm = add_box(stage, f"{path}/Forearm", (0.18, 0.18, reach * 0.5),
                      translate=(0, 0.25, 0.4 + reach * 0.6), rotate=(35, 0, 0))
    add_box(stage, f"{path}/Gripper", (0.22, 0.12, 0.12),
            translate=(0, 0.55, 0.4 + reach * 0.75))
    return root, forearm


def build_rack(stage, path, bays=3, levels=3):
    """A storage/pallet rack: uprights + horizontal beams per level.
    Defined once as a prototype and referenced instanceably by the caller."""
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    bay_w, level_h, depth = 1.2, 0.9, 1.0
    total_w = bays * bay_w
    height = levels * level_h
    # uprights
    for c in range(bays + 1):
        x = -total_w / 2 + c * bay_w
        for d in (-depth / 2, depth / 2):
            add_box(stage, f"{path}/Upright_{c}_{'f' if d > 0 else 'b'}",
                    (0.08, 0.08, height), translate=(x, d, height / 2))
    # beams + a pallet box on each level/bay
    for lv in range(levels):
        z = (lv + 1) * level_h
        add_box(stage, f"{path}/Beam_{lv}", (total_w, 0.08, 0.06),
                translate=(0, 0, z))
        for b in range(bays):
            bx = -total_w / 2 + (b + 0.5) * bay_w
            add_box(stage, f"{path}/Pallet_{lv}_{b}", (0.9, 0.8, 0.5),
                    translate=(bx, 0, z + 0.28))
    return root


def build_amr(stage, path):
    """An autonomous mobile robot (AMR) — a flat chassis carrying a bin."""
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    add_box(stage, f"{path}/Chassis", (0.8, 1.1, 0.25), translate=(0, 0, 0.15))
    add_box(stage, f"{path}/Payload", (0.6, 0.9, 0.4), translate=(0, 0, 0.45))
    return root


def build_workpiece(stage, path, translate=(0, 0, 0)):
    """A part travelling on the conveyor."""
    return add_box(stage, path, (0.4, 0.4, 0.3), translate=translate)
