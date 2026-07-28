"""UsdPhysics rigging for the factory twin.

Turns the visual robot arms into proper **articulations** — kinematic chains of
rigid bodies connected by revolute (hinge) joints — so that Isaac Sim / any
PhysX-backed Omniverse app loads them as articulated robots you can drive, not
as loose static meshes.

Chain (Z-up, arm at rest points straight up):

    base_link  ──fixed──▶ world          (bolted to the floor)
        │ j0  revolute, axis Z  (yaw turntable)
    link0
        │ j1  revolute, axis Y  (shoulder pitch)
    link1  (upper arm)
        │ j2  revolute, axis Y  (elbow pitch)
    link2  (forearm + gripper, one rigid body)

Every revolute joint carries angle limits and an angular DriveAPI so the arm
holds a commanded pose — the actuation Isaac Sim reads to control the robot.
"""

from __future__ import annotations

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

from . import assets

_IDENTITY = Gf.Quatf(1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Stage-level physics
# ---------------------------------------------------------------------------
def setup_physics_scene(stage, path="/World/PhysicsScene"):
    """Author the global physics scene: Earth gravity along -Z."""
    scene = UsdPhysics.Scene.Define(stage, path)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    return scene


def make_static_collider(prim):
    """Mark an existing visual prim (e.g. the floor) as a static collider —
    a CollisionAPI with no RigidBodyAPI means 'immovable world geometry'."""
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def add_collider(gprim):
    """Attach a collision shape to a gprim. If an ancestor has RigidBodyAPI the
    shape moves with that body; otherwise it is static world geometry."""
    UsdPhysics.CollisionAPI.Apply(gprim)
    return gprim


def make_kinematic_body(prim, mass=2.0):
    """A kinematic rigid body: follows its authored (time-sampled) transform
    exactly, yet still collides with and pushes dynamic bodies. This is how the
    conveyor workpieces both animate AND interact with the physics world.
    Apply add_collider() to its child geometry to give it a collision shape."""
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim


def make_dynamic_body(prim, mass=2.0):
    """A free dynamic rigid body — falls under gravity, collides, comes to rest.
    Used for the loose parts that drop onto the belt in Isaac Sim.
    Apply add_collider() to its child geometry to give it a collision shape."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim


# ---------------------------------------------------------------------------
# Rigid-body link + revolute joint helpers
# ---------------------------------------------------------------------------
def _rigid_link(stage, path, translate, mass):
    """A rigid-body link: an Xform (body, translate-only — never scale a body)
    holding collision/visual child geometry added by the caller."""
    link = UsdGeom.Xform.Define(stage, path)
    link.AddTranslateOp().Set(Gf.Vec3d(*translate))
    prim = link.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(mass)
    return prim


def _add_collision_box(stage, path, size_xyz, translate=(0, 0, 0)):
    """A visual + collision box, child of a link body."""
    prim = assets.add_box(stage, path, size_xyz, translate=translate)
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _add_collision_cyl(stage, path, radius, height):
    prim = assets.add_cylinder(stage, path, radius, height)
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _revolute(stage, path, body0, body1, anchor, axis,
              limits_deg, drive_target_deg, t0, t1,
              stiffness=1.0e5, damping=1.0e4):
    """Create a revolute joint between two links.

    anchor : joint pivot, expressed in the links' shared parent frame.
    t0, t1 : local translates of body0/body1 in that same parent frame.
    Because links are axis-aligned (identity rotation) at rest, the joint frame
    in each body is just (anchor - body_translate), rotation identity.
    """
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([body0.GetPath()])
    j.CreateBody1Rel().SetTargets([body1.GetPath()])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*[a - b for a, b in zip(anchor, t0)]))
    j.CreateLocalPos1Attr(Gf.Vec3f(*[a - b for a, b in zip(anchor, t1)]))
    j.CreateLocalRot0Attr(_IDENTITY)
    j.CreateLocalRot1Attr(_IDENTITY)
    j.CreateLowerLimitAttr(float(limits_deg[0]))
    j.CreateUpperLimitAttr(float(limits_deg[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(float(drive_target_deg))
    return j


# ---------------------------------------------------------------------------
# Articulated robot arm
# ---------------------------------------------------------------------------
def build_robot_arm_articulated(stage, path, mats, pose=(0.0, -35.0, 70.0)):
    """Build a 3-DOF articulated arm rooted at `path`.

    mats : dict with keys 'steel', 'robot' (UsdShade.Material).
    pose : (yaw, shoulder, elbow) drive targets in degrees.
    Returns the articulation root prim.
    """
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    # the articulation root tells PhysX "everything under here is one robot"
    UsdPhysics.ArticulationRootAPI.Apply(root)

    # --- links (translate-only bodies; geometry hangs underneath) ---
    t_base, t0, t1, t2 = (0, 0, 0.2), (0, 0, 0.5), (0, 0, 1.1), (0, 0, 1.95)

    base = _rigid_link(stage, f"{path}/base_link", t_base, mass=20.0)
    _add_collision_cyl(stage, f"{path}/base_link/geo", 0.35, 0.4)

    l0 = _rigid_link(stage, f"{path}/link0", t0, mass=6.0)
    _add_collision_cyl(stage, f"{path}/link0/geo", 0.25, 0.2)

    l1 = _rigid_link(stage, f"{path}/link1", t1, mass=8.0)
    _add_collision_box(stage, f"{path}/link1/geo", (0.2, 0.2, 1.0))

    l2 = _rigid_link(stage, f"{path}/link2", t2, mass=4.0)
    _add_collision_box(stage, f"{path}/link2/geo", (0.18, 0.18, 0.7))
    # gripper is rigidly part of link2 (a second collider on the same body)
    _add_collision_box(stage, f"{path}/link2/gripper", (0.24, 0.14, 0.14),
                       translate=(0, 0, 0.42))

    # --- joints ---
    # bolt the base to the world at its current pose
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{path}/joint_fixed")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])

    _revolute(stage, f"{path}/joint0_yaw", base, l0, anchor=(0, 0, 0.4),
              axis="Z", limits_deg=(-170, 170), drive_target_deg=pose[0],
              t0=t_base, t1=t0)
    _revolute(stage, f"{path}/joint1_shoulder", l0, l1, anchor=(0, 0, 0.6),
              axis="Y", limits_deg=(-90, 90), drive_target_deg=pose[1],
              t0=t0, t1=t1)
    _revolute(stage, f"{path}/joint2_elbow", l1, l2, anchor=(0, 0, 1.6),
              axis="Y", limits_deg=(-150, 150), drive_target_deg=pose[2],
              t0=t1, t1=t2)

    # --- materials ---
    assets.bind(base, mats["steel"])
    for link in (l0, l1, l2):
        for child in link.GetChildren():
            assets.bind(child, mats["robot"])

    return root
