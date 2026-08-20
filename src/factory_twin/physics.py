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
    """Author the global physics scene: Earth gravity along -Z, CCD on.

    Continuous collision detection has to be enabled scene-wide as well as
    per-body. Without it PhysX only tests for contact at discrete timesteps, so
    a fast body can pass straight through thin geometry between two steps. The
    drop parts reach ~7.7 m/s by the time they arrive at the 0.1 m floor slab,
    which is 0.13 m of travel per 1/60 s step — wider than the slab. Verified on
    an L40S: without this flag all three drop parts tunnel through the floor.

    The PhysX schemas are authored by name (AddAppliedSchema plus plain
    attributes) rather than through PhysxSchema, so building the scene still
    only needs `usd-core` — no Omniverse install.
    """
    scene = UsdPhysics.Scene.Define(stage, path)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    prim = scene.GetPrim()
    prim.AddAppliedSchema("PhysxSceneAPI")
    prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
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


def make_dynamic_body(prim, mass=2.0, ccd=True):
    """A free dynamic rigid body — falls under gravity, collides, comes to rest.
    Used for the loose parts that drop onto the belt in Isaac Sim.
    Apply add_collider() to its child geometry to give it a collision shape.

    `ccd` enables continuous collision detection on this body; see
    setup_physics_scene() for why the drop parts need it."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    if ccd:
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:enableCCD",
                             Sdf.ValueTypeNames.Bool).Set(True)
    return prim


def isolate_choreographed_parts(stage, choreographed, simulated,
                               root="/World/CollisionGroups"):
    """Stop the kinematic, time-sampled parts from fighting the solver.

    The workpieces are kinematic bodies, which PhysX treats as infinitely
    massive: whatever they touch loses the argument. Measured in Isaac Sim 5.1
    on an L40S, that costs the scene twice.

      * A pick part is authored *inside* the gripper for the carry - that is
        what a baked grasp is - so at frame 0 it shoves Robot_01 and Robot_02
        off their commanded rest pose by 8.8 and 20.6 degrees. Robots 00 and
        03, whose pick parts are elsewhere at frame 0, hold to 0.02 degrees.
        Deactivating /World/Workpieces drops the worst error to 0.4 degrees;
        deactivating any other group changes nothing.
      * A pass-through part wraps from the end of the belt back to the start in
        a single frame - 20 m in 1/24 s - and launches whatever is resting on
        the belt down the line. The drop parts land correctly at z = 1.05 and
        sit there for about a second before being swept 30-56 m along +Y, past
        the edge of the floor.

    Filtering the choreographed parts against the simulated ones removes both.
    They still collide with the floor and the belt, so they stay physical
    scenery; they just no longer win arguments with the robots.

    This is a scene-authoring fix, not a physical one. The honest fix for the
    grasp is a contact gripper driven by an Isaac Sim runtime script (a
    SurfaceGripper), where the arm really does carry the part. Until then this
    keeps the authored choreography and the physics from corrupting each other.
    """
    UsdGeom.Scope.Define(stage, root)
    g_chor = UsdPhysics.CollisionGroup.Define(stage, f"{root}/Choreographed")
    g_sim = UsdPhysics.CollisionGroup.Define(stage, f"{root}/Simulated")
    for path in choreographed:
        g_chor.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(path)
    for path in simulated:
        g_sim.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(path)
    g_chor.CreateFilteredGroupsRel().SetTargets([g_sim.GetPath()])
    return g_chor, g_sim


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


def set_drive_target(joint_prim, angle_deg, time=None):
    """Author an angular-drive target on a revolute joint. With `time` this
    writes a time sample, building the commanded trajectory Isaac Sim's PD
    controller follows (and that the FK preview reads to draw the motion)."""
    attr = UsdPhysics.DriveAPI(joint_prim, "angular").GetTargetPositionAttr()
    if time is None:
        attr.Set(float(angle_deg))
    else:
        attr.Set(float(angle_deg), time=time)


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
