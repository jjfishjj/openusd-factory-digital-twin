# Factory Digital Twin — OpenUSD

A procedurally-generated **factory production-line digital twin** built with
[OpenUSD](https://openusd.org) (Pixar's Universal Scene Description — the format
at the core of **NVIDIA Omniverse**).

Pure Python, no GPU required. It runs anywhere `usd-core` installs (incl. macOS)
and emits a `.usd` scene that opens directly in Omniverse / USD Composer /
Isaac Sim on any RTX machine, where the placeholder primitives can be swapped
for real CAD assets and lit with RTX path tracing.

![Factory floor plan](output/floorplan.png)

## Why this is an Omniverse project without an Omniverse install

Omniverse's three pillars are **OpenUSD** (scene description), **RTX**
(rendering) and **PhysX** (simulation). RTX needs an NVIDIA GPU — but *every*
Omniverse workflow is authored on top of USD, and PhysX is driven entirely by
**UsdPhysics** schemas that live in the USD file. This project builds both
layers directly, which is the transferable, portable core skill:

- **Stage & scenegraph** — `/World` hierarchy, `Xform` transforms, Z-up / metres
  metadata (the Isaac Sim / robotics convention).
- **Materials** — portable `UsdPreviewSurface` PBR shaders (`UsdShade`).
- **Lighting** — `UsdLux` dome + distant key light.
- **Instancing at scale** — storage racks are authored **once** as a prototype
  and stamped as `instanceable` references, the technique that keeps
  factory/warehouse twins light.
- **Semantic geometry** — the AMR route is a first-class `BasisCurves` path, not
  a baked mesh, so a nav stack (or Isaac Sim) can consume it.
- **Physics & articulation** — the robot arms are real `UsdPhysics`
  articulations (see below), so Isaac Sim loads them as drivable robots.

## Physics & articulation

Each of the 4 robot arms is authored as a **UsdPhysics articulation** — a
kinematic chain of rigid bodies joined by revolute (hinge) joints — so PhysX in
Isaac Sim treats it as one drivable robot instead of loose meshes:

```
base_link ──fixed──▶ world          (bolted to the floor)
   │ joint0_yaw       revolute · axis Z · ±170°   (turntable)
link0
   │ joint1_shoulder  revolute · axis Y · ±90°
link1  (upper arm)
   │ joint2_elbow     revolute · axis Y · ±150°
link2  (forearm + gripper)
```

- Global `UsdPhysics.Scene` with Earth gravity (−Z, 9.81 m/s²); the floor is a
  static collider.
- Every link has `RigidBodyAPI` + `MassAPI`; every visual gprim has
  `CollisionAPI`.
- Every revolute joint has angle **limits** and an angular **`DriveAPI`**
  (stiffness/damping + a target angle) — the actuation Isaac Sim reads to
  position the arm.

> **Rest pose vs. driven pose:** the USD file stores the arm at rest (pointing
> straight up). The `DriveAPI` target angles only bend the arm once **PhysX
> simulates** it in Isaac Sim — a static USD viewer will show it upright. This
> is expected: the drive targets are goals for the physics solver, not authored
> transforms.

Verify the rigging without a GPU:

```bash
PYTHONPATH=src python -c "from pxr import Usd,UsdPhysics as P; s=Usd.Stage.Open('output/factory_twin.usda'); \
print('articulations', sum(p.HasAPI(P.ArticulationRootAPI) for p in s.Traverse()), \
'revolute joints', sum(p.IsA(P.RevoluteJoint) for p in s.Traverse()))"
```

## Scene contents

| Asset | Count | Notes |
|-------|-------|-------|
| Conveyor belt | 1 | 20 m, runs along +Y (material-flow direction) |
| Workpieces | 6 | parts spaced along the belt |
| Robot-arm workstations | 4 | 3-DOF **UsdPhysics articulations**, along the +X side |
| Storage racks | 3 | `instanceable` — 3 bays × 3 levels each |
| AMR + route | 1 | mobile robot with a dashed loop path |
| Lights | 2 | dome + distant key |

## Quick start

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .

# generate the USD scene
factory-twin-build --out output/factory_twin.usda

# render a GPU-free top-down floor-plan PNG
factory-twin-preview --in output/factory_twin.usda --out output/floorplan.png
```

Or without installing:

```bash
pip install usd-core matplotlib
PYTHONPATH=src python -m factory_twin.build_scene  --out output/factory_twin.usda
PYTHONPATH=src python -m factory_twin.preview      --in  output/factory_twin.usda --out output/floorplan.png
```

## Open it in Omniverse (on an RTX machine)

1. Copy `output/factory_twin.usda` (or `.usdc`) to a Windows/Linux box with an
   NVIDIA RTX GPU.
2. Open it in **USD Composer** / **Omniverse Kit** / **Isaac Sim** — the scene
   loads with materials and lights already bound.
3. Swap placeholder cubes/cylinders for real assets, enable RTX path tracing,
   and (next step) add `UsdPhysics` joints to articulate the robot arms.

## Project layout

```
src/factory_twin/
  assets.py        # reusable USD asset builders (materials, boxes, robot, rack, AMR)
  physics.py       # UsdPhysics rigging: physics scene, articulated robot arms
  build_scene.py   # assembles the full production line -> .usda
  preview.py       # GPU-free top-down floor-plan renderer (matplotlib)
output/            # generated .usda / .usdc / .png (git-ignored except the sample PNG)
```

## Roadmap

- [x] `UsdPhysics` rigid bodies + revolute joints so the arms articulate in Isaac Sim
- [ ] Animate workpieces travelling down the belt (time-sampled transforms)
- [ ] Parameterise the whole layout from a YAML config
- [ ] A proper 3D isometric preview (currently top-down only)
- [ ] Conveyor + workpieces as physics bodies (parts ride / drop realistically)

## Requirements

- Python ≥ 3.10
- [`usd-core`](https://pypi.org/project/usd-core/) — OpenUSD Python runtime
- `matplotlib` — for the GPU-free preview only

## License

MIT
