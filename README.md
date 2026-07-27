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
(rendering) and **PhysX** (simulation). RTX and PhysX need an NVIDIA GPU — but
*every* Omniverse workflow is authored on top of USD. This project builds the
USD layer directly, which is the transferable, portable core skill:

- **Stage & scenegraph** — `/World` hierarchy, `Xform` transforms, Z-up / metres
  metadata (the Isaac Sim / robotics convention).
- **Materials** — portable `UsdPreviewSurface` PBR shaders (`UsdShade`).
- **Lighting** — `UsdLux` dome + distant key light.
- **Instancing at scale** — storage racks are authored **once** as a prototype
  and stamped as `instanceable` references, the technique that keeps
  factory/warehouse twins light.
- **Semantic geometry** — the AMR route is a first-class `BasisCurves` path, not
  a baked mesh, so a nav stack (or Isaac Sim) can consume it.

## Scene contents

| Asset | Count | Notes |
|-------|-------|-------|
| Conveyor belt | 1 | 20 m, runs along +Y (material-flow direction) |
| Workpieces | 6 | parts spaced along the belt |
| Robot-arm workstations | 4 | 3-segment arms on pedestals, along the +X side |
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
  build_scene.py   # assembles the full production line -> .usda
  preview.py       # GPU-free top-down floor-plan renderer (matplotlib)
output/            # generated .usda / .usdc / .png (git-ignored except the sample PNG)
```

## Roadmap

- [ ] `UsdPhysics` rigid bodies + revolute joints so the arms articulate in Isaac Sim
- [ ] Animate workpieces travelling down the belt (time-sampled transforms)
- [ ] Parameterise the whole layout from a YAML config
- [ ] A proper 3D isometric preview (currently top-down only)

## Requirements

- Python ≥ 3.10
- [`usd-core`](https://pypi.org/project/usd-core/) — OpenUSD Python runtime
- `matplotlib` — for the GPU-free preview only

## License

MIT
