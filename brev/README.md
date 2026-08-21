# Validating the twin in real PhysX

The scene in this repo is authored with `usd-core` alone and previewed without a
GPU. That is deliberate — but it means the physics is only *authored*, never
*run*. UsdPhysics is a description; whether the articulations are drivable and
whether the parts land where the layout says they should is a question only a
solver can answer.

`validate_physics.py` answers it. It runs the scene headless in Isaac Sim on an
NVIDIA GPU and reports what PhysX actually did.

## The machine

Run on an [NVIDIA Brev](https://brev.nvidia.com) instance:

| | |
|---|---|
| Instance | `g6e.xlarge` |
| GPU | NVIDIA **L40S**, 46 GB |
| Driver / CUDA | 580.178.04 / 13.0 |
| OS | Ubuntu 22.04.5 |
| Image | `j3soon/runai-isaac-lab-ex:2.3.2-ros2-jazzy` |
| Stack | **Isaac Sim 5.1.0**, Isaac Lab 2.3.2, ROS 2 Jazzy |
| Solver | PhysX TGS, CPU dynamics, 240 Hz substeps, `physics_dt` 1/120 |

```bash
brev copy ./ your-instance:/home/ubuntu/factory-twin/
brev exec your-instance \
  "docker exec isaac-lab-ex-ros2-isaac-sim-ex-1 bash -c \
   'cd /workspace/home/ubuntu/factory-twin && \
    /root/isaacsim/python.sh validate_physics.py'"
```

It writes `output/physx_report.json` and prints a human summary.

## What it checks

| Phase | Question |
|---|---|
| 0 | Does the USD declare what it claims? Schema counts, drive targets vs joint limits, CCD flags, gravity, units |
| 1 | Do the dynamic parts fall, and do they come to rest where the layout says? |
| 2a | Can each arm hold its authored rest pose? If not, **which part of the scene is interfering?** (ablation) |
| 2b | Play the authored 192-frame timeline and let Isaac Sim drive the joints from the USD time samples |
| 2c | Do the animated joints travel, and do the static-target joints hold? |

## Results

**All 12 checks pass.** PhysX parses all four arms as drivable robots and the
authored trajectory plays:

| | |
|---|---|
| Articulation roots parsed | **4 / 4** |
| DOF per arm | **3** — `joint0_yaw`, `joint1_shoulder`, `joint2_elbow` |
| Revolute joints / rigid bodies / colliders | 12 / 25 / 35 |
| Driven joints | 12 (8 time-sampled, 4 static by design) |
| Drive targets outside a joint limit | **0** |
| Shoulder travel under PhysX | **141.0°** (authored range 145.6°) |
| Elbow travel under PhysX | **85.2°** |
| Steady-state tracking lag | max **5.50°**, mean 5.27°, p95 5.41° |
| Arms holding their rest pose | **0.409°** worst |
| Static-target joints, peak deflection | **0.07°**, final error 0.000° |
| Drop parts at rest | **z = 1.05 m** — belt top 0.875 + half extent 0.175, residual 0.00000 m |

The full report is committed as [`physx_report.json`](physx_report.json).

So the README's central claim — *the arms reach toward the belt and retract on
the timeline in Isaac Sim* — holds up under measurement.

## The bug the solver found

It did not hold up at first. The initial run put the drop parts **30–56 m down
the line and off the edge of the floor**, and left two of the four arms unable
to hold their commanded pose:

```
Robot_00  hold error   0.02°      Robot_01  hold error   8.8°
Robot_03  hold error   0.13°      Robot_02  hold error  20.6°
```

Ablation named the cause in one pass — deactivating `/World/Workpieces` dropped
the worst error from **20.626° to 0.409°**, while deactivating any other group
changed nothing at all. The USD confirms the mechanism. At frame 0:

| Robot | its pick part sits at | hold error |
|---|---|---|
| `Robot_00` (base y = −7.5) | `(0.10, −13.50, 1.02)` — away on the belt | 0.02° |
| `Robot_01` (base y = −2.5) | `(0.47, **−2.31**, 1.60)` — **inside the arm** | 8.8° |
| `Robot_02` (base y = +2.5) | `(2.93, **2.70**, 1.72)` — **inside the arm** | 20.6° |
| `Robot_03` (base y = +7.5) | `(3.46, 7.50, 1.00)` — in its bin | 0.13° |

The workpieces are **kinematic** bodies, which PhysX treats as infinitely
massive. A baked grasp authors the carried part *inside the gripper* — that is
what "baked" means — so instead of the arm carrying the part, the part shoves
the arm. The same infinite mass explains the drop parts: a pass-through piece
wraps from the end of the belt back to the start in a single frame, 20 m in
1/24 s, and launches whatever is resting on the belt down the line.

Worth stressing what this was *not*. The first two hypotheses — collider
tunnelling, then an invalid collision shape — were both wrong, and the
instrumentation is what refuted them. The drop parts were crossing the belt at
**0.67 m/s**, moving 0.003 m per step against a 0.15 m slab: far too slow to
tunnel. And they were *already resting correctly* at z = 1.05 for a second
before they moved. They were never falling through anything. They were being
bulldozed.

### The fix

`physics.isolate_choreographed_parts()` puts the kinematic parts in a
`UsdPhysics.CollisionGroup` filtered against the robots and the dynamic parts.
They still collide with the floor and the belt, so they remain physical
scenery — they simply no longer win arguments with the solver.

| | before | after |
|---|---|---|
| Drop parts | swept to y = +29…+56, off the floor | **rest at z = 1.05, xy unmoved** |
| Worst hold error | 20.626° | **0.409°** |
| Steady tracking lag | 11–25° | **5.3–5.5°** |
| Shoulder travel spread across the 4 arms | 137.8–141.6° | **141.0° on all four** |

Ablation afterwards reports *no culprit* — with the baseline already at 0.409°
there is nothing left for it to find. That is the fix confirming itself.

This is a scene-authoring fix, not a physical one. The honest fix for the grasp
is a contact gripper driven by an Isaac Sim runtime script (a `SurfaceGripper`),
where the arm really does carry the part — already the repo's stated next step,
now with measurements behind it.

## Where the measurement window matters

**69° of startup convergence.** The USD stores the arms at the zero pose and the
motion in the drive targets, so when playback starts the solver has to travel to
the frame-0 pose. That is convergence, not tracking error — it is reported
separately rather than averaged into the tracking statistic.

Getting that window right changed a conclusion. Measured across the whole
playback, the static-target `joint0_yaw` appeared to swing **3.9–7.8°**, which
reads like a turntable drive too soft to resist the reaction torque of the pick
cycle, and was very nearly written up as exactly that. Measured over the steady
window alone, the same joints move **0.07°** and return to target with **0.000°**
error. The swing was the arms leaving the rest pose at t=0, not reaction torque.
There was nothing to fix — only something to measure correctly.

## Path-traced rendering

`render_rtx.py` opens the same stage, plays the authored timeline and
path-traces each frame on the GPU. The preview in the root README is
`--shot hero --frame-step 2 --spp 64` at 1280x720 — 97 frames in about 170 s on
the L40S.

```bash
brev exec your-instance \
  "docker exec isaac-lab-ex-ros2-isaac-sim-ex-1 bash -c \
   'cd /workspace/home/ubuntu/factory-twin && \
    /root/isaacsim/python.sh render_rtx.py --shot hero --frame-step 2 --spp 64'"
```

Three camera presets are included: `hero` (the whole plant), `line` (down the
belt) and `station` (one workstation). The output is a numbered PNG sequence;
`encode.sh` turns it into an mp4 and a looping gif with ffmpeg — run it wherever
you have ffmpeg, the Isaac Sim image does not ship it.

## Notes for anyone re-running this

Three things cost real time, all of them Isaac Sim behaviours rather than scene
problems:

1. **`World()` rewrites the stage's `timeCodesPerSecond`** to match its
   rendering rate. Read the authored timeline *before* constructing `World`, or
   every frame-to-substep calculation downstream is wrong.
2. **Isaac re-syncs time-sampled drive targets from USD to PhysX every step.**
   Commanding targets by hand while the timeline is stopped does nothing — the
   authored value at the current timeline time always wins. A scene like this
   has to be validated by *playing the timeline*.
3. **Any stop/play cycle tears down the PhysX tensor view** an `Articulation`
   wraps, which surfaces as `Failed to get DOF positions from backend`.
   Configure the timeline first, `world.reset()` once, then build the view.
