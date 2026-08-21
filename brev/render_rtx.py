"""Render the factory twin with RTX path tracing.

The GPU-free preview in this repo draws the scene with matplotlib: it proves the
kinematics but it is a diagram, not a render. This script opens the same USD in
Isaac Sim on an RTX GPU, plays the authored timeline, and path-traces every
frame - the materials, the lights and the geometry that were authored all along,
finally shaded by the renderer they were written for.

Outputs a numbered PNG sequence; encode it with brev/encode.sh.
"""

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--stage", default="/workspace/home/ubuntu/factory-twin/output/factory_twin.usda")
parser.add_argument("--outdir", default="/workspace/home/ubuntu/factory-twin/output/rtx")
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--spp", type=int, default=64, help="path-tracing samples per pixel")
parser.add_argument("--frame-step", type=int, default=2, help="render every Nth authored frame")
parser.add_argument("--shot", default="hero", choices=["hero", "line", "station"])
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({
    "headless": True,
    "renderer": "PathTracing",
    "width": args.width,
    "height": args.height,
})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from pxr import Usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402

# --- path tracing settings -------------------------------------------------
st = carb.settings.get_settings()
st.set("/rtx/rendermode", "PathTracing")
st.set("/rtx/pathtracing/spp", 1)
st.set("/rtx/pathtracing/totalSpp", args.spp)
st.set("/rtx/pathtracing/optixDenoiser/enabled", 1)
st.set("/rtx/pathtracing/maxBounces", 6)
print(f"path tracing: {args.width}x{args.height} @ {args.spp} spp", flush=True)

open_stage(args.stage)
stage = Usd.Stage.Open(args.stage)
TPS = float(stage.GetTimeCodesPerSecond() or 24.0)
START_TC, END_TC = int(stage.GetStartTimeCode()), int(stage.GetEndTimeCode())
print(f"timeline {START_TC}..{END_TC} @ {TPS} fps", flush=True)

# --- camera ----------------------------------------------------------------
# The plant runs along +Y; the robots stand on the +X side at x = 1.8, the racks
# on the -X side at x = -4.5. These frame the line without clipping either.
SHOTS = {
    "hero":    dict(position=(16.0, -20.0, 11.0), look_at=(0.0, -1.0, 1.2)),
    "line":    dict(position=(7.5, -17.0, 4.2),   look_at=(0.5, 2.0, 1.1)),
    "station": dict(position=(5.2, -10.5, 3.0),   look_at=(1.6, -7.5, 1.6)),
}
shot = SHOTS[args.shot]
camera = rep.create.camera(position=shot["position"], look_at=shot["look_at"],
                           focal_length=24.0)
render_product = rep.create.render_product(camera, (args.width, args.height))

outdir = os.path.join(args.outdir, args.shot)
os.makedirs(outdir, exist_ok=True)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=outdir, rgb=True)
writer.attach([render_product])
print(f"writing to {outdir}", flush=True)

# --- play the authored timeline, path-tracing as we go ---------------------
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / TPS, rendering_dt=1.0 / TPS)
timeline = omni.timeline.get_timeline_interface()
timeline.set_time_codes_per_second(TPS)
timeline.set_start_time(START_TC / TPS)
timeline.set_end_time(END_TC / TPS)
timeline.set_looping(False)
timeline.set_current_time(START_TC / TPS)
world.reset()

frames = list(range(START_TC, END_TC + 1, args.frame_step))
print(f"rendering {len(frames)} frames (every {args.frame_step})", flush=True)
for n, f in enumerate(frames):
    timeline.set_current_time(f / TPS)
    # let the stage settle on this timecode before the sampler starts
    world.step(render=False)
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    if n % 10 == 0:
        print(f"  frame {f:3d}  ({n + 1}/{len(frames)})", flush=True)

rep.orchestrator.wait_until_complete()
timeline.stop()
print(f"\ndone - {len(frames)} frames in {outdir}", flush=True)
sim_app.close()
