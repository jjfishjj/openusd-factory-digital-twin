"""GPU-free previews of a factory-twin USD stage.

Two outputs, both by reading world-space bounding boxes straight from USD — no
RTX, no PhysX solver needed:

* ``render``            → a top-down floor-plan PNG (static layout check).
* ``render_animation``  → an animated GIF of the conveyor flow, by sampling the
                          workpieces' *time-sampled* transforms frame by frame.

The GIF shows only the kinematic conveyor animation, which is authored directly
in USD. The dynamic drop-parts and the robot-arm drive poses require a physics
solver (Isaac Sim) and so appear static here.

Run:
  factory-twin-preview  --in output/factory_twin.usda --out output/floorplan.png
  factory-twin-animate  --in output/factory_twin.usda --out output/conveyor.gif
"""

from __future__ import annotations

import argparse
import io

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

# top-level group -> (colour, label). Order = draw order (back to front).
GROUPS = {
    "/World/Floor": ("#33343d", "Floor"),
    "/World/Conveyor": ("#5a5c62", "Conveyor"),
    "/World/Racks": ("#3359bf", "Storage racks"),
    "/World/Workstations": ("#f28c1a", "Robot stations"),
    "/World/AMR": ("#e6d919", "AMR"),
    "/World/DropParts": ("#33b359", "Drop parts (physics)"),
    "/World/Workpieces": ("#cc3333", "Workpieces (animated)"),
}


def _leaf_gprims(prim, group):
    if group == "/World/Floor":
        return [prim]
    return [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Gprim)]


def _draw_group(ax, stage, cache, group, color):
    prim = stage.GetPrimAtPath(group)
    if not prim:
        return
    for leaf in _leaf_gprims(prim, group):
        r = cache.ComputeWorldBound(leaf).ComputeAlignedRange()
        lo, hi = r.GetMin(), r.GetMax()
        w, h = hi[0] - lo[0], hi[1] - lo[1]
        if w <= 0 or h <= 0:
            continue
        ax.add_patch(mpatches.Rectangle(
            (lo[0], lo[1]), w, h, facecolor=color, edgecolor="white",
            linewidth=0.4, alpha=0.9))


def _draw_amr_route(ax, stage):
    curve = stage.GetPrimAtPath("/World/AMR_Path")
    if not curve:
        return
    pts = UsdGeom.BasisCurves(curve).GetPointsAttr().Get()
    ax.plot([p[0] for p in pts], [p[1] for p in pts], "--",
            color="#e6d919", linewidth=1.5, alpha=0.8)


def _setup_axes(ax, title):
    ax.set_facecolor("#15161a")
    ax.set_xlim(-9, 9)
    ax.set_ylim(-13, 13)
    ax.set_aspect("equal")
    ax.set_title(title, color="white", fontsize=11)
    ax.tick_params(colors="#888")
    for spine in ax.spines.values():
        spine.set_color("#888")
    ax.annotate("material flow ▲", xy=(6.5, 0), color="#aaa", rotation=90,
                fontsize=9, ha="center", va="center")


def _legend(ax):
    handles = [mpatches.Patch(color=c, label=lbl) for c, lbl in GROUPS.values()]
    ax.legend(handles=handles, loc="upper left", fontsize=7.5, framealpha=0.3,
              facecolor="#222", labelcolor="white")


# ---------------------------------------------------------------------------
# Static floor plan
# ---------------------------------------------------------------------------
def render(in_path: str, out_path: str) -> str:
    stage = Usd.Stage.Open(in_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    fig, ax = plt.subplots(figsize=(7, 9))
    for group, (color, _) in GROUPS.items():
        _draw_group(ax, stage, cache, group, color)
    _draw_amr_route(ax, stage)
    _setup_axes(ax, "Factory Digital Twin — top-down floor plan (metres)")
    _legend(ax)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130, facecolor="#15161a")
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# Animated conveyor GIF
# ---------------------------------------------------------------------------
def _fig_to_pil(fig) -> Image.Image:
    fig.canvas.draw()
    return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB")


def render_animation(in_path: str, out_path: str, max_frames: int = 40) -> str:
    stage = Usd.Stage.Open(in_path)
    start = int(stage.GetStartTimeCode())
    end = int(stage.GetEndTimeCode())
    fps = stage.GetTimeCodesPerSecond() or 24
    step = max(1, (end - start) // max_frames)
    frames = list(range(start, end, step))

    cache = UsdGeom.BBoxCache(Usd.TimeCode(start),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    static_groups = {g: c for g, (c, _) in GROUPS.items() if g != "/World/Workpieces"}

    images = []
    for tc in frames:
        cache.SetTime(Usd.TimeCode(tc))
        fig, ax = plt.subplots(figsize=(7, 9))
        for group, color in static_groups.items():
            _draw_group(ax, stage, cache, group, color)
        _draw_amr_route(ax, stage)
        _draw_group(ax, stage, cache, "/World/Workpieces", GROUPS["/World/Workpieces"][0])
        _setup_axes(ax, "Factory Digital Twin — conveyor flow")
        _legend(ax)
        fig.tight_layout()
        images.append(_fig_to_pil(fig))
        plt.close(fig)

    frame_ms = int(1000 * step / fps)
    images[0].save(out_path, save_all=True, append_images=images[1:],
                   duration=frame_ms, loop=0, optimize=True)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Top-down floor-plan PNG")
    parser.add_argument("--in", dest="in_path", default="output/factory_twin.usda")
    parser.add_argument("--out", dest="out_path", default="output/floorplan.png")
    args = parser.parse_args()
    print(f"✓ wrote {render(args.in_path, args.out_path)}")


def main_animate() -> None:
    parser = argparse.ArgumentParser(description="Animated conveyor-flow GIF")
    parser.add_argument("--in", dest="in_path", default="output/factory_twin.usda")
    parser.add_argument("--out", dest="out_path", default="output/conveyor.gif")
    parser.add_argument("--frames", type=int, default=40, help="max GIF frames")
    args = parser.parse_args()
    print(f"✓ wrote {render_animation(args.in_path, args.out_path, args.frames)}")


if __name__ == "__main__":
    main()
