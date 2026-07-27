"""Render a top-down floor-plan PNG of a factory-twin USD stage.

This is a lightweight, dependency-light sanity check for machines WITHOUT an
RTX GPU (e.g. macOS): instead of path-tracing the scene in Omniverse, it reads
each asset's world-space bounding box straight from USD and draws it as a
rectangle. It proves the layout is correct before you ever open Omniverse.

Run:  python -m factory_twin.preview --in output/factory_twin.usda --out output/floorplan.png
"""

from __future__ import annotations

import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

# top-level group -> colour + label
GROUPS = {
    "/World/Floor": ("#33343d", "Floor"),
    "/World/Conveyor": ("#5a5c62", "Conveyor"),
    "/World/Workpieces": ("#cc3333", "Workpieces"),
    "/World/Workstations": ("#f28c1a", "Robot stations"),
    "/World/Racks": ("#3359bf", "Storage racks"),
    "/World/AMR": ("#e6d919", "AMR"),
}


def _xy_rect(bbox):
    r = bbox.ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    return lo[0], lo[1], hi[0] - lo[0], hi[1] - lo[1]


def render(in_path: str, out_path: str) -> str:
    stage = Usd.Stage.Open(in_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    fig, ax = plt.subplots(figsize=(7, 9))
    ax.set_facecolor("#15161a")

    for group, (color, _) in GROUPS.items():
        prim = stage.GetPrimAtPath(group)
        if not prim:
            continue
        # draw each leaf gprim in the group as its own footprint.
        # TraverseInstanceProxies() is required to descend into instanceable
        # prims (the storage racks) whose geometry lives under a prototype.
        leaves = [prim] if group in ("/World/Floor",) else [
            p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Gprim)]
        for leaf in leaves:
            x, y, w, h = _xy_rect(cache.ComputeWorldBound(leaf))
            if w <= 0 or h <= 0:
                continue
            ax.add_patch(mpatches.Rectangle(
                (x, y), w, h, facecolor=color, edgecolor="white",
                linewidth=0.4, alpha=0.9))

    # AMR route as a dashed line
    curve = stage.GetPrimAtPath("/World/AMR_Path")
    if curve:
        pts = UsdGeom.BasisCurves(curve).GetPointsAttr().Get()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, "--", color="#e6d919", linewidth=1.5, alpha=0.8)

    ax.set_xlim(-9, 9)
    ax.set_ylim(-13, 13)
    ax.set_aspect("equal")
    ax.set_title("Factory Digital Twin — top-down floor plan (metres)",
                 color="white", fontsize=11)
    ax.tick_params(colors="#888")
    for spine in ax.spines.values():
        spine.set_color("#888")
    ax.annotate("material flow ▲", xy=(6.5, 0), color="#aaa", rotation=90,
                fontsize=9, ha="center", va="center")

    handles = [mpatches.Patch(color=c, label=lbl) for _, (c, lbl) in GROUPS.items()]
    ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.3,
              facecolor="#222", labelcolor="white")

    fig.tight_layout()
    fig.savefig(out_path, dpi=130, facecolor="#15161a")
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in", dest="in_path", default="output/factory_twin.usda")
    parser.add_argument("--out", dest="out_path", default="output/floorplan.png")
    args = parser.parse_args()
    out = render(args.in_path, args.out_path)
    print(f"✓ wrote {out}")


if __name__ == "__main__":
    main()
