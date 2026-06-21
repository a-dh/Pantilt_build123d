"""Render the pan-tilt motion to MP4 videos (headless, via matplotlib + ffmpeg).

Produces two 5-second clips with smooth sinusoidal motion that sweeps the joint
limits (pan -90..+90, tilt -15..+90), two pan cycles:

    videos/pantilt_opaque.mp4        solid volumes
    videos/pantilt_transparent.mp4   see-through volumes

The OCP viewer cannot capture video, so each part is tessellated once and the
same pan/tilt rotations used by the kinematic model are applied per frame with
numpy, then drawn with matplotlib's software 3D renderer.

Run:  venv/bin/python render_video.py
Quick smoke test:  QUICK=1 venv/bin/python render_video.py
"""

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import ocp_vscode
ocp_vscode.show = lambda *a, **k: None  # never open the viewer from here

from pantilt_build123d.pan_tilt_assembly import build_assembly

DURATION_S = 5.0
FPS = 24
TESS_TOL = 0.4           # mm; coarser = fewer triangles = faster render
FIGSIZE = (8, 4.5)       # inches
DPI = 120                # -> 960 x 540 (even, yuv420p-friendly)
ELEV, AZIM = 22, -60

if os.environ.get("QUICK"):
    DURATION_S, FPS, DPI = 1.0, 6, 80


# ---- motion: smooth sinusoids that hit the limits exactly -------------------
def pan_angle(t):
    """-90..+90, two full cycles over the clip."""
    return 90.0 * np.sin(2 * np.pi * 2 * t / DURATION_S)


def tilt_angle(t):
    """-15..+90, two full cycles, starting at the lower limit."""
    mid = (-15 + 90) / 2          # 37.5
    amp = (90 - (-15)) / 2        # 52.5
    return mid - amp * np.cos(2 * np.pi * 2 * t / DURATION_S)


def rot_z(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _apply(verts, rot, center):
    return (verts - center) @ rot.T + center


def tessellate_assembly():
    """Return (tess, pan_center, tilt_center).

    tess: list of dicts {verts(Nx3), faces(Mx3), rgb, group} where group is
    'static', 'pan', or 'tilt'.
    """
    asm = build_assembly()
    parts = asm["parts"]
    pan_center = np.array(tuple(asm["pan_joint"].location.position))
    tilt_center = np.array(tuple(asm["tilt_joint"].location.position))

    pan_objs = {id(rj.parent) for rj in asm["pan_rigid_joints"]}
    tilt_objs = {id(rj.parent) for rj in asm["tilt_rigid_joints"]}

    tess = []
    for name, part in parts.items():
        vlist, tris = part.tessellate(TESS_TOL)
        verts = np.array([(v.X, v.Y, v.Z) for v in vlist])
        faces = np.array(tris)
        rgb = tuple(part.color)[:3]
        if id(part) in tilt_objs:
            group = "tilt"
        elif id(part) in pan_objs:
            group = "pan"
        else:
            group = "static"
        tess.append({"name": name, "verts": verts, "faces": faces,
                     "rgb": rgb, "group": group})
    return tess, pan_center, tilt_center


def transformed(entry, Rz, Ry, pan_center, tilt_center):
    v = entry["verts"]
    if entry["group"] == "pan":
        v = _apply(v, Rz, pan_center)
    elif entry["group"] == "tilt":
        v = _apply(_apply(v, Ry, tilt_center), Rz, pan_center)
    return v


def global_bounds(tess, pan_center, tilt_center, n_samples=24):
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for i in range(n_samples):
        t = DURATION_S * i / n_samples
        Rz, Ry = rot_z(pan_angle(t)), rot_y(tilt_angle(t))
        for e in tess:
            v = transformed(e, Rz, Ry, pan_center, tilt_center)
            lo = np.minimum(lo, v.min(axis=0))
            hi = np.maximum(hi, v.max(axis=0))
    return lo, hi


def render(tess, pan_center, tilt_center, bounds, alpha, out_mp4, frame_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    lo, hi = bounds
    span = (hi - lo)
    n_frames = int(round(DURATION_S * FPS))
    opaque = alpha >= 0.99

    fig = plt.figure(figsize=FIGSIZE, dpi=DPI)
    ax = fig.add_subplot(111, projection="3d")

    for i in range(n_frames):
        t = i / FPS
        Rz, Ry = rot_z(pan_angle(t)), rot_y(tilt_angle(t))
        ax.clear()
        ax.set_axis_off()
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_zlim(lo[2], hi[2])
        ax.set_box_aspect(span)
        ax.view_init(elev=ELEV, azim=AZIM)
        for e in tess:
            v = transformed(e, Rz, Ry, pan_center, tilt_center)
            polys = v[e["faces"]]
            r, g, b = e["rgb"]
            coll = Poly3DCollection(
                polys,
                facecolor=(r, g, b, alpha),
                edgecolor=(0.1, 0.1, 0.1, 0.15) if opaque else "none",
                linewidths=0.1,
            )
            ax.add_collection3d(coll)
        fig.savefig(frame_dir / f"f{i:04d}.png")
    plt.close(fig)

    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
         "-i", str(frame_dir / "f%04d.png"),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(out_mp4)],
        check=True,
    )


def main(out_dir="videos"):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print("Tessellating assembly...")
    tess, pan_center, tilt_center = tessellate_assembly()
    tri_total = sum(len(e["faces"]) for e in tess)
    print(f"  {len(tess)} parts, {tri_total} triangles")
    bounds = global_bounds(tess, pan_center, tilt_center)

    for alpha, name in ((1.0, "opaque"), (0.35, "transparent")):
        out_mp4 = out_dir / f"pantilt_{name}.mp4"
        frame_dir = Path(tempfile.mkdtemp(prefix=f"pt_{name}_"))
        try:
            print(f"Rendering {name} -> {out_mp4} ...")
            render(tess, pan_center, tilt_center, bounds, alpha, out_mp4, frame_dir)
            print(f"  wrote {out_mp4}")
        finally:
            shutil.rmtree(frame_dir, ignore_errors=True)
    print("Done.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "videos")
