"""Render the pan-tilt motion to MP4 videos (headless, via VTK + ffmpeg).

Produces two 5-second clips with smooth sinusoidal motion that sweeps the joint
limits (pan -90..+90, tilt -15..+90), two pan cycles:

    videos/pantilt_opaque.mp4        solid volumes
    videos/pantilt_transparent.mp4   see-through volumes

The OCP viewer cannot capture video, so each part is tessellated once into a
VTK surface actor plus a black modeled-edge actor, and the same pan/tilt
rotations used by the kinematic model are applied per frame as VTK transforms.
VTK's offscreen, z-buffered OpenGL renderer gives correct occlusion and
hidden-line edges; depth peeling makes the transparent clip order-independent.

Run:  venv/bin/python render_video.py
Quick smoke test:  QUICK=1 venv/bin/python render_video.py
"""

import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

import ocp_vscode
ocp_vscode.show = lambda *a, **k: None  # never open the viewer from here

from pantilt_build123d.pan_tilt_assembly import build_assembly

DURATION_S = 5.0
FPS = 24
TESS_TOL = 0.4           # mm; coarser = fewer triangles = faster render
EDGE_DEFLECTION = 0.1    # mm; chord tolerance for the black edge hairlines
FIGSIZE = (8, 6)         # inches
DPI = 160                # -> 1280 x 960 (Quad VGA, even / yuv420p-friendly)
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


def edge_polylines(part):
    """Discretize each modeled edge of *part* into an (K,3) polyline.

    Uses a curvature-aware deflection sampler so straight edges become two
    points and curves get just enough points to stay within EDGE_DEFLECTION.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    polylines = []
    for edge in part.edges():
        adaptor = BRepAdaptor_Curve(edge.wrapped)
        disc = GCPnts_QuasiUniformDeflection(adaptor, EDGE_DEFLECTION)
        if not disc.IsDone() or disc.NbPoints() < 2:
            continue
        pts = [adaptor.Value(j) for j in range(1, disc.NbPoints() + 1)]
        polylines.append(np.array([(p.X(), p.Y(), p.Z()) for p in pts]))
    return polylines


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
        polylines = edge_polylines(part)
        if polylines:
            everts = np.concatenate(polylines)
            esizes = [len(p) for p in polylines]
        else:
            everts, esizes = None, []
        tess.append({"name": name, "verts": verts, "faces": faces,
                     "rgb": rgb, "group": group,
                     "everts": everts, "esizes": esizes})
    return tess, pan_center, tilt_center


def _group_apply(verts, group, Rz, Ry, pan_center, tilt_center):
    if group == "pan":
        return _apply(verts, Rz, pan_center)
    if group == "tilt":
        return _apply(_apply(verts, Ry, tilt_center), Rz, pan_center)
    return verts


def transformed(entry, Rz, Ry, pan_center, tilt_center):
    return _group_apply(entry["verts"], entry["group"],
                        Rz, Ry, pan_center, tilt_center)


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


def _surface_polydata(verts, faces):
    """vtkPolyData of triangles from numpy verts (Nx3) and faces (Mx3)."""
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(verts, dtype=np.float64)))
    conn = np.empty((len(faces), 4), dtype=np.int64)
    conn[:, 0] = 3
    conn[:, 1:] = faces
    cells = vtk.vtkCellArray()
    cells.SetCells(len(faces), numpy_to_vtkIdTypeArray(conn.ravel(), deep=1))
    poly = vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetPolys(cells)
    return poly


def _edge_polydata(everts, esizes):
    """vtkPolyData of polylines from concatenated points and per-edge sizes."""
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk

    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(everts, dtype=np.float64)))
    lines = vtk.vtkCellArray()
    off = 0
    for n in esizes:
        lines.InsertNextCell(n)
        for j in range(n):
            lines.InsertCellPoint(off + j)
        off += n
    poly = vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetLines(lines)
    return poly


def render(tess, pan_center, tilt_center, bounds, alpha, out_mp4, frame_dir):
    """Render the clip with VTK's offscreen, z-buffered OpenGL renderer.

    Each part becomes a shaded surface actor plus a black modeled-edge line
    actor; both share a per-group transform that is updated each frame. Real
    depth testing gives correct occlusion (no vanishing parts, no flicker) and
    hidden-line edges (no x-ray); depth peeling makes the transparent clip
    order-independent.
    """
    import math
    import vtk

    width, height = int(FIGSIZE[0] * DPI), int(FIGSIZE[1] * DPI)
    lo, hi = bounds
    center = (lo + hi) / 2.0
    diag = float(np.linalg.norm(hi - lo))
    n_frames = int(round(DURATION_S * FPS))
    opaque = alpha >= 0.99

    vtk.vtkMapper.SetResolveCoincidentTopologyToPolygonOffset()

    ren = vtk.vtkRenderer()
    ren.SetBackground(1, 1, 1)
    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(ren)
    rw.SetSize(width, height)
    if opaque:
        rw.SetMultiSamples(8)
    else:
        rw.SetMultiSamples(0)
        rw.SetAlphaBitPlanes(1)
        ren.SetUseDepthPeeling(1)
        ren.SetMaximumNumberOfPeels(12)
        ren.SetOcclusionRatio(0.0)

    # One transform per kinematic group, updated in place each frame; the
    # static group keeps the identity (no user transform).
    t_pan, t_tilt = vtk.vtkTransform(), vtk.vtkTransform()
    group_tf = {"pan": t_pan, "tilt": t_tilt}

    for e in tess:
        tf = group_tf.get(e["group"])

        sm = vtk.vtkPolyDataMapper()
        sm.SetInputData(_surface_polydata(e["verts"], e["faces"]))
        sa = vtk.vtkActor()
        sa.SetMapper(sm)
        prop = sa.GetProperty()
        prop.SetColor(*e["rgb"])
        prop.SetOpacity(alpha)
        prop.SetAmbient(0.4)
        prop.SetDiffuse(0.6)
        prop.SetSpecular(0.0)
        if tf is not None:
            sa.SetUserTransform(tf)
        ren.AddActor(sa)

        if e["everts"] is not None:
            em = vtk.vtkPolyDataMapper()
            em.SetInputData(_edge_polydata(e["everts"], e["esizes"]))
            # Pull the lines a touch toward the camera so they sit on the
            # surface instead of z-fighting with it.
            em.SetRelativeCoincidentTopologyLineOffsetParameters(-1.0, -1.0)
            ea = vtk.vtkActor()
            ea.SetMapper(em)
            ep = ea.GetProperty()
            ep.SetColor(0, 0, 0)
            ep.SetLineWidth(1.0)
            ep.SetLighting(False)
            if tf is not None:
                ea.SetUserTransform(tf)
            ren.AddActor(ea)

    # Parallel ("technical") camera matching the ELEV/AZIM view angles.
    el, az = math.radians(ELEV), math.radians(AZIM)
    d = np.array([math.cos(el) * math.cos(az),
                  math.cos(el) * math.sin(az),
                  math.sin(el)])
    cam = ren.GetActiveCamera()
    cam.SetParallelProjection(True)
    cam.SetFocalPoint(*center)
    cam.SetPosition(*(center + d * diag * 2.0))
    cam.SetViewUp(0, 0, 1)
    cam.SetParallelScale(diag * 0.52)

    pcx, pcy, pcz = pan_center
    tcx, tcy, tcz = tilt_center
    writer = vtk.vtkPNGWriter()
    for i in range(n_frames):
        t = i / FPS
        pa, ta = pan_angle(t), tilt_angle(t)
        t_pan.Identity()
        t_pan.Translate(pcx, pcy, pcz)
        t_pan.RotateZ(pa)
        t_pan.Translate(-pcx, -pcy, -pcz)
        t_tilt.Identity()
        t_tilt.Translate(pcx, pcy, pcz)
        t_tilt.RotateZ(pa)
        t_tilt.Translate(-pcx, -pcy, -pcz)
        t_tilt.Translate(tcx, tcy, tcz)
        t_tilt.RotateY(ta)
        t_tilt.Translate(-tcx, -tcy, -tcz)
        ren.ResetCameraClippingRange()
        rw.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(rw)
        w2i.ReadFrontBufferOff()
        w2i.Update()
        writer.SetFileName(str(frame_dir / f"f{i:04d}.png"))
        writer.SetInputConnection(w2i.GetOutputPort())
        writer.Write()
    rw.Finalize()

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
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "out_dir",
        nargs="?",
        default="videos",
        help="directory to write the MP4 clips into (default: videos)",
    )
    args = parser.parse_args()
    main(args.out_dir)
