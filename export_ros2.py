"""Export the pan-tilt assembly as ROS2-importable URDF + STL meshes.

Run:
    venv/bin/python export_ros2.py [output_dir]   # default: ./pantilt_description

Produces a ROS-package-style tree::

    <output_dir>/meshes/{base_link,mount_plate,pan_link,tilt_link}.stl
    <output_dir>/urdf/pantilt.urdf

The kinematic tree is tf-friendly: every link frame is identity-oriented and
placed at the joint axis it rotates about, so the URDF joint origins are simple
translations and the axes are unit vectors.

    base_link  (pan servo + static bearing)            [root / fixed base]
      |-- mount_plate  (host mounting plate)           fixed   -- separable
      |-- pan_link  (upper bracket, tilt servo,        continuous +Z (pan)
      |              pan horn, counter-shaft rod)
            |-- tilt_link  (tilt yoke + tilt horn)     revolute  +Y (tilt)

Per the project's ROS2 goal, everything except ``mount_plate`` is the
controllable robot; the mounting plate is a separate fixed link the user can
drop and replace with their own mount.

Meshes are exported in millimetres; the URDF references them with
``scale="0.001 ..."`` and expresses all joint origins in metres.
"""

import sys
from math import radians
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

from build123d import Compound, Pos, export_stl

from pantilt_build123d.pan_tilt_assembly import build_assembly

MM_TO_M = 0.001
PACKAGE = "pantilt_description"

# Reasonable placeholder actuator limits (SG90-class hobby servos).
EFFORT_NM = 0.2
VELOCITY_RAD_S = 6.0

# link name -> part names that make up that rigid body
LINK_PARTS = {
    "base_link": ["pan_servo", "pan_static_bearing"],
    "mount_plate": ["host_plate"],
    "pan_link": ["upper_pan_bracket", "tilt_servo", "pan_horn", "counter_shaft_rod"],
    "tilt_link": ["tilt_yoke", "tilt_horn"],
}

# RViz display colour per link (rgba)
LINK_RGBA = {
    "base_link": "0.5 0.5 0.5 1",
    "mount_plate": "0.4 0.4 0.4 1",
    "pan_link": "0.9 0.8 0.1 1",
    "tilt_link": "0.1 0.8 0.9 1",
}


def link_frame_world(name, asm):
    """World-space origin (mm) of each link's frame."""
    if name == "pan_link":
        p = asm["pan_joint"].location.position
        return (p.X, p.Y, p.Z)
    if name == "tilt_link":
        p = asm["tilt_joint"].location.position
        return (p.X, p.Y, p.Z)
    return (0.0, 0.0, 0.0)  # base_link and mount_plate share the world frame


def export_meshes(asm, mesh_dir):
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for link, part_names in LINK_PARTS.items():
        ox, oy, oz = link_frame_world(link, asm)
        # Express each part in the link frame (origin at the joint axis).
        moved = [Pos(-ox, -oy, -oz) * asm["parts"][n] for n in part_names]
        compound = Compound(children=moved)
        export_stl(compound, str(mesh_dir / f"{link}.stl"))
        print(f"  meshes/{link}.stl  ({len(part_names)} parts)")


def _xyz_m(vec_mm):
    return " ".join(f"{c * MM_TO_M:.6f}" for c in vec_mm)


def _add_link(robot, name):
    link = ET.SubElement(robot, "link", name=name)
    mesh_attr = {
        "filename": f"package://{PACKAGE}/meshes/{name}.stl",
        "scale": "0.001 0.001 0.001",
    }
    for tag in ("visual", "collision"):
        node = ET.SubElement(link, tag)
        ET.SubElement(node, "origin", xyz="0 0 0", rpy="0 0 0")
        geom = ET.SubElement(node, "geometry")
        ET.SubElement(geom, "mesh", **mesh_attr)
        if tag == "visual":
            mat = ET.SubElement(node, "material", name=f"{name}_color")
            ET.SubElement(mat, "color", rgba=LINK_RGBA[name])


def build_urdf(asm):
    robot = ET.Element("robot", name="pantilt")
    for name in LINK_PARTS:
        _add_link(robot, name)

    pan_o = link_frame_world("pan_link", asm)
    tilt_o = link_frame_world("tilt_link", asm)
    tilt_rel = tuple(t - p for t, p in zip(tilt_o, pan_o))  # both identity-oriented

    # mount plate: fixed, separable interface to the host
    mj = ET.SubElement(robot, "joint", name="mount_joint", type="fixed")
    ET.SubElement(mj, "parent", link="base_link")
    ET.SubElement(mj, "child", link="mount_plate")
    ET.SubElement(mj, "origin", xyz="0 0 0", rpy="0 0 0")

    # pan: full continuous rotation about +Z
    pj = ET.SubElement(robot, "joint", name="pan_joint", type="continuous")
    ET.SubElement(pj, "parent", link="base_link")
    ET.SubElement(pj, "child", link="pan_link")
    ET.SubElement(pj, "origin", xyz=_xyz_m(pan_o), rpy="0 0 0")
    ET.SubElement(pj, "axis", xyz="0 0 1")
    ET.SubElement(pj, "limit", effort=str(EFFORT_NM), velocity=str(VELOCITY_RAD_S))

    # tilt: revolute about +Y, limited to the model's declared range
    tilt_lo, tilt_hi = asm["tilt_joint"].angular_range
    tj = ET.SubElement(robot, "joint", name="tilt_joint", type="revolute")
    ET.SubElement(tj, "parent", link="pan_link")
    ET.SubElement(tj, "child", link="tilt_link")
    ET.SubElement(tj, "origin", xyz=_xyz_m(tilt_rel), rpy="0 0 0")
    ET.SubElement(tj, "axis", xyz="0 1 0")
    ET.SubElement(
        tj, "limit",
        lower=f"{radians(tilt_lo):.4f}", upper=f"{radians(tilt_hi):.4f}",
        effort=str(EFFORT_NM), velocity=str(VELOCITY_RAD_S),
    )
    return robot


def main(out_dir):
    out_dir = Path(out_dir)
    asm = build_assembly()

    print(f"Exporting ROS2 description to {out_dir}/")
    export_meshes(asm, out_dir / "meshes")

    urdf_dir = out_dir / "urdf"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    robot = build_urdf(asm)
    pretty = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    urdf_path = urdf_dir / "pantilt.urdf"
    urdf_path.write_text(pretty)
    print(f"  urdf/pantilt.urdf  ({len(LINK_PARTS)} links, 3 joints)")
    print("Done.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else PACKAGE)
