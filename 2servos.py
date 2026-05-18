from pantilt_build123d.sg9_servo import SG9Servo
from pantilt_build123d.upper_swivel_ring import UpperSwivelRing
from build123d import Location, Box, Align, Cylinder
from build123d.geometry import (
    Axis,
    Color,
)
from ocp_vscode.config import Camera
import copy

from ocp_vscode import show

if __name__ == "__main__":
    servo1 = SG9Servo(color=Color("blue")) # pan servo
    top_of_shaft = servo1.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
    
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False) # tilt servo
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((servo2.width/2 +
                                    servo1.gear_cover_clearance_radius + 2,
                                        0, 0))) # Move out to bracket face; no Z offset needed
    
    ring = UpperSwivelRing(servo1)

    mounts = servo1.mounts()
    if mounts["left_mount"] is None and  mounts["right_mount"] is None:
        raise ValueError("At least one mount (left or right) must be present on the pan servo for mounting the tilt servo.")
    else:
        mount = mounts["left_mount"] if mounts["left_mount"] is not None else mounts["right_mount"]
        mount_faces = mount.faces().filter_by(Axis.Z).sort_by(Axis.Z)
        top_of_mount_face = mount_faces[-1]


    body_to_cut = copy.deepcopy(servo1.body)

    shaft_center_x = servo1.horn_mount.position.X               # pan rotation axis X (default 5.0)
    gear_cover_top_z = servo1.body_height / 2 + servo1.gear_cover_height  # 16.5
    plate_z = top_of_mount_face.center().Z                      # Z of mount-ear top face (≈ 6.5)
    plate_size = servo1.bounding_box().diagonal

    # Host mounting plate — centred on pan shaft axis, body slot cut in world space
    mounting_plate_on_host = Box(plate_size, plate_size, 2.5,
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    mounting_plate_on_host = mounting_plate_on_host.move(Location((shaft_center_x, 0, plate_z)))
    mounting_plate_on_host = mounting_plate_on_host - body_to_cut
    mounting_plate_on_host.color = Color("gray")

    # Pan swivel bearing — centred on pan shaft axis, annular (inner bore clears gear cover)
    bearing_z = plate_z + 2.5
    pan_static_bearing = Cylinder(radius=plate_size / 2, height=2.5,
                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_static_bearing = pan_static_bearing.move(Location((shaft_center_x, 0, bearing_z)))
    inner_bore = Cylinder(radius=servo1.width / 2 + 1, height=3,
                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    inner_bore = inner_bore.move(Location((shaft_center_x, 0, bearing_z - 0.5)))
    pan_static_bearing = pan_static_bearing - inner_bore - body_to_cut
    pan_static_bearing.color = Color("green")

    # Upper thrust bearing sits directly on top of pan_static_bearing.
    upper_bearing_z = bearing_z + 2.5  # top face of pan_static_bearing
    upper_bearing_inner = Cylinder(
        radius=servo1.gear_cover_clearance_radius + 0.2, height=2.5,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, upper_bearing_z)))
    upper_bearing = Cylinder(
        radius=20, height=2.5,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, upper_bearing_z)))
    upper_bearing = upper_bearing - upper_bearing_inner
    upper_bearing.color = Color("yellow")

    show(
        [servo1, servo2, ring, mounting_plate_on_host, pan_static_bearing, upper_bearing],
        reset_camera=Camera.KEEP,
    )