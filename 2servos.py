from pantilt_build123d.sg9_servo import SG9Servo
from build123d import Location, Box, Align, Cylinder
from build123d.geometry import (
    Axis,
    Color,
)
from ocp_vscode.config import Camera
import copy

from ocp_vscode import show

if __name__ == "__main__":
    servo1 = SG9Servo(color=Color("blue"), label="pan servo") # pan servo
    top_of_shaft = servo1.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
    
    
    mounts = servo1.mounts()
    if mounts["left_mount"] is None and  mounts["right_mount"] is None:
        raise ValueError("At least one mount (left or right) must be present on the pan servo for mounting the tilt servo.")
    else:
        mount = mounts["left_mount"] if mounts["left_mount"] is not None else mounts["right_mount"]
        mount_faces = mount.faces().filter_by(Axis.Z).sort_by(Axis.Z)
        top_of_mount_face = mount_faces[-1]


    body_to_cut = copy.deepcopy(servo1.body)

    # the mounting plate. TODO: make this the starting point instead of the pan servo
    plate_size = servo1.bounding_box().diagonal
    mounting_plate_on_host = Box(plate_size, plate_size, 2.5,
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    mounting_plate_on_host.color = Color("gray")
    mpoh_bottom_face = mounting_plate_on_host.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    translation_vector = top_of_mount_face.center() - mpoh_bottom_face.center()
    translation_vector.X = 0
    translation_vector.Y = 0
    mounting_plate_on_host = mounting_plate_on_host - body_to_cut
    mounting_plate_on_host = mounting_plate_on_host.translate(translation_vector)

    pan_static_bearing = Cylinder(radius=plate_size / 2, height=2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_static_bearing.color = color=Color("green")
    pan_static_bearing = pan_static_bearing - body_to_cut
    psb_bottom_face = pan_static_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    mpoh_top_face = mounting_plate_on_host.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    psb_translation_vector = mpoh_top_face.center() - psb_bottom_face.center()
    psb_translation_vector.X = 0
    psb_translation_vector.Y = 0
    pan_static_bearing = pan_static_bearing.translate(psb_translation_vector)

    # the panning but not tilting Part
    pan_dynamic_bearing = Cylinder(radius=plate_size / 2,
                                      height= 2,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing = pan_dynamic_bearing - Cylinder(radius=servo1.gear_cover_clearance_radius,
                                                        height=2.5,
                                                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing.color = Color("lightgreen")
    
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False, label='tilt servo') # tilt servo
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((servo2.width/2 +
                                    servo1.gear_cover_clearance_radius + 2,
                                        0,
                                        servo1.gear_cover_height +0.25))) # Move out to avoid collision

    # the panning and tilting fixture

    show([servo1, servo2, mounting_plate_on_host, pan_static_bearing, pan_dynamic_bearing],
         reset_camera=Camera.KEEP)
