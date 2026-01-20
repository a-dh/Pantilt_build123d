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

    ### Model the host attachmet stuff ###
    plate_size = 36  # size of the mounting plate
    mounting_plate_on_host = Box(plate_size, plate_size, 2.5,
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    mounting_plate_on_host.label = "Mounting Spot"
    mounting_plate_on_host.color = Color("gray")
    mpoh_bottom_face = mounting_plate_on_host.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    # When the mounted servo is constructed, cut a hole with assembly clearances for it in the mounting plate

    ### the static portions of the pan acuator ###
    servo1 = SG9Servo(color=Color("blue")) # pan servo
    servo1.label = "Pan Servo"
    top_of_shaft = servo1.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
    
    mounts = servo1.mounts()
    if mounts["left_mount"] is None and  mounts["right_mount"] is None:
        raise ValueError("At least one mount (left or right) must be present on the pan servo for mounting the tilt servo.")
    else:
        mount = mounts["left_mount"] if mounts["left_mount"] is not None else mounts["right_mount"]
        mount_faces = mount.faces().filter_by(Axis.Z).sort_by(Axis.Z)
        top_of_mount_face = mount_faces[-1]

    ### cut the servo body out of the mounting plate ###
    body_to_cut = copy.deepcopy(servo1.body)
    mounting_plate_on_host = mounting_plate_on_host - body_to_cut

    # Move the servo so that the top of the mounting tabs are flush with the top of the mounting plate
    translation_vector = mpoh_bottom_face.center() - top_of_mount_face.center()
    translation_vector.X = 0
    translation_vector.Y = 0
    servo1 = servo1.translate(translation_vector)

    ### model the static portion of the pan actuator bearing ###
    pan_static_bearing = Cylinder(radius=plate_size / 2, height=2.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_static_bearing.color = color=Color("green")
    pan_static_bearing = pan_static_bearing - body_to_cut
    psb_bottom_face = pan_static_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    mpoh_top_face = mounting_plate_on_host.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    psb_translation_vector = mpoh_top_face.center() - psb_bottom_face.center()
    psb_translation_vector.X = 0
    psb_translation_vector.Y = 0
    pan_static_bearing = pan_static_bearing.translate(psb_translation_vector)

    ### model the panning portion of the tilt actuator ###
    # swivel bearing
    pan_dynamic_bearing = Cylinder(radius=plate_size / 2, height=servo1.gear_cover_height, 
                                   align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing.label = "Pan Dynamic Bearing"
    pan_dynamic_bearing.color = color=Color("lightgreen")
    pdb_servo_clearance_hole = Cylinder(radius=servo1.gear_cover_clearance_radius,
                                       height=servo1.gear_cover_height + 1,
                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing = pan_dynamic_bearing - pdb_servo_clearance_hole
    psb_top_face = pan_static_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    pdb_bottom_face = pan_dynamic_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    pdb_translation_vector = psb_top_face.center() - pdb_bottom_face.center()
    pan_dynamic_bearing = pan_dynamic_bearing.translate(pdb_translation_vector)
    pan_dynamic_bearing = pan_dynamic_bearing.move(Location((0,0,.1)))  # small gap for free movement

    # tilt servo
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False) # tilt servo
    servo2.label = "Tilt Servo"
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((servo2.width/2 +
                                    servo1.gear_cover_clearance_radius + 2,
                                        0,
                                        servo1.gear_cover_height +0.25))) # Move out to avoid collision

    show([servo1, servo2, mounting_plate_on_host, pan_static_bearing, pan_dynamic_bearing],
         reset_camera=Camera.KEEP)