from pantilt_build123d.sg9_servo import SG9Servo

from build123d.objects_part import Compound, Box, Cylinder
from build123d import  Plane, Shape, Face
from build123d.build_enums import Align 
from build123d.geometry import (
    Axis,
    Color,
    Location,
    Vector
)
from ocp_vscode.config import Camera
import copy

from ocp_vscode import show


def model_pan_static(mounting_plate: Shape = None):
    """
    model_pan_static models the static portion of the pan actuator including the pan servo and its static swivel bearing.

    :param mounting_plate_on_host: If not None[default] The mounting plate on the host structure 
            where the pan servo will be mounted
    
    :return: A tuple containing:
        - mounting_plate: The modified mounting plate with servo body cut out (if mounting_plate was provided)
        - pan_static: A dictionary containing the static components of the pan actuator (TODO: make it a build123d assembly):

    :raises ValueError: If mounting_plate is provided but the pan servo has no mounts to attach the plate.
    """

    ### the static portions of the pan acuator ###
    servo1 = SG9Servo(color=Color("blue")) # pan servo
    servo1.label = "Pan Servo"
    body_to_cut = copy.deepcopy(servo1.body)

    ### model the static portion of the pan actuator bearing ###
    static_swivel_bearing = Cylinder(radius=plate_size / 2, height=2.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    static_swivel_bearing.label = "Pan Static Bearing"
    static_swivel_bearing.color = color=Color("green")
    psb_bottom_face = static_swivel_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
    mpoh_top_face = mounting_plate_on_host.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1]
    psb_translation_vector = mpoh_top_face.center() - psb_bottom_face.center()
    psb_translation_vector.X = 0
    psb_translation_vector.Y = 0
    static_swivel_bearing = static_swivel_bearing.translate(psb_translation_vector)
    static_swivel_bearing.move(Location((servo1.final_shaft.center().X, 0.0, 0.0))) 
    static_swivel_bearing = static_swivel_bearing - body_to_cut

    if mounting_plate is not None:
        ### Align the servo mounting tabs with the top of the mounting plate
        mounts = servo1.mounts()
        if mounts["left_mount"] is None and  mounts["right_mount"] is None:
            raise ValueError("At least one mount (left or right) must be present on the pan servo for mounting the tilt servo.")
        else:
            mount = mounts["left_mount"] if mounts["left_mount"] is not None else mounts["right_mount"]
            mount_faces = mount.faces().filter_by(Axis.Z).sort_by(Axis.Z)
            top_of_mount_face = mount_faces[-1]

        # Move the servo so that the top of the mounting tabs are flush with the top of the mounting plate
        mpoh_bottom_face = mounting_plate.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0]
        translation_vector = mpoh_bottom_face.center() - top_of_mount_face.center()
        translation_vector.X = 0
        translation_vector.Y = 0
        servo1 = servo1.translate(translation_vector)

        ### cut the servo body out of the mounting plate ###
        mounting_plate = mounting_plate - body_to_cut

    assembly = Compound(label="Pan Static Assembly",
                                    children=[ servo1, static_swivel_bearing])
    assembly.servo = servo1
    assembly.swivel_bearing = static_swivel_bearing

    return mounting_plate, assembly

def model_pan_to_tilt_assembly(static_bearing_offset: float, bearing_diameter: float,
                               pan_servo: SG9Servo):
    """
    builds the  model of the pan_to_tilt_assembly
    where: 
       * the part's 0,0,0 local origin is where the screw enters the pan servo final shaft
       * the unit will pivot around Axis.Z at local 0,0,0
       * the top of the static swivel bearing is located at Z = -static_bearing_offset

    :param static_bearing_offset: Description
    :param pan_servo: Description
    """

    ### model the panning portion of the tilt actuator ###
    # swivel bearing Shape
    pan_dynamic_bearing = Cylinder(radius=bearing_diameter / 2, height=pan_servo.gear_cover_height, 
                                   align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing.label = "Pan Dynamic Bearing"
    pan_dynamic_bearing.color = Color("lightgreen")
    pdb_servo_clearance_hole = Cylinder(radius=pan_servo.gear_cover_clearance_radius,
                                       height=pan_servo.gear_cover_height + 1,
                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
    pan_dynamic_bearing = pan_dynamic_bearing - pdb_servo_clearance_hole
    pan_dynamic_bearing.label = "Pan Dynamic Bearing"

    # swivel bearing positioning
    pan_dynamic_bearing = pan_dynamic_bearing.move(
        Location((0.0, 0.0, -static_bearing_offset + 0.2))
        )  # small gap for free movement

    # tilt servo
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False) # tilt servo
    servo2.label = "Tilt Servo"
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90).rotate(Axis.Z, 180)  # Rotate for tilting
    new_var = Location((pan_servo.gear_cover_clearance_radius + 0.2 +
                                   servo2.width / 2,
                                     0, 0))
                
    servo2 = servo2.move(new_var) 
    
    assembly = Compound(label="Pan to Tilt Assembly",
                                    children=[ servo2, pan_dynamic_bearing])
    assembly.servo = servo2
    assembly.swivel_bearing = pan_dynamic_bearing

    return assembly

if __name__ == "__main__":

    ### Model the host attachmet stuff ###
    plate_size = 36  # size of the mounting plate
    mounting_plate_thickeness = 2.5  # thickness of the mounting plate
    mounting_plate_on_host = Box(plate_size, plate_size, mounting_plate_thickeness,
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    mounting_plate_on_host.label = "Mounting Spot"
    mounting_plate_on_host.color = Color("gray")
    mounting_plate_on_host, pan_static_assembly = model_pan_static(mounting_plate_on_host)

    for a in pan_static_assembly.children:
        print(f'{a}')
    servo1 = pan_static_assembly.servo

    ### Define the pan pivot axis. ###
    ## all of the pan actuator components will be defined relative to this axis ##
    pan_pivot_face : Face = servo1.final_shaft.faces().sort_by(Axis.Z)[-1]  # top face of the pan servo shaft
    swivel_bearing_top_face : Face = pan_static_assembly.swivel_bearing.faces().sort_by(Axis.Z)[-1]
    plane : Plane = Plane(swivel_bearing_top_face)
    static_bearing_offset : float = pan_pivot_face.center().distance_to_plane (plane )

    pan_dynamic_assembly = model_pan_to_tilt_assembly(static_bearing_offset, plate_size, servo1)    
    pan_dynamic_assembly.move(pan_pivot_face.center_location)  # move up to final shaft hole

    show( [ pan_dynamic_assembly,
            mounting_plate_on_host,
            pan_static_assembly ],
         reset_camera=Camera.KEEP)