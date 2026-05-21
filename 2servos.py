from pantilt_build123d.sg9_servo import SG9Servo
from pantilt_build123d.sg9_servo_horn import SG9ServoHorn
from build123d import Location, Box, Align, Cylinder, Face, Wire, Vector, extrude
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
    _servo2_width = servo2.width  # save before rotate drops custom attrs
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((_servo2_width/2 +
                                    servo1.gear_cover_clearance_radius + 1,
                                        0, 0)))  # X offset only; Z snapped to upper_bearing below

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

    # Upper thrust bearing — annular ring sitting on top of pan_static_bearing.
    # This is the rotating part; it is keyed to servo1's shaft via a horn arm pocket.
    upper_bearing_z = bearing_z + 2.5  # bottom = top face of pan_static_bearing
    upper_bearing_inner = Cylinder(
        radius=servo1.gear_cover_clearance_radius + 0.2, height=2.5,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, upper_bearing_z)))
    upper_bearing = Cylinder(
        radius=plate_size / 2, height=2.5,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, upper_bearing_z)))
    upper_bearing = upper_bearing - upper_bearing_inner

    # Snap servo2 bottom face onto upper_bearing top face
    ub_top_z = upper_bearing.faces().filter_by(Axis.Z).sort_by(Axis.Z)[-1].center().Z
    s2_bot_z = servo2.faces().filter_by(Axis.Z).sort_by(Axis.Z)[0].center().Z
    servo2   = servo2.move(Location((0, 0, ub_top_z - s2_bot_z)))

    # Instantiate horn model; save geometry params before move drops custom attrs.
    _horn = SG9ServoHorn()
    horn_arm_thickness    = _horn.arm_thickness        # 2.5
    horn_arm_width        = _horn.arm_width            # 4.0
    horn_arm_length       = _horn.arm_length           # 17.0
    horn_hub_height       = _horn.hub_height           # 7.5
    horn_hub_outer_radius = _horn.hub_outer_radius     # 4.0
    arm_screw_y           = _horn.arm_hole_positions[-1]  # outermost hole, 15 mm
    horn = _horn.move(Location((shaft_center_x, 0, gear_cover_top_z)))
    horn.color = Color("lightgray")

    # Arm bottom sits above shaft top; top is flush with hub/cup top.
    arm_bottom_z   = gear_cover_top_z + horn_hub_height - horn_arm_thickness   # 21.5
    buildup_bottom = upper_bearing_z + 2.5                                      # 14.0
    buildup_top    = gear_cover_top_z + horn_hub_height                         # 24.0
    buildup_height = buildup_top - buildup_bottom                               # 10.0

    # Box: 14 mm wide in X (centred on shaft), extends +Y from 6 mm behind shaft
    buildup = Box(14, 26, buildup_height,
                  align=(Align.CENTER, Align.MIN, Align.MIN))
    buildup = buildup.move(Location((shaft_center_x, -6, buildup_bottom)))

    # Horn arm pocket: tapered to match arm + circular cap at tip
    _pocket_clr = 0.2  # clearance each side
    _hw_base = horn_hub_outer_radius + _pocket_clr   # half-width at hub end
    _hw_tip  = horn_arm_width / 2    + _pocket_clr   # half-width at tip
    _ph      = horn_arm_thickness + 0.1              # pocket extrusion height
    horn_pocket = extrude(
        Face(Wire.make_polygon([
            Vector(shaft_center_x - _hw_base, -0.5,            arm_bottom_z),
            Vector(shaft_center_x + _hw_base, -0.5,            arm_bottom_z),
            Vector(shaft_center_x + _hw_tip,  horn_arm_length, arm_bottom_z),
            Vector(shaft_center_x - _hw_tip,  horn_arm_length, arm_bottom_z),
        ])),
        _ph,
    )
    horn_pocket_cap = Cylinder(radius=_hw_tip, height=_ph,
                               align=(Align.CENTER, Align.CENTER, Align.MIN))
    horn_pocket_cap = horn_pocket_cap.move(Location((shaft_center_x, horn_arm_length, arm_bottom_z - 0.05)))
    horn_pocket = horn_pocket + horn_pocket_cap

    # M2 screw hole aligned with outermost horn arm hole (arm_screw_y in +Y)
    arm_screw_hole = Cylinder(radius=1.1, height=buildup_height + 1,
                              align=(Align.CENTER, Align.CENTER, Align.MIN))
    arm_screw_hole = arm_screw_hole.move(Location((shaft_center_x, arm_screw_y, buildup_bottom - 0.5)))

    # Gear cover clearance: removes material where gear covers protrude (Z=14..16.5)
    gear_cover_clearance = Cylinder(
        radius=servo1.gear_cover_clearance_radius + 0.2,
        height=gear_cover_top_z - buildup_bottom + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, buildup_bottom)))

    # Hub clearance: full horn hub height above gear cover (Z=16.5..24.0)
    hub_clearance = Cylinder(
        radius=horn_hub_outer_radius + 0.3,
        height=horn_hub_height + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, gear_cover_top_z - 0.05)))

    buildup = buildup - horn_pocket - arm_screw_hole - gear_cover_clearance - hub_clearance
    upper_bearing = upper_bearing + buildup
    upper_bearing.color = Color("yellow")

    show(
        [servo1, servo2, mounting_plate_on_host, pan_static_bearing, upper_bearing, horn],
        reset_camera=Camera.KEEP,
    )
