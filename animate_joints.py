from time import sleep

from build123d import Align, Box, Cylinder, Location
from build123d.geometry import Axis, Color
from build123d.joints import RigidJoint, RevoluteJoint
from ocp_vscode import show
from ocp_vscode.config import Camera

from pantilt_build123d.sg9_servo import SG9Servo


def build_scene():
    servo1 = SG9Servo(color=Color("blue"))
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False)
    _servo2_width = servo2.width
    servo2 = servo2.rotate(Axis.Z, 90).rotate(Axis.X, 90)
    servo2 = servo1.horn_mount * servo2
    servo2 = servo2.move(
        Location((servo1.gear_cover_clearance_radius + 0.2 + _servo2_width / 2, 0, 0))
    )

    shaft_loc = Location(servo1.horn_mount.position)
    pan_axis = Axis(shaft_loc.position, (0, 0, 1))
    pan_joint = RevoluteJoint(
        "pan_shaft",
        to_part=servo1,
        axis=pan_axis,
        angle_reference=(1, 0, 0),
    )

    upper_bearing = Cylinder(radius=6, height=2.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    upper_bearing = upper_bearing.move(Location((shaft_loc.position.X, shaft_loc.position.Y, shaft_loc.position.Z)))
    upper_bearing_joint = RigidJoint(
        "upper_bearing_rigid",
        to_part=upper_bearing,
        joint_location=shaft_loc,
    )
    pan_joint.connect_to(upper_bearing_joint, angle=0)

    servo2_rigid = RigidJoint(
        "servo2_rigid",
        to_part=servo2,
        joint_location=shaft_loc,
    )
    upper_bearing_joint.connect_to(servo2_rigid, angle=0)

    servo2_horn_global = servo2.location * servo2.horn_mount
    tilt_axis = Axis(servo2_horn_global.position, (0, 1, 0))
    tilt_joint = RevoluteJoint(
        "tilt_shaft",
        to_part=servo2,
        axis=tilt_axis,
        angle_reference=(0, 0, 1),
    )

    tilt_plate = Box(10, 3, 10, align=(Align.CENTER, Align.MIN, Align.CENTER))
    tilt_plate = tilt_plate.move(
        Location((servo2_horn_global.position.X, servo2_horn_global.position.Y - 2, servo2_horn_global.position.Z))
    )
    tilt_plate_joint = RigidJoint(
        "tilt_plate_rigid",
        to_part=tilt_plate,
        joint_location=servo2_horn_global,
    )
    tilt_joint.connect_to(tilt_plate_joint, angle=0)

    return servo1, servo2, upper_bearing, tilt_plate, pan_joint, upper_bearing_joint, servo2_rigid, tilt_joint, tilt_plate_joint


def animate():
    servo1, servo2, upper_bearing, tilt_plate, pan_joint, upper_bearing_joint, servo2_rigid, tilt_joint, tilt_plate_joint = build_scene()

    for pan_angle in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
        pan_joint.connect_to(upper_bearing_joint, angle=pan_angle)
        upper_bearing_joint.connect_to(servo2_rigid, angle=0)
        for tilt_angle in [0, 15, 30, 45, 30, 15, 0]:
            tilt_joint.connect_to(tilt_plate_joint, angle=tilt_angle)
            show(
                servo1,
                servo2,
                upper_bearing,
                tilt_plate,
                render_joints=True,
                tab="joint_animation",
                reset_camera=Camera.KEEP,
                names=["pan_servo", "tilt_servo", "upper_bearing", "tilt_plate"],
            )
            sleep(0.2)


if __name__ == "__main__":
    animate()
