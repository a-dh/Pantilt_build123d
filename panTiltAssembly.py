from pantilt_build123d.sg9_servo import SG9Servo
from pantilt_build123d.sg9_servo_horn import SG9ServoHorn, SG9ServoHornPocket
from build123d import Location, Box, Align, Cylinder, Plane, RevoluteJoint, RigidJoint
from build123d.geometry import (
    Axis,
    Color,
)
from ocp_vscode.config import Camera
import copy

from ocp_vscode import show


def build_assembly():
    """Build the full pan-tilt assembly.

    Returns a dict so both the static viewer (``__main__`` below) and
    animate_joints.py can use the same geometry:

      - ``parts``: ordered name -> Part (in show() order)
      - ``pan_center`` / ``tilt_center``: points on the pan (+Z) and tilt (+Y)
        rotation axes, for animating the parts
      - ``panned``: names of parts that rotate with the pan axis
      - ``tilted``: names of parts that additionally rotate with the tilt axis
    """
    servo1 = SG9Servo(color=Color("blue")) # pan servo
    top_of_shaft = servo1.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]

    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False) # tilt servo
    _servo2_width = servo2.width  # save before rotate drops custom attrs
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((servo1.gear_cover_clearance_radius + 0.2 + _servo2_width/2,
                                        0, 0)))  # -X face at edge of gear cover clearance bore

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

    # Kinematic joint for the pan shaft: servo1 drives the upper bearing.
    pan_shaft_location = Location(servo1.horn_mount.position)
    pan_joint = RevoluteJoint(
        "pan_shaft",
        to_part=servo1,
        axis=Axis(pan_shaft_location.position, (0, 0, 1)),
        angle_reference=(1, 0, 0),
    )
    upper_bearing_joint = RigidJoint(
        "upper_bearing_rigid",
        to_part=upper_bearing,
        joint_location=pan_shaft_location,
    )
    pan_joint.connect_to(upper_bearing_joint, angle=0)

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

    # Box: 14 mm wide in X, symmetric in Y to contain the full double-ended arm
    _buildup_half_y = horn_arm_length + horn_arm_width / 2 + 1   # 20 mm
    buildup = Box(14, 2 * _buildup_half_y, buildup_height,
                  align=(Align.CENTER, Align.CENTER, Align.MIN))
    buildup = buildup.move(Location((shaft_center_x, 0, buildup_bottom)))

    horn_pocket = SG9ServoHornPocket(horn, clearance=0.2, extra_depth=0.2)
    
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



    buildup = buildup - horn_pocket - arm_screw_hole - gear_cover_clearance
    upper_bearing = upper_bearing + buildup
    upper_bearing.color = Color("yellow")

    # Servo2 bracket — C-clamp around the ear tab only.
    # Sits on top of the servo2 body (bracket -Z = body top in world Z).
    # Open at -Y: ear tab slides in from below; M2 screw from -Y threads into +Y wall.
    wall_t = 2.5
    s2_bb  = servo2.bounding_box()
    s2_cx  = (s2_bb.min.X + s2_bb.max.X) / 2

    # Ear tab world extents (SG9Servo defaults, after servo2 rotation)
    # Original ear z_pos = -body_height/2 + 17 = 5.5 → world Y = -5.5 after rotate(X,90)
    ear_hole_z = s2_bb.min.Z + servo1.length + 4.0   # world Z of ear hole = 40
    ear_top_y  = servo1.body_height / 2 - 17 + 1.0   # +Y face of ear tab  = -4.5
    ear_bot_z  = ear_hole_z - 4.0                     # -Z face of ear = body top = 36
    ear_top_z  = ear_hole_z + 4.0                     # +Z face of ear = 44

    b_min_y = ear_top_y                               # bracket open at -Y = ear +Y face
    b_max_y = s2_bb.max.Y + wall_t                    # lower section +Y wall outer face = 14
    b_min_z = buildup_bottom                          # bracket bottom = top of large cylinder of swivel ring = 14
    # Cap thinned from +Z: stop just wall_t above screw hole centre
    # Cap thinned from +Y: only as deep as 8mm screw + back wall
    b_max_z     = ear_hole_z + wall_t                 # cap top = screw centre + wall_t = 42.5
    b_cap_max_y = b_min_y + 8.0 + wall_t             # cap +Y face: 8mm screw depth + wall = 6.0
    cap_z_start = ear_bot_z + 0.2                     # cap starts just above body top = 36.2

    # Lower section: full Y width, encloses servo2 body from swivel ring top to body top
    outer_lower = Box(
        s2_bb.size.X + 2 * wall_t,
        b_max_y - b_min_y,
        cap_z_start - b_min_z,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).move(Location((s2_cx, b_min_y, b_min_z)))

    # Cap section: narrower in Y and Z, just enough for the M2 screw
    outer_cap = Box(
        s2_bb.size.X + 2 * wall_t,
        b_cap_max_y - b_min_y,
        b_max_z - cap_z_start,
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).move(Location((s2_cx, b_min_y, cap_z_start)))

    # Counter-shaft boss: 3 mm smooth rod, collinear with servo2's output shaft
    rod_d = 3.0
    boss_r = rod_d / 2 + wall_t                      # 4.0 mm outer radius
    boss_len = 6.0 - wall_t                           # protrusion beyond +Y wall so total bore depth = 6 mm
    shaft_axis_z = s2_bb.min.Z + shaft_center_x + servo1.length / 2  # = 30.0

    boss = Cylinder(radius=boss_r, height=boss_len,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss = boss.rotate(Axis.X, -90)                  # axis in +Y
    boss = boss.move(Location((s2_cx, b_max_y, shaft_axis_z)))

    # Bore sized for press-fit 3 mm rod; use rod_d/2 and calibrate slicer tolerance
    rod_bore = Cylinder(radius=rod_d / 2,
                        height=boss_len + wall_t + 0.1,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    rod_bore = rod_bore.rotate(Axis.X, -90)
    rod_bore = rod_bore.move(Location((s2_cx, b_max_y - wall_t - 0.05, shaft_axis_z)))

    outer = outer_lower + outer_cap + boss

    # Ear slot cavity: removes inner material of lower section except ±X and +Y walls
    cav_y = b_max_y - b_min_y - wall_t + 1.0         # overshoot -Y by 1 mm for clean boolean
    cavity = Box(
        s2_bb.size.X + 0.4,
        cav_y,
        cap_z_start - b_min_z,               # matches lower section height exactly
        align=(Align.CENTER, Align.MIN, Align.MIN),
    ).move(Location((s2_cx, b_min_y - 1.0, b_min_z)))

    # M2 screw hole in Y through the cap, aligned with ear tab hole
    screw_y_span = b_cap_max_y - b_min_y + 2
    screw_hole = Cylinder(radius=1.0, height=screw_y_span,
                          align=(Align.CENTER, Align.CENTER, Align.CENTER))
    screw_hole = screw_hole.rotate(Axis.X, 90)
    screw_hole = screw_hole.move(Location((s2_cx, (b_min_y + b_cap_max_y) / 2, ear_hole_z)))

    # Gear cover clearance: bracket rotates with the upper bearing; bore out the same
    # cylinder used in the upper bearing so the -X wall clears servo1's gear cover.
    gc_clearance = Cylinder(
        radius=servo1.gear_cover_clearance_radius + 0.2,
        height=gear_cover_top_z - b_min_z + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).move(Location((shaft_center_x, 0, b_min_z)))

    servo2_bracket = outer - cavity - screw_hole - gc_clearance - rod_bore

    # Co-printed single piece: upper pan swivel bearing + tilt bracket
    upper_pan_bracket = upper_bearing + servo2_bracket
    upper_pan_bracket.color = Color("yellow")

    _tb_clr = 0.2
    tilt_plate_y = horn_arm_thickness + _tb_clr + wall_t   # pocket depth + clearance + back wall = 5.2 mm
    counter_plate_y_start = b_max_y + boss_len             # inner face of counter-shaft tilt plate, flush with boss tip

    # Counter-shaft rod: extends from servo2 body through boss bore and through counter-shaft tilt plate
    rod2 = Cylinder(radius=rod_d / 2,
                    height=(counter_plate_y_start + tilt_plate_y + 0.5) - (s2_bb.max.Y + 0.1),
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    rod2 = rod2.rotate(Axis.X, -90)          # axis in +Y
    rod2 = rod2.move(Location((s2_cx, s2_bb.max.Y + 0.1, shaft_axis_z)))
    rod2.color = Color("silver")

    # Servo2 horn: hub points in -Y (servo2 shaft direction), arm extends in -X
    servo2_gear_cover_top_y = -(servo1.body_height / 2 + servo1.gear_cover_height)  # = -16.5
    horn2 = SG9ServoHorn()
    horn2 = horn2.rotate(Axis.X, 90)    # hub → -Y, arm → +Z
    horn2 = horn2.rotate(Axis.Y, -90)   # arm → -X horizontal
    horn2 = horn2.move(Location((s2_cx, servo2_gear_cover_top_y, shaft_axis_z)))
    horn2.color = Color("lightgray")

    # Tilt plate: receives servo2 horn arm in a -Y-face pocket
    horn_arm_neg_y = servo2_gear_cover_top_y - horn_hub_height  # -Y face of horn arm = -24.0

    tb_x_max = s2_cx + 5.0           # +X end: 5 mm past servo2 shaft = 25.2
    tb_x_min = tb_x_max - 40.0       # -X end = -24.8

    # Shaft-side tilt plate: receives servo2 horn arm in a -Y-face pocket
    tilt_plate = Box(40, tilt_plate_y, 10,
                     align=(Align.MIN, Align.MIN, Align.CENTER))
    tilt_plate = tilt_plate.move(Location((tb_x_min, horn_arm_neg_y, shaft_axis_z)))

    arm_pocket = SG9ServoHornPocket(SG9ServoHorn(), clearance=0.2, extra_depth=0.4)
    arm_pocket = arm_pocket.rotate(Axis.X, 90)    # hub → -Y, arm → +Z
    arm_pocket = arm_pocket.rotate(Axis.Y, -90)   # arm → -X horizontal
    arm_pocket = arm_pocket.move(Location((s2_cx, servo2_gear_cover_top_y, shaft_axis_z)))

    tilt_plate = tilt_plate - arm_pocket

    # Counter-shaft tilt plate: mirrors tilt_plate on the +Y side, pivots on counter_shaft_rod
    tilt_plate2 = Box(40, tilt_plate_y, 10,
                      align=(Align.MIN, Align.MIN, Align.CENTER))
    tilt_plate2 = tilt_plate2.move(Location((tb_x_min, counter_plate_y_start, shaft_axis_z)))

    rod_clearance_bore = Cylinder(radius=rod_d / 2 + 0.2,
                                  height=tilt_plate_y + 1.0,
                                  align=(Align.CENTER, Align.CENTER, Align.CENTER))
    rod_clearance_bore = rod_clearance_bore.rotate(Axis.X, 90)   # bore axis in +Y
    rod_clearance_bore = rod_clearance_bore.move(
        Location((s2_cx, counter_plate_y_start + tilt_plate_y / 2, shaft_axis_z)))

    tilt_plate2 = tilt_plate2 - rod_clearance_bore

    # -X end plate: joins tilt_plate and tilt_plate2 at their -X edges
    tilt_end_plate = Box(tilt_plate_y,
                         counter_plate_y_start + tilt_plate_y - horn_arm_neg_y,
                         10,
                         align=(Align.MIN, Align.MIN, Align.CENTER))
    tilt_end_plate = tilt_end_plate.move(Location((tb_x_min, horn_arm_neg_y, shaft_axis_z)))

    # Co-printed single piece: both tilt plates joined by the -X end plate (U-yoke)
    tilt_yoke = tilt_plate + tilt_plate2 + tilt_end_plate
    tilt_yoke.color = Color("cyan")

    # Kinematic joint for the tilt shaft: servo2 drives the tilt yoke.
    # tilt_yoke is a boolean union, so its .location is identity and its
    # geometry lives in world coords. The RigidJoint frame must therefore
    # match the revolute joint's world frame at angle 0 (axis +Y, reference
    # +Z) so connect_to leaves the as-built yoke in place instead of
    # relocating it off both shafts.
    servo2_horn_global = servo2.location * servo2.horn_mount
    tilt_joint = RevoluteJoint(
        "tilt_shaft",
        to_part=servo2,
        axis=Axis(servo2_horn_global.position, (0, 1, 0)),
        angle_reference=(0, 0, 1),
    )
    tilt_frame = Plane(origin=servo2_horn_global.position, x_dir=(0, 0, -1), z_dir=(0, 1, 0))
    tilt_yoke_joint = RigidJoint(
        "tilt_yoke_rigid",
        to_part=tilt_yoke,
        joint_location=Location(tilt_frame),
    )
    tilt_joint.connect_to(tilt_yoke_joint, angle=0)

    parts = {
        "pan_servo": servo1,
        "tilt_servo": servo2,
        "host_plate": mounting_plate_on_host,
        "pan_static_bearing": pan_static_bearing,
        "upper_pan_bracket": upper_pan_bracket,
        "pan_horn": horn,
        "counter_shaft_rod": rod2,
        "tilt_horn": horn2,
        "tilt_yoke": tilt_yoke,
    }
    return {
        "parts": parts,
        "pan_center": pan_shaft_location.position,
        "tilt_center": servo2_horn_global.position,
        # Everything above the pan bearing turns with the pan axis...
        "panned": ["upper_pan_bracket", "tilt_servo", "pan_horn",
                   "counter_shaft_rod", "tilt_horn", "tilt_yoke"],
        # ...and the yoke plus servo2's own horn additionally turn with tilt.
        "tilted": ["tilt_horn", "tilt_yoke"],
    }


if __name__ == "__main__":
    assembly = build_assembly()
    parts = assembly["parts"]
    show(
        *parts.values(),
        names=list(parts.keys()),
        reset_camera=Camera.KEEP,
    )
