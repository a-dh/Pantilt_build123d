from build123d import (
    BuildSketch,
    Circle,
    Face,
    Part,
    Plane,
    Polygon,
    Shape,
    Solid,
    extrude,
    fillet,
    make_face,
)
from build123d.build_enums import Align, Mode
from build123d.geometry import Axis, Color, Location, Pos, Rot
from build123d.objects_curve import BuildLine, Line, ThreePointArc
from build123d.objects_part import Box, Compound, Cylinder


class SG9Servo(Part):
    """
    SG9Servo models a standard SG9 servo motor with optional mounting ears.

    In real life the servo is regularly sold with two mounting ears, one on each side.
    This model allows for either, both, or neither ear to be included/excluded. Perform
     exclusions in real life with a saw.

    """

    def __init__(
        self,
        servo_length=22,
        servo_width=12,
        servo_height=23,
        cover_length=12,
        cover_height=5,
        shaft_diameter=5,
        shaft_height=10,
        spline_teeth=25,
        spline_depth=0.4,
        ear_thickness=2,
        ear_height_pos=17,
        ear_hole_dia=2,
        ear_hole_offset=4,
        color=Color("lightgray"),
        left_mount=True,
        right_mount=True,
        **kwargs,
    ) -> None:
        self.width: int = servo_width
        self.body_height: int = servo_height  # do not interfere with Part.height
        self.length: int = servo_length
        self.gear_cover_height: int = cover_height
        self.gear_cover_clearance_radius: float = 3 * servo_width / 4

        # --- Body ---
        self.body = Box(servo_length, servo_width, servo_height)

        # --- Shaft and Splines ---
        shaft_center_x: float = servo_length / 2 - cover_length / 2
        final_shaft = Cylinder(radius=shaft_diameter / 2, height=shaft_height)
        final_shaft = (
            Pos(shaft_center_x, 0, servo_height / 2 + shaft_height / 2) * final_shaft
        )

        # --- Use the Top Face of the Shaft as a Reference ---
        # Select the face with the highest Z coordinate
        shaft_top_face: Face = final_shaft.faces().sort_by(Axis.Z)[-1]
        shaft_top_plane: Plane = Plane(shaft_top_face)

        # Create the hole relative to this plane (Z=0 on the plane is the face surface)
        screw_hole = shaft_top_plane * Cylinder(radius=1, height=shaft_height)

        teeth_list = []
        for i in range(spline_teeth):
            angle: float = i * (360 / spline_teeth)
            tooth = Cylinder(radius=spline_depth, height=shaft_height / 2)
            tooth = (
                Pos(shaft_center_x, 0, servo_height / 2 + 3 * shaft_height / 4)
                * Rot(0, 0, angle)
                * Pos(shaft_diameter / 2, 0, 0)
                * tooth
            )
            teeth_list.append(tooth)
        spline = Compound(teeth_list)

        # --- Gear Covers ---
        final_gear_cover = Cylinder(radius=servo_width / 2, height=cover_height)
        self.final_gear_cover = (
            Pos(shaft_center_x, 0, servo_height / 2 + cover_height / 2)
            * final_gear_cover
        )

        penultimate_gear_cover = Cylinder(radius=servo_width / 4, height=cover_height)
        self.penultimate_gear_cover = (
            Pos(
                shaft_center_x - servo_width / 2, 0, servo_height / 2 + cover_height / 2
            )
            * penultimate_gear_cover
        )

        # --- Side Ears ---
        ear_length: int = 2 * ear_hole_offset
        ear_geom = Box(ear_length, servo_width, ear_thickness)
        ear_geom -= Pos(ear_length / 2 - ear_hole_offset, 0, 0) * Cylinder(
            radius=ear_hole_dia / 2, height=ear_thickness + 1
        )

        # Position ears relative to the bottom of the body and keep track of where they
        #  are for clients
        z_pos: float = -servo_height / 2 + ear_height_pos
        if left_mount:
            self.left_mount = (
                Pos(servo_length / 2 + ear_length / 2, 0, z_pos) * ear_geom
            )
        else:
            self.left_mount = None

        if right_mount:
            self.right_mount = (
                Pos(-(servo_length / 2 + ear_length / 2), 0, z_pos) * ear_geom
            )
        else:
            self.right_mount = None

        self.top_of_gear_cover_face: Face = (
            final_gear_cover.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
        )

        # --- Assembly and Finishing ---
        servo_shape = (
            self.body
            + final_shaft
            + spline
            + self.final_gear_cover
            + self.penultimate_gear_cover
            + self.left_mount
            + self.right_mount
        )
        servo_shape -= screw_hole
        self.final_shaft: Shape = final_shaft

        # Initialize the Part with the constructed shape
        super().__init__(servo_shape.wrapped, **kwargs)
        self.color = color

    def mounts(self):
        pass  # Replace this with the actual implementation for the mounts method

        return {"right_mount": self.right_mount, "left_mount": self.left_mount}


def tapered_bar(
    length: float = 12.0,
    width_start: float = 8.0,
    width_end: float = 3.0,
    thickness: float = 2.0,
) -> Solid:
    """
    Tapered bar constructed from two end circles connected by a polygon.
    Circles provide rounded ends; polygon provides taper.
    """

    r_start = width_start / 2
    r_end = width_end / 2

    # Circle centers
    cL = Location((0, 0))
    cR = Location((length, 0))

    # Tangent polygon connection points (top/bottom)
    pL_top = (0, r_start)
    pL_bot = (0, -r_start)
    pR_top = (length, r_end)
    pR_bot = (length, -r_end)

    with BuildSketch() as s:
        # Add both end circles
        Circle(r_start, mode=Mode.ADD).move(cL)
        Circle(r_end, mode=Mode.ADD).move(cR)

        # Add polygon between the ends to create the taper
        Polygon([pL_top, pR_top, pR_bot, pL_bot], mode=Mode.ADD)

        # Union automatically from ADD modes; face is implicit
    return extrude(s.sketch, thickness)


class SG9ServoHorn(Part):
    """
    A simple servo horn for the SG9 servo

    """

    def __init__(
        self,
        horn_diameter=8,
        horn_thickness=3,
        screw_hole_dia=2,
        shaft_diameter=5,
        **kwargs,
    ) -> None:
        """
        Docstring for __init__

        :param self: Description
        :param horn_diameter: Description
        :param horn_thickness: Description
        :param screw_hole_dia: Description

        """

        align_center_top = (Align.CENTER, Align.CENTER, Align.MAX)
        align_center_bottom = (Align.CENTER, Align.CENTER, Align.MIN)

        # the top of the splined shaft hole is the natural reference for the horn
        splined_shaft_hole = Cylinder(
            radius=shaft_diameter / 2, height=horn_thickness, align=align_center_top
        )
        shaft_socket = Cylinder(
            radius=horn_diameter / 2, height=horn_thickness, align=align_center_bottom
        )
        screw_hole = Cylinder(
            radius=screw_hole_dia / 2,
            height=horn_thickness + 1,
            align=align_center_bottom,
        )
        shaft_socket -= Pos(0, 0, -1) * splined_shaft_hole
        shaft_socket -= Pos(0, 0, +0.5) * screw_hole

        horn = shaft_socket
        bar = tapered_bar(
            length=20,
            width_start=horn_diameter,
            width_end=horn_diameter / 2,
            thickness=1.5,
        )

        horn += bar

        super().__init__(horn.wrapped, **kwargs)


if __name__ == "__main__":
    from ocp_vscode import show

    servo = SG9Servo(label="SG9 Servo")
    horn = SG9ServoHorn(label="SG9 Servo Horn")
    show(servo, horn)
