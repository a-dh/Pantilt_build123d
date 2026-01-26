import weakref
from typing import Tuple

from build123d import (
    BuildSketch,
    Circle,
    Face,
    Locations,
    Part,
    Plane,
    Polygon,
    Shape,
    Solid,
    extrude,
)
from build123d.build_enums import Align, Mode
from build123d.geometry import Axis, Location, Pos, Rot
from build123d.objects_part import Box, Compound, Cylinder


class SG9Servo_Assembly(Compound):
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
        self.body.label = "SG9 Servo Body"

        # --- Shaft and Splines ---
        shaft_center_x: float = servo_length / 2 - cover_length / 2
        final_shaft = Cylinder(radius=shaft_diameter / 2, height=shaft_height)
        self.final_shaft = (
            Pos(shaft_center_x, 0, servo_height / 2 + shaft_height / 2) * final_shaft
        )
        final_shaft.label = "SG9 Servo Shaft"

        # --- Use the Top Face of the Shaft as a Reference ---
        # Select the face with the highest Z coordinate
        shaft_top_face: Face = final_shaft.faces().sort_by(Axis.Z)[-1]
        shaft_top_plane: Plane = Plane(shaft_top_face)

        # Create the hole relative to this plane (Z=0 on the plane is the face surface)
        screw_hole = shaft_top_plane * Cylinder(
            radius=1,
            height=shaft_height / 2,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        )
        self.final_shaft -= screw_hole

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
        self.spline = Compound(teeth_list)
        self.final_shaft += self.spline

        # --- Gear Covers ---
        final_gear_cover = Cylinder(radius=servo_width / 2, height=cover_height)
        self.final_gear_cover = (
            Pos(shaft_center_x, 0, servo_height / 2 + cover_height / 2)
            * final_gear_cover
        )
        self.final_gear_cover.label = "SG9 Servo Final Gear Cover"

        penultimate_gear_cover = Cylinder(radius=servo_width / 4, height=cover_height)
        self.penultimate_gear_cover = (
            Pos(
                shaft_center_x - servo_width / 2, 0, servo_height / 2 + cover_height / 2
            )
            * penultimate_gear_cover
        )
        self.penultimate_gear_cover.label = "SG9 Servo Penultimate Gear Cover"

        # --- Side Ears ---
        ear_length: int = 2 * ear_hole_offset
        ear_geom = Box(ear_length, servo_width, ear_thickness)
        ear_geom -= Pos(ear_length / 2 - ear_hole_offset, 0, 0) * Cylinder(
            radius=ear_hole_dia / 2, height=ear_thickness + 1
        )



        # --- Assembly and Finishing ---
        self.label = kwargs.get("label", "SG9 Servo")
        super().__init__(** kwargs)
        self.children=[
                self.body,
                self.final_shaft,
                self.spline,
                self.final_gear_cover,
                self.penultimate_gear_cover,
            ]

        # Position ears relative to the bottom of the body and keep track of where they
        #  are for clients
        z_pos: float = -servo_height / 2 + ear_height_pos
        if left_mount:
            self.left_mount = (
                Pos(servo_length / 2 + ear_length / 2, 0, z_pos) * ear_geom
            )
            self.left_mount.label = "SG9 Servo Left Mount"
            self.children += tuple(self.left_mount)
        else:
            self.left_mount = None

        if right_mount:
            self.right_mount = (
                Pos(-(servo_length / 2 + ear_length / 2), 0, z_pos) * ear_geom
            )
            self.right_mount.label = "SG9 Servo Right Mount"
            self.children += tuple(self.right_mount)
        else:
            self.right_mount = None

        self.top_of_gear_cover_face: Face = (
            final_gear_cover.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
        )
        print(self.children)
        self.final_shaft: Shape = final_shaft
        self.final_shaft.top_face = shaft_top_face
        self.final_shaft.radius = shaft_diameter / 2 + spline_depth

    def mounts(self):
        children = self.children
        left_mount = children.filter_left_mount if "left_mount" in children else None
        right_mount = children.filter_right_mount if "right_mount" in children else None

        return {"right_mount": right_mount, "left_mount": left_mount}


def _tapered_bar(
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
        with Locations(cL):
            Circle(r_start, mode=Mode.ADD)
        with Locations(cR):
            Circle(r_end, mode=Mode.ADD)

        # Add polygon between the ends to create the taper
        with Locations((length / 2, 0)):
            Polygon([pL_top, pR_top, pR_bot, pL_bot], mode=Mode.ADD)

    return extrude(s.sketch, thickness)


class SG9ServoHorn(Part):
    """
    A simple servo horn for the SG9 servo

    """

    def __init__(
        self,
        servo: SG9Servo_Assembly,
        horn_diameter=None,
        horn_length=20,
        horn_thickness=4,
        screw_hole_dia=2,
        shaft_diameter=5,
        screw_hole_height=0.75,
        **kwargs,
    ) -> None:
        """
        Docstring for __init__

        :param self: Description
        :param horn_diameter: Socket outer diameter
        :param horn_thickness: Top-to-bottom thickness of the horn extension from the
          socket
        :param screw_hole_dia: Description
        :param shaft_diameter: the 'diameter' of the servo splined shaft
        :param kwargs: Additional keyword arguments passed to the Part constructor
        """

        if horn_diameter is None:
            horn_diameter = (1.0 + servo.final_shaft.radius) * 2.0

        horn_bar_length = horn_length - horn_diameter / 2
        bar = _tapered_bar(
            length=horn_bar_length,
            width_start=horn_diameter,
            width_end=horn_diameter / 2,
            thickness=1.5,
        )
        # bar = bar.move(Location((0, 0, horn_thickness / 2)))

        align_center_top = (Align.CENTER, Align.CENTER, Align.MAX)
        align_center_bottom = (Align.CENTER, Align.CENTER, Align.MIN)

        # the top of the splined shaft hole is the natural reference for the horn
        splined_shaft_hole = Cylinder(
            radius=shaft_diameter / 2, height=horn_thickness, align=align_center_top
        )
        shaft_socket = Cylinder(
            radius=horn_diameter / 2, height=horn_thickness, align=align_center_top
        ).move(Location((0, 0, screw_hole_height)))
        screw_hole = Cylinder(
            radius=screw_hole_dia / 2,
            height=horn_thickness + 1,
            align=align_center_bottom,
        )

        horn = shaft_socket
        horn += bar
        horn -= splined_shaft_hole
        horn -= Pos(0, 0, -horn_thickness / 2) * screw_hole

        self.length = horn_length
        self.socket_diameter = horn_diameter
        self.original_servo = weakref.ref(servo)

        super().__init__(horn.wrapped, **kwargs)


if __name__ == "__main__":
    from ocp_vscode import show

    servo = SG9Servo_Assembly(label="SG9 Servo")
    horn = SG9ServoHorn(servo, label="SG9 Servo Horn")
    horn.move(Location((0, 30, 0)))
    show(servo, horn)
