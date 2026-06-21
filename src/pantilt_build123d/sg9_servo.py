from build123d import *

class SG9Servo(Part):
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
        cable_pitch=1.27,
        cable_conductor_dia=1.2,
        cable_stub_length=10.0,
        color=Color("lightgray"),
        left_mount=True,
        right_mount=True,
        **kwargs
    ):
        """

        Doctests:

        >>> servo = SG9Servo()
        >>> servo.width
        12
        >>> lefty = SG9Servo(left_mount=True, right_mount=False)
        >>> lefty.left_mount is not None
        True
        >>> lefty.right_mount is None
        True
        >>> righty = SG9Servo(left_mount=False, right_mount=True)
        >>> righty.left_mount is None
        True
        >>> righty.right_mount is not None
        True
        >>> both = SG9Servo(left_mount=True, right_mount=True)
        >>> both.left_mount is not None and both.right_mount is not None
        True
        """
        self.width = servo_width
        self.body_height = servo_height # do not interfere with Part.height
        self.length = servo_length
        self.gear_cover_height = cover_height
        self.gear_cover_clearance_radius = 3 * servo_width / 4

        # --- Body ---
        self.body = Box(servo_length, servo_width, servo_height)

        # --- Shaft and Splines ---
        shaft_center_x = servo_length / 2 - cover_length / 2
        shaft_base = Cylinder(radius=shaft_diameter / 2, height=shaft_height)
        shaft_base = Pos(shaft_center_x, 0, servo_height / 2 + shaft_height / 2) * shaft_base


        # --- Use the Top Face of the Shaft as a Reference ---
        # Select the face with the highest Z coordinate
        shaft_top_face = shaft_base.faces().sort_by(Axis.Z)[-1]
        shaft_top_plane = Plane(shaft_top_face)

        # Define a helpful refference point for mounting horns
        self.horn_mount = Location((shaft_center_x, 0, servo_height / 2 + shaft_height))

        # Create the hole relative to this plane (Z=0 on the plane is the face surface)
        screw_hole = shaft_top_plane * Cylinder(radius=1, height=shaft_height)

        teeth_list = []
        for i in range(spline_teeth):
            angle = i * (360 / spline_teeth)
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
        final_gear_cover_radius = servo_width / 2
        final_gear_cover = Cylinder(radius=final_gear_cover_radius, height=cover_height)
        self.final_gear_cover = Pos(shaft_center_x, 0, servo_height / 2 + cover_height / 2) * final_gear_cover


        pen_gear_cover_radius = servo_width / 4
        penultimate_gear_cover = Cylinder(radius=pen_gear_cover_radius, height=cover_height)
        self.penultimate_gear_cover = Pos(shaft_center_x - servo_width / 2, 0, servo_height / 2 + cover_height / 2) * penultimate_gear_cover

        # --- Side Ears ---
        ear_length = 2 * ear_hole_offset
        ear_geom = Box(ear_length, servo_width, ear_thickness)
        ear_geom -= Pos(ear_length / 2 - ear_hole_offset, 0, 0) * Cylinder(radius=ear_hole_dia / 2, height=ear_thickness + 1)

        # Position ears relative to the bottom of the body and keep track of where they are for clients
        z_pos = -servo_height / 2 + ear_height_pos
        if left_mount:
            self.left_mount = Pos(servo_length / 2 + ear_length / 2, 0, z_pos) * ear_geom
        else:
            self.left_mount = None

        if right_mount:
            self.right_mount = Pos(-(servo_length / 2 + ear_length / 2), 0, z_pos) * ear_geom
        else:
            self.right_mount = None

        self.top_of_gear_cover_face = final_gear_cover.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]

        # --- Cable stub: 3-conductor ribbon, 0.05" (1.27 mm) pitch ---
        # Exits the shaft-end (+X) face, 2 mm up from the base bottom, centred
        # across the body width. A short stub for visualisation; each conductor
        # embeds 1 mm into the body so the union is clean.
        cable_embed = 1.0
        cable_z = -servo_height / 2 + 2.0
        cable_face_x = servo_length / 2
        cable_conductor_len = cable_stub_length + cable_embed
        cable_center_x = cable_face_x + (cable_stub_length - cable_embed) / 2
        cable_conductors = []
        for offset in (-cable_pitch, 0.0, cable_pitch):
            conductor = Cylinder(radius=cable_conductor_dia / 2, height=cable_conductor_len)
            conductor = conductor.rotate(Axis.Y, 90)          # lay the stub along +X
            conductor = Pos(cable_center_x, offset, cable_z) * conductor
            cable_conductors.append(conductor)
        self.cable = Compound(children=cable_conductors)

        # --- Assembly and Finishing ---
        servo_shape = self.body + shaft_base + spline + self.final_gear_cover + self.penultimate_gear_cover + self.left_mount + self.right_mount
        servo_shape -= screw_hole
        for conductor in cable_conductors:
            servo_shape += conductor

        # Initialize the Part with the constructed shape
        super().__init__(servo_shape.wrapped, **kwargs)
        self.color = color
        self.gear_boxes_radius = final_gear_cover_radius + pen_gear_cover_radius

    def mounts(self):
        """
        Docstring for mounts

        :param self: Description

        >>> servo = SG9Servo(left_mount=True, right_mount=False)
        >>> mounts = servo.mounts()
        >>> mounts["left_mount"] is not None
        True
        >>> mounts["right_mount"] is None
        True
        >>> servo2 = SG9Servo(left_mount=False, right_mount=True)
        >>> mounts2 = servo2.mounts()
        >>> mounts2["left_mount"] is None
        True
        >>> mounts2["right_mount"] is not None
        True
        >>> servo3 = SG9Servo(left_mount=True, right_mount=True)
        >>> mounts3 = servo3.mounts()
        >>> mounts3["left_mount"] is not None and mounts3["right_mount"] is not None
        True

        """

        return {
            "right_mount": self.right_mount,
            "left_mount": self.left_mount
        }

if __name__ == "__main__":
    import doctest
    doctest.testmod(verbose=True)

    from ocp_vscode import show
    model = SG9Servo()
    show(model)
