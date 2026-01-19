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
        color=Color("lightgray"),
        left_mount=True,
        right_mount=True,
        **kwargs
    ):
        self.width = servo_width
        self.body_height = servo_height # do not interfere with Part.height
        self.length = servo_length
        self.gear_cover_height = cover_height
        self.gear_cover_clearance_radius = 3 * servo_width / 4

        # --- Body ---
        body = Box(servo_length, servo_width, servo_height)
        self.body = body

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
        final_gear_cover = Cylinder(radius=servo_width / 2, height=cover_height)
        final_gear_cover = Pos(shaft_center_x, 0, servo_height / 2 + cover_height / 2) * final_gear_cover
        
        penultimate_gear_cover = Cylinder(radius=servo_width / 4, height=cover_height)
        penultimate_gear_cover = Pos(shaft_center_x - servo_width / 2, 0, servo_height / 2 + cover_height / 2) * penultimate_gear_cover

        # --- Side Ears ---
        ear_length = 2 * ear_hole_offset
        ear_geom = Box(ear_length, servo_width, ear_thickness)
        ear_geom -= Pos(ear_length / 2 - ear_hole_offset, 0, 0) * Cylinder(radius=ear_hole_dia / 2, height=ear_thickness + 1)
        
        # Position ears relative to the bottom of the body and keep track of where they are for clients
        z_pos = -servo_height / 2 + ear_height_pos
        if left_mount:
            left_ear = Pos(servo_length / 2 + ear_length / 2, 0, z_pos) * ear_geom
            mount = left_ear
        else:
            left_ear = Part()
            mount = None
        if right_mount:
            right_ear = Pos(-(servo_length / 2 + ear_length / 2), 0, z_pos) * ear_geom
            mount = right_ear
        else:
            right_ear = Part()
            # not setting mount here to avoid overwriting left ear if both are present
        if mount:
            mount_faces = mount.faces().filter_by(Axis.Z).sort_by(Axis.Z)
            self.top_of_mount_face = mount_faces[-1]
            self.bottom_of_mount_face = mount_faces[0]
        else:
            self.top_of_mount_face = None
            self.bottom_of_mount_face = None
        self.top_of_gear_cover_face = final_gear_cover.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]

        # --- Assembly and Finishing ---
        servo_shape = body + shaft_base + spline + final_gear_cover + penultimate_gear_cover + left_ear + right_ear
        servo_shape -= screw_hole

        # Initialize the Part with the constructed shape
        super().__init__(servo_shape.wrapped, **kwargs)
        self.color = color


if __name__ == "__main__":
    from ocp_vscode import show
    model = SG9Servo(left_mount=False, right_mount=False)
    show(model)
