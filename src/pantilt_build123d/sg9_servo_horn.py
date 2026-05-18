from build123d import *


class SG9ServoHorn(Part):
    """
    Single-arm SG9 servo horn.

    Origin at (0, 0, 0) = base centre: the bottom face of the hub / arm,
    which rests on the gear cover top face when seated on the servo shaft.
    The arm extends in +Y.  The spline bore is centred at X=Y=0.

    Key attributes (all in mm):
      hub_outer_radius  – outer radius of the cylindrical boss over the spline
      hub_height        – height of hub above the arm face (= shaft height - gear-cover height)
      arm_thickness     – thickness of the arm (Z extent at base)
      arm_length        – full length of arm from shaft centre to tip
      arm_hole_positions – Y positions of mounting holes along the arm
    """

    def __init__(
        self,
        hub_outer_radius: float = 4.0,
        hub_inner_radius: float = 2.5,   # shaft spline bore radius
        hub_height: float = 5.0,
        arm_length: float = 17.0,
        arm_width: float = 4.0,
        arm_thickness: float = 2.5,
        arm_hole_positions: tuple = (5.0, 10.0, 15.0),
        arm_hole_radius: float = 1.0,
        center_screw_radius: float = 1.0,
        color: Color = Color("lightgray"),
        **kwargs,
    ):
        self.hub_outer_radius = hub_outer_radius
        self.hub_height = hub_height
        self.arm_thickness = arm_thickness
        self.arm_length = arm_length
        self.arm_hole_positions = arm_hole_positions

        # Hub: cylindrical boss from arm face up to shaft top
        hub = Cylinder(radius=hub_outer_radius, height=hub_height,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Arm: flat strip in +Y from shaft centre to tip
        arm = Box(arm_width, arm_length, arm_thickness,
                  align=(Align.CENTER, Align.MIN, Align.MIN))

        # Through-bore: spline fit bore from bottom, centre screw bore from top
        spline_bore = Cylinder(radius=hub_inner_radius, height=hub_height + 1,
                               align=(Align.CENTER, Align.CENTER, Align.MIN))
        spline_bore = spline_bore.move(Location((0, 0, -0.5)))

        center_screw_bore = Cylinder(radius=center_screw_radius, height=hub_height + 1,
                                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        center_screw_bore = center_screw_bore.move(Location((0, 0, -0.5)))

        # Arm mounting holes
        arm_holes = []
        for pos in arm_hole_positions:
            hole = Cylinder(radius=arm_hole_radius, height=arm_thickness + 1,
                            align=(Align.CENTER, Align.CENTER, Align.MIN))
            arm_holes.append(hole.move(Location((0, pos, -0.5))))

        shape = hub + arm - spline_bore - center_screw_bore
        for hole in arm_holes:
            shape -= hole

        super().__init__(shape.wrapped, **kwargs)
        self.color = color


if __name__ == "__main__":
    from ocp_vscode import show
    show(SG9ServoHorn())
