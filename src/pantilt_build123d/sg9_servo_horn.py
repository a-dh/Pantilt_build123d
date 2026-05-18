from build123d import *


class SG9ServoHorn(Part):
    """
    Single-arm SG9 servo horn.

    Origin at (0, 0, 0) = hub base centre, resting on the gear cover top face.
    Arm extends in +Y.  All bores are centred at X=Y=0.

    Hub bore profile (Z, bottom → top):
      0 .. shaft_engage_depth          spline bore (hub_inner_radius)
                                        fits tightly over the shaft splines
      shaft_engage_depth .. hub_height-cup_depth
                                        narrow screw-shaft bore (center_screw_radius)
                                        sits above the physical shaft tip
      hub_height-cup_depth .. hub_height
                                        cup counterbore (screw_head_radius)
                                        receives the M2 screw head

    Arm: flat strip in +Y; its TOP face is level with the cup top (= hub_height).
         Arm bottom = hub_height - arm_thickness.
    """

    def __init__(
        self,
        hub_outer_radius: float = 4.0,
        hub_inner_radius: float = 2.5,       # spline bore radius (fits shaft)
        shaft_engage_depth: float = 5.0,     # depth of spline bore = shaft above gear cover
        hub_height: float = 7.5,             # shaft_engage_depth + cup_depth
        arm_length: float = 17.0,
        arm_width: float = 4.0,
        arm_thickness: float = 2.5,          # also equals cup_depth for a flush top
        arm_hole_positions: tuple = (5.0, 10.0, 15.0),
        arm_hole_radius: float = 1.0,
        center_screw_radius: float = 1.0,    # M2 screw shaft bore
        cup_depth: float = 2.5,              # counterbore depth for screw head
        screw_head_radius: float = 2.0,      # counterbore radius for M2 hex head
        color: Color = Color("lightgray"),
        **kwargs,
    ):
        self.hub_outer_radius = hub_outer_radius
        self.hub_height = hub_height
        self.shaft_engage_depth = shaft_engage_depth
        self.arm_thickness = arm_thickness
        self.arm_length = arm_length
        self.arm_hole_positions = arm_hole_positions

        arm_bottom_z = hub_height - arm_thickness   # arm top flush with hub/cup top

        # --- Solid bodies ---
        hub = Cylinder(radius=hub_outer_radius, height=hub_height,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Arm top is flush with hub top; bottom at arm_bottom_z
        arm = Box(arm_width, arm_length, arm_thickness,
                  align=(Align.CENTER, Align.MIN, Align.MIN))
        arm = arm.move(Location((0, 0, arm_bottom_z)))

        # --- Bores ---
        # Spline bore: fits over shaft splines from base up to shaft_engage_depth
        spline_bore = Cylinder(radius=hub_inner_radius,
                               height=shaft_engage_depth + 0.1,
                               align=(Align.CENTER, Align.CENTER, Align.MIN))
        spline_bore = spline_bore.move(Location((0, 0, -0.05)))

        # Screw shaft bore: narrow channel through entire hub for M2 screw
        screw_bore = Cylinder(radius=center_screw_radius,
                              height=hub_height + 1,
                              align=(Align.CENTER, Align.CENTER, Align.MIN))
        screw_bore = screw_bore.move(Location((0, 0, -0.5)))

        # Cup counterbore: receives M2 screw head at hub top
        cup = Cylinder(radius=screw_head_radius,
                       height=cup_depth + 0.1,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        cup = cup.move(Location((0, 0, hub_height - cup_depth - 0.05)))

        # Arm mounting holes at given +Y positions
        arm_holes = []
        for pos in arm_hole_positions:
            hole = Cylinder(radius=arm_hole_radius,
                            height=arm_thickness + 1,
                            align=(Align.CENTER, Align.CENTER, Align.MIN))
            arm_holes.append(hole.move(Location((0, pos, arm_bottom_z - 0.5))))

        shape = hub + arm - spline_bore - screw_bore - cup
        for hole in arm_holes:
            shape -= hole

        super().__init__(shape.wrapped, **kwargs)
        self.color = color


if __name__ == "__main__":
    from ocp_vscode import show
    show(SG9ServoHorn())
