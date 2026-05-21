from build123d import *

from build123d import Align, Cylinder, Face, Location, Vector, Wire, extrude


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
        self.arm_width = arm_width
        self.arm_length = arm_length
        self.arm_hole_positions = arm_hole_positions

        arm_bottom_z = hub_height - arm_thickness   # arm top flush with hub/cup top

        # --- Solid bodies ---
        hub = Cylinder(radius=hub_outer_radius, height=hub_height,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Arm: tapers from full hub diameter at base to arm_width at tip, +Y direction.
        # Built as an extruded trapezoid plus a circular cap at the tip.
        arm_face = Face(Wire.make_polygon([
            Vector(-hub_outer_radius, 0, 0),
            Vector( hub_outer_radius, 0, 0),
            Vector( arm_width / 2,    arm_length, 0),
            Vector(-arm_width / 2,    arm_length, 0),
        ]))
        arm_body = extrude(arm_face, arm_thickness).move(Location((0, 0, arm_bottom_z)))
        arm_cap  = Cylinder(radius=arm_width / 2, height=arm_thickness,
                            align=(Align.CENTER, Align.CENTER, Align.MIN))
        arm_cap  = arm_cap.move(Location((0, arm_length, arm_bottom_z)))
        arm = arm_body + arm_cap

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


class SG9ServoHornPocket(Part):
    """Clearance pocket matching an SG9ServoHorn.

    The pocket is constructed from an existing horn instance and is sized to
    be a clearance volume that is larger than the horn envelope by the
    requested radial clearance. It preserves the same orientation and is
    positioned so the horn hub and the pocket hub are concentric at the origin.

    Parameters:
        horn (SG9ServoHorn): The horn model to base the pocket on.
        clearance (float): Radial clearance around the horn's outer profile.
            Defaults to 0.2.
        extra_depth (float): Additional pocket depth in the Z direction. Positive
            values extend the pocket away from the servo (above the horn);
            negative values extend it toward the servo (below the horn base).
            Defaults to 0.2.
    """

    def __init__(self, horn: SG9ServoHorn, clearance: float = 0.2, extra_depth: float = 0.2, **kwargs):
        self.horn = horn
        self.clearance = clearance
        self.extra_depth = extra_depth

        hub_outer_radius = horn.hub_outer_radius
        arm_width = horn.arm_width
        arm_length = horn.arm_length
        arm_thickness = horn.arm_thickness
        hub_height = horn.hub_height

        arm_bottom_z = hub_height - arm_thickness
        extra_top = max(extra_depth, 0)
        extra_bottom = max(-extra_depth, 0)
        pocket_height = arm_thickness + extra_top + extra_bottom
        pocket_z = arm_bottom_z - extra_bottom

        arm_face = Face(Wire.make_polygon([
            Vector(-hub_outer_radius - clearance, 0, 0),
            Vector( hub_outer_radius + clearance, 0, 0),
            Vector( arm_width / 2 + clearance, arm_length, 0),
            Vector(-arm_width / 2 - clearance, arm_length, 0),
        ]))
        arm_body = extrude(arm_face, pocket_height).move(Location((0, 0, pocket_z)))
        arm_cap = Cylinder(radius=arm_width / 2 + clearance, height=pocket_height,
                            align=(Align.CENTER, Align.CENTER, Align.MIN))
        arm_cap = arm_cap.move(Location((0, arm_length, pocket_z)))

        hub = Cylinder(radius=hub_outer_radius + clearance,
                       height=hub_height + extra_top + extra_bottom,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        hub = hub.move(Location((0, 0, -extra_bottom)))

        pocket = hub + arm_body + arm_cap
        pocket = horn.location * pocket

        super().__init__(pocket.wrapped, **kwargs)


def make_horn_pocket(shaft_center_x, horn_hub_outer_radius, horn_arm_width,
                     horn_arm_thickness, horn_arm_length, arm_bottom_z,
                     pocket_clearance=0.2, gear_cover_top_z=None,
                     horn_hub_height=None):
    """Create a tapered horn arm pocket with a circular cap at the tip.

    The pocket is built as a trapezoidal extrusion from the horn's base at the
    servo shaft axis to the tip of the horn arm. A circular cap is added at the
    end of the extrusion to match the rounded profile of a standard SG90/SG9
    servo horn arm.

    Parameters:
        shaft_center_x (float): X coordinate of the servo shaft axis in world
            coordinates. This anchors the horn pocket relative to the pan servo.
        horn_hub_outer_radius (float): Outer radius of the horn hub. The pocket
            base is offset by this radius plus clearance.
        horn_arm_width (float): Full width of the horn arm at its tip. The pocket
            tip half-width is computed from half the arm width plus clearance.
        horn_arm_thickness (float): Thickness of the horn arm. The pocket is
            extruded by this amount plus a small extra allowance.
        horn_arm_length (float): Length of the horn arm from the shaft axis to the
            tip. This defines the Y extent of the trapezoidal pocket.
        arm_bottom_z (float): Z coordinate of the bottom face of the horn arm.
            The pocket is created at this vertical position.
        pocket_clearance (float, optional): Radial clearance added around the
            horn geometry to ensure the pocket is slightly larger than the horn.
            Defaults to 0.2.
        gear_cover_top_z (float, optional): If provided, enables an additional
            hub clearance cylinder above the gear cover top.
        horn_hub_height (float, optional): If provided with gear_cover_top_z,
            defines the hub height for the hub clearance cylinder.

    Returns:
        Solid: A Build123D solid representing the pocket volume, suitable for
        subtractive boolean operations against a horn mount or housing.
    """
    _pocket_clr = pocket_clearance
    _hw_base = horn_hub_outer_radius + _pocket_clr
    _hw_tip = horn_arm_width / 2 + _pocket_clr
    horn_top_to_pocket_top = 0.2  # small extra clearance to ensure the pocket fully contains the horn arm thickness at the top face
    _ph = horn_arm_thickness + horn_top_to_pocket_top
    horn_pocket = extrude(
        Face(Wire.make_polygon([
            Vector(shaft_center_x - _hw_base, -0.5,            arm_bottom_z),
            Vector(shaft_center_x + _hw_base, -0.5,            arm_bottom_z),
            Vector(shaft_center_x + _hw_tip,  horn_arm_length, arm_bottom_z),
            Vector(shaft_center_x - _hw_tip,  horn_arm_length, arm_bottom_z),
        ])),
        _ph,
    )
    horn_pocket_cap = Cylinder(radius=_hw_tip, height=_ph + horn_top_to_pocket_top,
                               align=(Align.CENTER, Align.CENTER, Align.MIN))
    horn_pocket_cap = horn_pocket_cap.move(Location((shaft_center_x, horn_arm_length, arm_bottom_z - 0.05)))
    horn_pocket =horn_pocket + horn_pocket_cap

    if gear_cover_top_z is not None and horn_hub_height is not None:
        hub_clearance = Cylinder(
            radius=horn_hub_outer_radius + 0.3,
            height=horn_hub_height + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).move(Location((shaft_center_x, 0, gear_cover_top_z - 0.05)))
        horn_pocket = horn_pocket + hub_clearance

    return horn_pocket
