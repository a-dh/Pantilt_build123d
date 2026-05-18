from build123d import *
from pantilt_build123d.sg9_servo import SG9Servo


class UpperSwivelRing(Part):
    """
    Connects servo1's rotating horn to servo2's body so that pan rotation
    is transmitted to the full tilt assembly.

    Three fused sections:
      - Horn disc   : flat disc sitting on top of servo1's shaft
      - Bridge arm  : horizontal bar reaching servo2's -X face
      - Side bracket: vertical plate cradling servo2's body and mount ear

    All default dimensions match a default SG9Servo pair assembled with the
    standard X offset (servo2.width/2 + servo1.gear_cover_clearance_radius + 2)
    and no Z offset.
    """

    def __init__(
        self,
        servo1: SG9Servo,
        arm_thickness: float = 3.0,
        arm_width: float = 8.0,
        bracket_thickness: float = 3.0,
        color: Color = Color("orange"),
        **kwargs,
    ):
        # --- Reference geometry from servo1 ---
        horn_x = servo1.horn_mount.position.X   # shaft centre X in world (default 5.0)
        horn_z = servo1.horn_mount.position.Z   # shaft top Z in world (default 21.5)

        # servo2 centre X when placed with the standard X offset (Z offset must be 0)
        servo2_center_x = horn_x + servo1.width / 2 + servo1.gear_cover_clearance_radius + 2
        servo2_half_width = servo1.width / 2
        servo2_left_x = servo2_center_x - servo2_half_width  # -X face of servo2 body

        # servo2 body extents after rotation (servo1.length → Z, servo1.body_height → Y)
        servo2_body_z_half = servo1.length / 2
        servo2_body_y_half = servo1.body_height / 2

        # mount ear Z range in world (ear Z-local = [body_z_half, body_z_half + ear_length])
        ear_length = 8.0   # 2 × ear_hole_offset, matches SG9Servo default
        bracket_z_min = horn_z - servo2_body_z_half
        bracket_z_max = horn_z + servo2_body_z_half + ear_length

        # --- 1: Horn disc ---
        # Radius matches servo1 gear cover (width/2) so the disc sits cleanly above it
        disc_radius = servo1.width / 2
        disc = Cylinder(radius=disc_radius, height=arm_thickness)
        disc = Pos(horn_x, 0, horn_z + arm_thickness / 2) * disc

        # --- 2: Bridge arm ---
        arm_span = servo2_left_x - horn_x   # from disc centre to servo2 left face
        arm = Box(arm_span, arm_width, arm_thickness)
        arm = Pos(horn_x + arm_span / 2, 0, horn_z + arm_thickness / 2) * arm

        # --- 3: Side bracket ---
        bracket_height = bracket_z_max - bracket_z_min
        bracket_center_z = (bracket_z_min + bracket_z_max) / 2
        bracket = Box(bracket_thickness, 2 * servo2_body_y_half, bracket_height)
        bracket = Pos(servo2_left_x - bracket_thickness / 2, 0, bracket_center_z) * bracket

        # --- Horn centre screw hole (M2, through the disc in Z) ---
        screw_hole = Pos(horn_x, 0, horn_z - 0.5) * Cylinder(radius=1.0, height=arm_thickness + 1)

        # --- Assemble ---
        ring_shape = disc + arm + bracket
        ring_shape -= screw_hole

        # Public reference: the +X face of the bracket where servo2's -X face should rest
        self.servo2_attach_x = servo2_left_x

        super().__init__(ring_shape.wrapped, **kwargs)
        self.color = color
