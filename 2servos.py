from pantilt_build123d.sg9_servo import SG9Servo
from build123d import Location
from build123d.geometry import (
    Axis,
    Color
)
from ocp_vscode.config import Camera

from ocp_vscode import show

if __name__ == "__main__":
    servo1 = SG9Servo(color=Color("blue")) # pan servo
    top_of_shaft = servo1.faces().filter_by(Axis.Z, 1).sort_by(Axis.Z)[-1]
    
    servo2 = SG9Servo(color=Color("lightblue"), right_mount=False) # tilt servo
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo1.horn_mount * servo2 # Move up to tilting position
    servo2 = servo2.move(Location((servo2.width/2 +
                                    servo1.gear_cover_clearance_radius + 2,
                                        0,
                                        servo1.gear_cover_height +0.25))) # Move out to avoid collision
    
    faces = servo1.body.faces().filter_by(Axis.Z).sort_by(Axis.Z)
    print(f"Servo 1 faces Z: {faces}")

    show([servo1, servo2],
         reset_camera=Camera.KEEP)