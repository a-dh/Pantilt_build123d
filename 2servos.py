from pantilt_build123d import build_servo
from build123d import Location
from build123d.geometry import (
    Axis,
    Color
)
from ocp_vscode.config import Camera

from ocp_vscode import show

if __name__ == "__main__":
    servo1 = build_servo(color=Color("blue")) # pan servo

    servo2 = build_servo(color=Color("lightblue")) # tilt servo
    servo2 = servo2.rotate(Axis.Z,90).rotate(Axis.X,90)  # Rotate for tilting
    servo2 = servo2.moved(Location((25, 0, 50))) # Move up to tilting position

    show([servo1, servo2],
         reset_camera=Camera.KEEP)