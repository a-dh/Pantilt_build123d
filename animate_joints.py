from time import sleep

from build123d import Location, Rotation
from ocp_vscode import show
from ocp_vscode.config import Camera

from panTiltAssembly import build_assembly


def ping_pong(lo, hi, step):
    """One oscillation cycle lo -> hi -> lo as a list of angles.

    Steps by ``step`` and always lands on ``hi`` (a final short step is used
    if the range isn't an exact multiple of ``step``). The turnaround
    endpoints (lo and hi) are not duplicated, so the list can be looped
    seamlessly to repeat the cycle.
    """
    up = list(range(lo, hi + 1, step))
    if up[-1] != hi:
        up.append(hi)
    down = up[-2:0:-1]          # hi-step ... lo+step (exclude both endpoints)
    return up + down


def rotation_about(center, rx, ry, rz):
    """Location that rotates by the given Euler angles about ``center``."""
    return Location(center) * Rotation(rx, ry, rz) * Location(-center)


def animate():
    asm = build_assembly()
    parts = asm["parts"]
    pan_center = asm["pan_center"]
    tilt_center = asm["tilt_center"]
    panned = set(asm["panned"])
    tilted = set(asm["tilted"])

    names = list(parts.keys())

    # Both axes step by 30 deg and move together each frame. The pan axis
    # oscillates -90..+90 and the tilt axis -15..+90; the loop runs for two
    # full pan cycles, with tilt oscillating concurrently on its own period.
    pan_cycle = ping_pong(-90, 90, 30)
    tilt_cycle = ping_pong(-15, 90, 30)

    pan_cycles = 2
    total_frames = pan_cycles * len(pan_cycle)

    for frame in range(total_frames):
        pan_angle = pan_cycle[frame % len(pan_cycle)]
        tilt_angle = tilt_cycle[frame % len(tilt_cycle)]

        pan_rot = rotation_about(pan_center, 0, 0, pan_angle)    # about +Z
        tilt_rot = rotation_about(tilt_center, 0, tilt_angle, 0)  # about +Y

        posed = []
        for name, part in parts.items():
            if name in tilted:
                # Tilt first (in the base frame), then carry through the pan.
                located = pan_rot * tilt_rot * part
            elif name in panned:
                located = pan_rot * part
            else:
                located = part
            located.color = part.color
            posed.append(located)

        show(
            *posed,
            names=names,
            tab="joint_animation",
            reset_camera=Camera.KEEP,
        )
        sleep(0.2)


if __name__ == "__main__":
    animate()
