from time import sleep

from ocp_vscode import show
from ocp_vscode.config import Camera

from pantilt_build123d.pan_tilt_assembly import build_assembly


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


def animate():
    asm = build_assembly()
    parts = asm["parts"]
    pan_joint = asm["pan_joint"]
    tilt_joint = asm["tilt_joint"]
    pan_rigid_joints = asm["pan_rigid_joints"]
    tilt_rigid_joints = asm["tilt_rigid_joints"]

    names = list(parts.keys())
    show_parts = list(parts.values())

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

        # Pan first so servo2 (which carries the tilt joint) is repositioned,
        # then tilt places the yoke and tilt horn relative to the panned servo2.
        for rj in pan_rigid_joints:
            pan_joint.connect_to(rj, angle=pan_angle)
        for rj in tilt_rigid_joints:
            tilt_joint.connect_to(rj, angle=tilt_angle)

        show(
            *show_parts,
            names=names,
            render_joints=True,
            tab="joint_animation",
            reset_camera=Camera.KEEP,
        )
        sleep(0.2)


if __name__ == "__main__":
    animate()
