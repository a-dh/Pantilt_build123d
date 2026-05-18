# Pan-Tilt Project Rules

## Mechanical design constraints

- **Tilt axis as low as possible** — minimise Z distance between the tilt servo's shaft and the top of servo1's shaft.
- **Horn screw accessibility** — the M2 screw holding the upper swivel ring on servo1's shaft must be reachable at some tilt angle without disassembling the tilt mechanism.
- **360° pan range** — the pan mechanism must allow full continuous rotation (for Waveshare serial bus servos); no wiring or geometry may block the sweep.
- **Pan bearing load distribution** — use wide-diameter swivel bearings to carry the Z-axis load and limit X/Y wobble, keeping radial forces off servo1's shaft.
- **Dual tilt axis support** — the camera/payload mount must be borne on both sides of the tilt servo: one pivot on the servo2 shaft side, and a counter-shaft pivot on the opposite side of the servo body. The counter-shaft must be collinear with the servo2 shaft (same Y-axis line) to prevent binding.
