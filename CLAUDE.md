# Pan-Tilt Project Rules

## Design priorities (in order)

1. **Minimise centre of gravity** of all panned/tilted mass — every design decision should prefer lower CofG first.
2. **X/Y compactness** — minimise the footprint of the assembly in the horizontal plane.
3. All other constraints below apply within these priorities.

## Coding conventions

- **OCP viewer labels** — every `show()` call must pass a `names` list with one descriptive snake_case label per object. Pass objects as individual positional arguments (not a list) so the viewer maps names correctly. Labels must remain consistent across edits; update them if a shape's role changes.

## Mechanical design constraints

- **Tilt axis as low as possible** — minimise Z distance between the tilt servo's shaft and the top of servo1's shaft.
- **Horn screw accessibility** — the M2 screw holding the upper swivel ring on servo1's shaft must be reachable at some tilt angle without disassembling the tilt mechanism.
- **360° pan range** — the pan mechanism must allow full continuous rotation (for Waveshare serial bus servos); no wiring or geometry may block the sweep.
- **Gear cover swept volume must remain empty** — the cylindrical volume swept by servo1's gear covers during pan rotation (radius = `gear_cover_clearance_radius + 0.2` centred on servo1's shaft axis) must be kept clear of all parts of the upper swivelling assembly at every Z level where it exists. Apply the same clearance bore used in the upper bearing to any part that extends into this zone.
- **Pan bearing load distribution** — use wide-diameter swivel bearings to carry the Z-axis load and limit X/Y wobble, keeping radial forces off servo1's shaft.
- **Dual tilt axis support** — the camera/payload mount must be borne on both sides of the tilt servo: one pivot on the servo2 shaft side, and a counter-shaft pivot on the opposite side of the servo body. The counter-shaft must be collinear with the servo2 shaft (same Y-axis line) to prevent binding.
