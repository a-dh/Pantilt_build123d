#!/usr/bin/env python
"""Export the printed pan-tilt parts for slicing on a Flashforge Creator Pro 2.

Each printed part is written once per format, in the model's *design
orientation* (assembly coordinates). No print orientation is baked in — lay
each part flat in your slicer per the print plan. Bought parts (servos, horns,
the steel counter-shaft rod) are skipped.

Formats, highest fidelity first — pick whichever your slicer reads:

  .step  True B-rep, lossless. OrcaSlicer / PrusaSlicer / Cura (+STEP plugin)
         import this and tessellate at their own quality. Best path for the
         FFCP2 via Orca/Prusa (which also have the IDEX support you want for
         upper_pan_bracket).
  .3mf   High-resolution mesh carrying explicit mm units. Read by FlashPrint 5
         (the native FFCP2 slicer) and by Orca/Prusa/Cura.
  .stl   Universal binary fallback, same mesh resolution as the .3mf.

Usage:
    python export_parts.py [output_dir]      # default: ./export
"""

from pathlib import Path
import sys

from build123d import Mesher, Unit, export_step, export_stl

from pantilt_build123d.pan_tilt_assembly import build_assembly

# Parts produced on the printer. Everything else in the assembly is bought.
PRINTED_PARTS = ("host_plate", "pan_static_bearing", "upper_pan_bracket", "tilt_yoke")

# Mesh fidelity for .3mf / .stl. Both are far finer than any FDM nozzle can
# resolve, so the Ø3 press-fit/pivot bores stay round after tessellation.
LINEAR_DEFLECTION = 0.01    # mm  — max chord deviation from the true surface
ANGULAR_DEFLECTION = 0.05   # rad — ~2.9° between facets on curved faces


def export_part(part, stem: Path):
    """Write `part` to STEP + 3MF + STL named `<stem>.<ext>`; return the paths."""
    written = []

    step_path = stem.with_suffix(".step")
    export_step(part, str(step_path), unit=Unit.MM)
    written.append(step_path)

    mesher = Mesher(unit=Unit.MM)
    mesher.add_shape(
        part,
        linear_deflection=LINEAR_DEFLECTION,
        angular_deflection=ANGULAR_DEFLECTION,
    )
    mf_path = stem.with_suffix(".3mf")
    mesher.write(str(mf_path))
    written.append(mf_path)

    stl_path = stem.with_suffix(".stl")
    export_stl(  # binary STL (ascii_format=False) for size + precision
        part,
        str(stl_path),
        tolerance=LINEAR_DEFLECTION,
        angular_tolerance=ANGULAR_DEFLECTION,
    )
    written.append(stl_path)

    return written


def main(out_dir="export"):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    parts = build_assembly()["parts"]
    missing = [n for n in PRINTED_PARTS if n not in parts]
    if missing:
        raise KeyError(f"build_assembly() is missing printed parts: {missing}")

    for name in PRINTED_PARTS:
        for path in export_part(parts[name], out / name):
            print(f"  wrote {path}  ({path.stat().st_size / 1024:.1f} KiB)")

    print(f"\n{len(PRINTED_PARTS)} parts exported to {out.resolve()}/")
    print(
        "Design orientation = assembly coordinates. In the slicer: use "
        "STEP for Orca/Prusa (highest fidelity) or 3MF for FlashPrint, then "
        "lay each part flat per the print plan."
    )


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "export")
