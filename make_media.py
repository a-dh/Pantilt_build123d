"""Derive web-ready preview assets (GIF + PNG) from the rendered MP4 clips.

Reads the clips produced by ``render_video.py`` from ``videos/`` and writes
small, README-friendly assets into ``docs/media/`` (committed to the repo, since
GitHub markdown will not inline-play MP4):

    docs/media/pantilt_opaque.gif       looping ~2.5 s opaque clip
    docs/media/pantilt_transparent.gif  looping ~2.5 s see-through clip
    docs/media/pantilt_hero.png         still hero frame

The full-quality MP4s are not committed; they are published as GitHub Release
assets instead.

Run:  python make_media.py        (after: python render_video.py)
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VIDEO_DIR = ROOT / "videos"
OUT_DIR = ROOT / "docs" / "media"

CLIPS = ("opaque", "transparent")

GIF_SECONDS = 2.5        # one pan cycle (the clip runs two cycles over 5 s)
GIF_WIDTH = 640          # px; downscaled to keep the GIF small
GIF_FPS = 15
HERO_TS = 0.6            # s; opaque-clip timestamp for the still hero frame


def _ffmpeg(*args):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], check=True)


def make_gif(src, dst):
    """Two-pass palette GIF: generate an optimal palette, then apply it."""
    vf = f"fps={GIF_FPS},scale={GIF_WIDTH}:-1:flags=lanczos"
    with tempfile.NamedTemporaryFile(suffix=".png") as pal:
        _ffmpeg("-t", str(GIF_SECONDS), "-i", str(src),
                "-vf", f"{vf},palettegen", pal.name)
        _ffmpeg("-t", str(GIF_SECONDS), "-i", str(src), "-i", pal.name,
                "-lavfi", f"{vf}[x];[x][1:v]paletteuse", str(dst))


def make_hero(src, dst):
    _ffmpeg("-ss", str(HERO_TS), "-i", str(src), "-frames:v", "1", str(dst))


def main():
    argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    ).parse_args()

    missing = [c for c in CLIPS if not (VIDEO_DIR / f"pantilt_{c}.mp4").exists()]
    if missing:
        names = ", ".join(f"pantilt_{c}.mp4" for c in missing)
        sys.exit(f"Missing {names} in {VIDEO_DIR} — run `python render_video.py` first.")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for clip in CLIPS:
        gif = OUT_DIR / f"pantilt_{clip}.gif"
        print(f"GIF  {gif} ...")
        make_gif(VIDEO_DIR / f"pantilt_{clip}.mp4", gif)

    hero = OUT_DIR / "pantilt_hero.png"
    print(f"PNG  {hero} ...")
    make_hero(VIDEO_DIR / "pantilt_opaque.mp4", hero)
    print("Done.")


if __name__ == "__main__":
    main()
