"""render_animation.py — turn frames CSVs into MP4 animations.

Reads outputs/csv/{scenario}_{method}_frames.csv, emits
outputs/videos/{scenario}_{method}.mp4 (one per method) and a grid
comparison outputs/videos/compare_{scenario}.mp4.

Requires: matplotlib, ffmpeg on PATH. If ffmpeg is missing, falls back to
animated GIF via Pillow.
"""
from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


HERE = Path(__file__).resolve().parent.parent
CSV_DIR = HERE / "outputs" / "csv"
VID_DIR = HERE / "outputs" / "videos"
METHODS = ["M0", "M1", "M2", "M3", "M4", "M4.5"]
SCENARIOS = ["box_drop"]


def _read_frames(path: Path):
    """Return dict[time] -> dict with 'box': (x,y,z) and 'nodes': list[(x,y,z,mass)]."""
    frames = defaultdict(lambda: {"box": None, "nodes": []})
    with path.open(newline="") as f:
        rdr = csv.reader(f); next(rdr)
        for row in rdr:
            t = float(row[0]); kind = row[1]
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            if kind == "B":
                frames[t]["box"] = (x, y, z)
            elif kind == "N":
                m = float(row[9])
                frames[t]["nodes"].append((x, y, z, m))
    return sorted(frames.items())


def _save_animation(fig, anim, mp4_path: Path, fps: int = 10):
    VID_DIR.mkdir(parents=True, exist_ok=True)
    try:
        writer = FFMpegWriter(fps=fps, bitrate=1200)
        anim.save(mp4_path, writer=writer)
        print(f"[anim] wrote {mp4_path}")
    except Exception as e:
        gif_path = mp4_path.with_suffix(".gif")
        print(f"[anim] ffmpeg unavailable ({e}); falling back to GIF: {gif_path}")
        anim.save(gif_path, writer=PillowWriter(fps=fps))


def render_one(scenario: str, method: str):
    path = CSV_DIR / f"{scenario}_{method}_frames.csv"
    if not path.exists():
        print(f"[anim] missing {path}")
        return
    frames = _read_frames(path)
    if not frames:
        print(f"[anim] empty {path}")
        return

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"{scenario} — {method}")

    def update(idx):
        ax.cla()
        t, fr = frames[idx]
        ax.set_title(f"{scenario} — {method}   t = {t:.2f}s")
        ax.set_xlim(-0.5, 0.5); ax.set_ylim(-0.5, 0.5); ax.set_zlim(0, 1.0)
        ax.set_xlabel("x"); ax.set_ylabel("z"); ax.set_zlabel("y")
        # Grid nodes — sand cloud
        if fr["nodes"]:
            xs = [n[0] for n in fr["nodes"]]
            zs = [n[2] for n in fr["nodes"]]
            ys = [n[1] for n in fr["nodes"]]
            ax.scatter(xs, zs, ys, s=8, c="tan", alpha=0.7)
        # Box centre
        if fr["box"]:
            bx, by, bz = fr["box"]
            ax.scatter([bx], [bz], [by], s=120, c="steelblue", marker="s")

    anim = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    out = VID_DIR / f"{scenario}_{method.replace('.', '_')}.mp4"
    _save_animation(fig, anim, out, fps=10)
    plt.close(fig)


def main(argv):
    for s in SCENARIOS:
        for m in METHODS:
            render_one(s, m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
