"""Presentation figures and an obstacle-synchronized replay from the new manifest."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Circle, Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import make_scene

OUT = ROOT / "results" / "revised"
COLORS = {"fixed_dwa": "#7c3aed", "periodic_dwa": "#d97706", "replan_dwa": "#0284c7"}
NAMES = {"fixed_dwa": "Fixed A* + DWA", "periodic_dwa": "Periodic replanning", "replan_dwa": "Blocked-triggered replanning"}
plt.rcParams.update({"font.family": ["DejaVu Sans"], "font.size": 12,
                     "axes.spines.top": False, "axes.spines.right": False})


def draw(ax, scene):
    for o in scene.static:
        x, y = o.params["center"]
        if o.kind=="box":
            w, h = o.params["size"]
            patch = Rectangle((x-w/2, y-h/2), w, h)
        else:
            patch = Circle((x, y), o.params["radius"])
        patch.set(facecolor="#94a3b8", edgecolor="#475569")
        ax.add_patch(patch)
    ax.scatter(*scene.start, c="#16a34a", s=65, zorder=8)
    ax.scatter(*scene.goal, c="#dc2626", marker="*", s=140, zorder=8)
    ax.set(xlim=(-5, 5), ylim=(-5, 5), xlabel="X (m)", ylabel="Y (m)")
    ax.set_aspect("equal"); ax.grid(alpha=.12)


def figures(rows):
    scenes = list(dict.fromkeys(r["scene"] for r in rows))
    methods = list(dict.fromkeys(r["algorithm"] for r in rows))
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for j, method in enumerate(methods):
        values, successes = [], []
        for name in scenes:
            selected = [r for r in rows if r["scene"]==name and r["algorithm"]==method]
            successes.append(sum(r["status"]=="success" for r in selected)/len(selected)*100)
            # Failed trials receive the declared 65s budget; no faster-collision bias.
            values.append(np.mean([float(r["finish_time"]) if r["status"]=="success" else 65. for r in selected]))
        xx = np.arange(len(scenes))+(j-(len(methods)-1)/2)*.24
        axes[0].bar(xx, values, width=.23, color=COLORS[method], label=NAMES[method])
        axes[1].bar(xx, successes, width=.23, color=COLORS[method])
    labels = [s.replace("_", "\n") for s in scenes]
    for ax in axes:
        ax.set_xticks(range(len(scenes)), labels, fontsize=10)
        ax.grid(axis="y", alpha=.2); ax.set_axisbelow(True)
    axes[0].set(title="Time with failure penalty", ylabel="Seconds (failure = 65 s)", ylim=(0, 70))
    axes[1].set(title="Collision-free task success", ylabel="Success rate (%)", ylim=(0, 105))
    count = len({r["seed"] for r in rows})
    fig.suptitle(f"2D kinematic evaluation | {count} paired schedule phases | identical local controller", fontsize=16)
    fig.legend(loc="lower center", ncol=3, fontsize=10)
    fig.subplots_adjust(bottom=.20, top=.83, wspace=.25)
    fig.savefig(OUT / "benchmark.png", dpi=180)
    plt.close(fig)
    paths = [f"blocked_passage_{m}_seed0.json" for m in ("fixed_dwa", "replan_dwa")]
    if all((OUT / p).exists() for p in paths):
        runs = [json.loads((OUT / p).read_text(encoding="utf-8")) for p in paths]
    else:
        with gzip.open(OUT / "raw_runs.json.gz", "rt", encoding="utf-8") as f:
            archive = json.load(f)
        runs = [archive[p] for p in paths]
    scene = make_scene("blocked_passage")
    fig, axes = plt.subplots(1, 2, figsize=(13, 7.2))
    for ax, run in zip(axes, runs):
        draw(ax, scene)
        initial = np.array(run["initial_path"])
        ax.plot(initial[:, 0], initial[:, 1], "--", c="#64748b", lw=1.5, label="Initial static path")
        for h in run["path_history"]:
            path = np.array(h["path"])
            ax.plot(path[:, 0], path[:, 1], "--", c="#0284c7", label=f"Replan at {h['t']:.1f}s")
        tr = run["trajectory"]
        ax.plot([p["x"] for p in tr], [p["y"] for p in tr], c=COLORS[run["algorithm"]], lw=2.2, label="Executed trajectory")
        for o in scene.moving:
            ax.add_patch(Circle(o.position(15), o.radius, color="#ef4444", alpha=.7, label="Cart at t=15s"))
        ax.set_title(f"{NAMES[run['algorithm']]}\n{run['status']}: {run['finish_time']:.1f}s, replans={run['replans']}")
        ax.legend(loc="lower left", fontsize=9)
    fig.suptitle("Wait versus change route (2D simulation)\nRed cart: snapshot at t=15s; trajectories span the full run", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .91))
    fig.savefig(OUT / "blocked_passage.png", dpi=180, bbox_inches="tight")
    plt.close(fig)
    return runs


def video(runs):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg required for --video")
    plt.rcParams["animation.ffmpeg_path"] = ffmpeg
    scene = make_scene("blocked_passage")
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2))
    fig.subplots_adjust(top=.82, bottom=.15, left=.06, right=.96, wspace=.18)
    artists = []
    for ax, run in zip(axes, runs):
        draw(ax, scene)
        ax.set_title(NAMES[run["algorithm"]], fontsize=14)
        trail, = ax.plot([], [], color=COLORS[run["algorithm"]], lw=2)
        planned, = ax.plot([], [], "--", c="#0284c7", lw=1.1)
        body = Circle(scene.start, .18, color=COLORS[run["algorithm"]], zorder=9)
        cart = Circle(scene.moving[0].position(0), scene.moving[0].radius, color="#ef4444", zorder=8)
        ax.add_patch(body); ax.add_patch(cart)
        label = ax.text(.02, .97, "", transform=ax.transAxes, va="top", fontsize=10)
        artists.append((trail, planned, body, cart, label))
    clock = fig.suptitle("", fontsize=20)
    fig.text(.5, .045, "2D kinematic simulation | same cart schedule | observed scan points only | 3x playback", ha="center", fontsize=11)

    def update(t):
        clock.set_text(f"Blocked passage: t = {t:.1f} s")
        for run, (trail, planned, body, cart, label) in zip(runs, artists):
            tr = run["trajectory"]
            ts = np.array([p["t"] for p in tr])
            i = max(0, min(len(tr)-1, np.searchsorted(ts, t)-1))
            trail.set_data([p["x"] for p in tr[:i+1]], [p["y"] for p in tr[:i+1]])
            body.center = (float(np.interp(t, ts, [p["x"] for p in tr])), float(np.interp(t, ts, [p["y"] for p in tr])))
            cart.center = scene.moving[0].position(t)
            history = [h for h in run["path_history"] if h["t"]<=t]
            p = np.array(history[-1]["path"] if history else run["initial_path"])
            planned.set_data(p[:, 0], p[:, 1])
            label.set_text(f"Replans: {len(history)}"+(f" | {run['status']}" if t>=ts[-1] else ""))
        return []

    end = max(r["finish_time"] for r in runs)
    anim = FuncAnimation(fig, update, frames=np.arange(0, end+.25, .25), interval=1000/12)
    anim.save(OUT / "blocked_passage.mp4", writer=FFMpegWriter(fps=12, codec="libx264", bitrate=2000,
              extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]), dpi=100)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    with (OUT / "summary.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    runs = figures(rows)
    if args.video:
        video(runs)


if __name__ == "__main__":
    main()
