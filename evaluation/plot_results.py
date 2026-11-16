"""Render the map, paired results, and a synchronized six-method replay."""
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
from navigation.settings import METHODS, METHOD_LABELS
from navigation.classical import voronoi_route

OUT = ROOT / "results"
COLORS = {"astar": "#7c3aed", "rrt_star": "#16a34a", "voronoi": "#0f766e",
          "apf": "#dc2626", "dwa": "#d97706", "voronoi_visibility": "#0284c7"}
CART_COLORS = ("#ef4444", "#f97316")
plt.rcParams.update({"font.family": ["DejaVu Sans"], "font.size": 11,
                     "axes.spines.top": False, "axes.spines.right": False})


def draw(ax, scene):
    for o in scene.static:
        x, y = o.params["center"]
        if o.kind == "box":
            w, h = o.params["size"]
            patch = Rectangle((x-w/2, y-h/2), w, h)
        else:
            patch = Circle((x, y), o.params["radius"])
        patch.set(facecolor="#94a3b8", edgecolor="#475569")
        ax.add_patch(patch)
    ax.scatter(*scene.start, c="#16a34a", s=65, zorder=8, label="Start")
    ax.scatter(*scene.goal, c="#dc2626", marker="*", s=140, zorder=8, label="Goal")
    ax.set(xlim=(-5, 5), ylim=(-5, 5), xlabel="X (m)", ylabel="Y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=.12)


def map_figure(scene, initial_path):
    fig, ax = plt.subplots(figsize=(9, 9))
    draw(ax, scene)
    p = np.asarray(initial_path)
    ax.plot(p[:, 0], p[:, 1], "--", color="#0284c7", lw=2, label="Initial static A* path")
    for i, o in enumerate(scene.moving):
        waypoints = np.asarray(o.waypoints)
        ax.plot(waypoints[:, 1], waypoints[:, 2], ":", c=CART_COLORS[i], lw=2,
                label=f"{o.name}: scripted track")
        ax.add_patch(Circle(o.position(30), o.radius, color=CART_COLORS[i], alpha=.8))
    ax.annotate("Temporary loading blockage\n(t = 12..58 s, seed 0)", (0, 0),
                (-3.9, -.3), arrowprops={"arrowstyle": "->"}, fontsize=10)
    ax.annotate("Alternative upper aisle", (.1, 2.7),
                (-.8, 4.4), arrowprops={"arrowstyle": "->"}, fontsize=10)
    ax.set_title("Warehouse navigation: one map, seven shelves, two moving carts\n"
                 "Cart footprints shown at t = 30 s; schedules are hidden from the planner", fontsize=13)
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "map.png", dpi=180)
    plt.close(fig)


def figures(rows, runs, scene):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
    for j, method in enumerate(METHODS):
        selected = [r for r in rows if r["algorithm"] == method]
        times = [float(r["finish_time"]) if r["status"] == "success" else scene.duration for r in selected]
        lengths = [float(r["actual_path_length"]) for r in selected if r["status"] == "success"]
        success = sum(r["status"] == "success" for r in selected)
        axes[0].bar(j, np.mean(times), yerr=np.std(times), color=COLORS[method], capsize=5)
        if lengths:
            axes[1].bar(j, np.mean(lengths), yerr=np.std(lengths), color=COLORS[method], capsize=5)
        else:
            axes[1].text(j, .2, "—", ha="center")
        axes[2].bar(j, success/len(selected)*100, color=COLORS[method])
        axes[2].text(j, success/len(selected)*100+2, f"{success}/{len(selected)}", ha="center")
    for ax in axes:
        ax.set_xticks(range(len(METHODS)), ["A*", "RRT*", "Voronoi", "APF", "DWA", "VV"], fontsize=9)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    axes[0].set(title="Arrival time (mean +/- SD)", ylabel="Seconds; failures = 100 s")
    axes[1].set(title="Travel length of successful trials", ylabel="Meters (mean +/- SD)")
    axes[2].set(title="Collision-free success", ylabel="Percent", ylim=(0, 112))
    fig.suptitle("Robot obstacle avoidance | five baselines + published Voronoi-Visibility | five seeds", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(OUT / "benchmark.png", dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(17, 13))
    for ax, run in zip(axes.flat, runs):
        draw(ax, scene)
        initial = np.asarray(run["initial_path"])
        reference = "Goal direction" if run["algorithm"] in ("apf", "dwa") else "Initial path"
        ax.plot(initial[:, 0], initial[:, 1], "--", c="#64748b", lw=1.2, label=reference)
        tr = run["trajectory"]
        ax.plot([p["x"] for p in tr], [p["y"] for p in tr], c=COLORS[run["algorithm"]], lw=2.3, label="Executed")
        for i, o in enumerate(scene.moving):
            ax.add_patch(Circle(o.position(30), o.radius, color=CART_COLORS[i], alpha=.7))
        ax.set_title(f"{METHOD_LABELS[run['algorithm']]}\n{run['status']}: {run['finish_time']:.1f}s")
        ax.legend(loc="lower right", fontsize=8)
    fig.suptitle("Seed 0 | 2D kinematic simulation\nCart snapshots at t=30s; trajectories cover the full run", fontsize=14)
    fig.subplots_adjust(left=.05, right=.99, bottom=.06, top=.90, wspace=.20, hspace=.40)
    fig.savefig(OUT / "warehouse.png", dpi=180)
    plt.close(fig)


def method_figure(runs, scene):
    chosen = {r["algorithm"]: r for r in runs}
    grid = scene.grid()
    raw = voronoi_route(grid, scene.start, scene.goal)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.8))
    for ax in axes:
        draw(ax, scene)
        ax.plot(raw[:, 0], raw[:, 1], ":", c="#94a3b8", lw=1, label="Voronoi backbone")
        for method in ("voronoi", "voronoi_visibility"):
            r = chosen[method]
            p = np.asarray(r["initial_path"])
            ax.plot(p[:, 0], p[:, 1], "o-", markersize=3, lw=2, c=COLORS[method],
                    label=f"{'Voronoi + greedy' if method=='voronoi' else 'VV'}: {r['planned_path_length']:.3f} m")
    axes[0].set_title("Complete planned routes | same 0.32m inflation")
    subset = raw[(raw[:, 0] < -2.0) & (raw[:, 1] > -2.1) & (raw[:, 1] < .5)][::4]
    for i, a in enumerate(subset):
        for b in subset[i+1:]:
            if grid.is_line_collision_free(a, b):
                axes[1].plot([a[0], b[0]], [a[1], b[1]], c="#64748b", alpha=.18, lw=.6, zorder=1)
    axes[1].set(xlim=(-4.65, -2.0), ylim=(-2.2, .5), title="First turn | a sample of valid visibility edges")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[1].legend(loc="upper right", fontsize=8)
    fig.suptitle("Voronoi-Visibility reproduction: add safe line-of-sight edges, then search their total length", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(OUT / "voronoi_vs_vv.png", dpi=180)
    plt.close(fig)


def video(runs, scene):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg required for --video")
    plt.rcParams["animation.ffmpeg_path"] = ffmpeg
    fig, axes = plt.subplots(2, 3, figsize=(19.2, 10.8))
    fig.subplots_adjust(top=.90, bottom=.09, left=.04, right=.98, wspace=.18, hspace=.32)
    artists = []
    for ax, run in zip(axes.flat, runs):
        draw(ax, scene)
        ax.set_title(METHOD_LABELS[run["algorithm"]], fontsize=14)
        trail, = ax.plot([], [], color=COLORS[run["algorithm"]], lw=2)
        planned, = ax.plot([], [], "--", c=COLORS[run["algorithm"]], lw=1.1)
        body = Circle(scene.start, .18, color=COLORS[run["algorithm"]], zorder=9)
        ax.add_patch(body)
        carts = []
        for i, o in enumerate(scene.moving):
            cart = Circle(o.position(0), o.radius, color=CART_COLORS[i], zorder=8)
            ax.add_patch(cart)
            carts.append(cart)
        label = ax.text(.02, .97, "", transform=ax.transAxes, va="top", fontsize=10)
        artists.append((trail, planned, body, carts, label))
    clock = fig.suptitle("", fontsize=20)
    fig.text(.5, .025, "2D kinematic simulation | five baselines + Voronoi-Visibility (Niu et al., 2019) | identical cart schedules | 3x playback", ha="center", fontsize=12)

    def update(t):
        clock.set_text(f"Warehouse: t = {t:.1f} s")
        for run, (trail, planned, body, carts, label) in zip(runs, artists):
            tr = run["trajectory"]
            ts = np.asarray([p["t"] for p in tr])
            i = max(0, min(len(tr)-1, np.searchsorted(ts, t)-1))
            trail.set_data([p["x"] for p in tr[:i+1]], [p["y"] for p in tr[:i+1]])
            body.center = (float(np.interp(t, ts, [p["x"] for p in tr])),
                           float(np.interp(t, ts, [p["y"] for p in tr])))
            for cart, o in zip(carts, scene.moving):
                cart.center = o.position(t)
            p = np.asarray(run["initial_path"])
            planned.set_data(p[:, 0], p[:, 1])
            label.set_text(run['status'] if t >= ts[-1] else "Navigating")
        return []

    end = max(r["finish_time"] for r in runs)
    anim = FuncAnimation(fig, update, frames=np.arange(0, end+.25, .25), interval=1000/12)
    anim.save(OUT / "warehouse.mp4", writer=FFMpegWriter(fps=12, codec="libx264", bitrate=2500,
              extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]), dpi=100)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    with (OUT / "summary.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    with gzip.open(OUT / "raw_runs.json.gz", "rt", encoding="utf-8") as f:
        archive = json.load(f)
    scene = make_scene()
    runs = [archive[f"{scene.name}_{m}_seed0.json"] for m in METHODS]
    map_figure(scene, runs[0]["initial_path"])
    figures(rows, runs, scene)
    method_figure(runs, scene)
    if args.video:
        video(runs, scene)


if __name__ == "__main__":
    main()
