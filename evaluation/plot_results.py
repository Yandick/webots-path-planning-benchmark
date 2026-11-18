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
from navigation.scenarios import make_scene, static_distance
from navigation.settings import METHODS, METHOD_LABELS, EXPERIMENT_GROUPS
from navigation.classical import voronoi_route
from navigation.metrics import resample_path
from navigation.smoothing import safe_bubbles

OUT = ROOT / "results"
COLORS = {"astar": "#7c3aed", "rrt_star": "#16a34a", "voronoi": "#0f766e",
          "apf": "#dc2626", "dwa": "#d97706", "voronoi_qp": "#0284c7"}
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
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    groups = list(EXPERIMENT_GROUPS.values())
    labels = [[METHOD_LABELS[method] for method in groups[0]],
              ["Heading follower", "APF", "DWA"]]
    titles = ["Global planners | same follower", "Local controllers | same A* guide"]
    for row, methods in enumerate(groups):
        for j, method in enumerate(methods):
            selected = [r for r in rows if r["algorithm"] == method]
            times = [float(r["finish_time"]) if r["status"] == "success" else scene.duration for r in selected]
            lengths = [float(r["actual_path_length"]) for r in selected if r["status"] == "success"]
            success = sum(r["status"] == "success" for r in selected)
            axes[row, 0].bar(j, np.mean(times), yerr=np.std(times), color=COLORS[method], capsize=5)
            if lengths:
                axes[row, 1].bar(j, np.mean(lengths), yerr=np.std(lengths), color=COLORS[method], capsize=5)
            else:
                axes[row, 1].text(j, .2, "—", ha="center")
            axes[row, 2].bar(j, success/len(selected)*100, color=COLORS[method])
            axes[row, 2].text(j, success/len(selected)*100+2, f"{success}/{len(selected)}", ha="center")
        for ax in axes[row]:
            ax.set_xticks(range(len(methods)), labels[row], fontsize=9)
            ax.grid(axis="y", alpha=.2)
            ax.set_axisbelow(True)
        axes[row, 0].set(title=titles[row]+"\nArrival time (mean +/- SD)", ylabel="Seconds; failures = 100 s")
        axes[row, 1].set(title="Travel length of successful trials", ylabel="Meters (mean +/- SD)")
        axes[row, 2].set(title="Collision-free success", ylabel="Percent", ylim=(0, 112))
    fig.suptitle("One warehouse | online obstacle observations and replanning | five paired seeds", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(OUT / "benchmark.png", dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(17, 13))
    for ax, run in zip(axes.flat, runs):
        draw(ax, scene)
        initial = np.asarray(run["initial_path"])
        reference = "Common A* reference" if run["algorithm"] in ("apf", "dwa") else "Initial path"
        ax.plot(initial[:, 0], initial[:, 1], "--", c="#64748b", lw=1.2, label=reference)
        for i, update in enumerate(run["route_updates"][1:]):
            p = np.asarray(update["path"])
            ax.plot(p[:, 0], p[:, 1], ":", c=COLORS[run["algorithm"]], lw=.9, alpha=.6,
                    label="Replanned reference" if i==0 else None)
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
    fig, axes = plt.subplots(1, 2, figsize=(13, 7.1))
    for ax in axes:
        draw(ax, scene)
        ax.plot(raw[:, 0], raw[:, 1], ":", c="#94a3b8", lw=1, label="Safe Voronoi reference")
        for method, label in (("voronoi", "Voronoi + greedy"),
                              ("voronoi_qp", METHOD_LABELS["voronoi_qp"])):
            p = np.asarray(chosen[method]["initial_path"])
            ax.plot(p[:, 0], p[:, 1], lw=2.2, c=COLORS[method], label=label)
    reference = resample_path(raw, .10)
    lower, upper = safe_bubbles(grid, reference)
    for k in range(0, len(reference), 4):
        axes[1].add_patch(Rectangle(lower[k], *(upper[k]-lower[k]),
                         facecolor="#a5b4fc", edgecolor="#6366f1", alpha=.18,
                         label="QP safe bubbles (sample)" if k==0 else None))
    for method in ("voronoi", "voronoi_qp"):
        tr = chosen[method]['trajectory']
        axes[1].plot([p['x'] for p in tr], [p['y'] for p in tr], '--',
                     lw=1.2, c=COLORS[method], alpha=.8)
        local = [p for p in tr if -4.85 < p['x'] < -2.7 and -2.7 < p['y'] < -.15]
        closest = min(local, key=lambda p:static_distance(np.array([p['x'],p['y']]),scene.static))
        axes[1].add_patch(Circle((closest['x'], closest['y']), .18,
                         facecolor=COLORS[method], edgecolor=COLORS[method], alpha=.25))
    axes[0].set_title("Planned routes | same 0.32m inflation")
    axes[1].set(xlim=(-4.85, -2.7), ylim=(-2.7, -.15),
                title="First turn | dashed: executed\nCircles: robot bodies at closest local shelf approach")
    for ax in axes:
        ax.legend(loc="lower right", fontsize=8)
    fig.suptitle(f"{METHOD_LABELS['voronoi_qp']} | Wen et al. (2024), Section IV-D", fontsize=13)
    fig.tight_layout(rect=(0, .02, 1, .92))
    fig.savefig(OUT / "voronoi_smoothing.png", dpi=180)
    plt.close(fig)


def safety_figure(rows):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.7))
    metrics = [("min_surface_clearance", "Minimum actual\nsurface clearance", "Meters; higher is safer"),
               ("planned_peak_curvature", "Peak initial planned curvature\n(0.10m samples)", "1/m; lower is smoother"),
               ("moving_angular_accel_rms", "Command angular acceleration RMS\nwhile moving", "rad/s²; lower is smoother")]
    for ax, (key, title, ylabel) in zip(axes, metrics):
        for i, method in enumerate(("voronoi", "voronoi_qp")):
            values = [float(r[key]) for r in rows if r['algorithm']==method]
            mean, sd = np.mean(values), np.std(values)
            ax.bar(i, mean, yerr=sd, capsize=5, color=COLORS[method], width=.6)
            ax.text(i, mean+sd+.03*mean, f"{mean:.3f}", ha="center", fontsize=11)
        ax.set_xticks([0, 1], [METHOD_LABELS[m] for m in ("voronoi", "voronoi_qp")])
        ax.set(title=title, ylabel=ylabel)
        ax.margins(y=.20)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle("Safety and smoothness | same map, sensors and follower | five paired seeds", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .90))
    fig.savefig(OUT / "safety_smoothness.png", dpi=180)
    plt.close(fig)


def failure_figure(archive):
    failures = [r for r in archive.values() if r['status']=='collision']
    output = OUT / "collision_case.png"
    if not failures:
        output.unlink(missing_ok=True)
        return
    run = failures[0]
    scene = make_scene(seed=run['seed'])
    tr = [p for p in run['trajectory'] if p['t'] >= run['finish_time']-4.]
    fig, ax = plt.subplots(figsize=(9, 7))
    draw(ax, scene)
    ax.plot([p['x'] for p in tr], [p['y'] for p in tr], lw=2.2,
            c=COLORS[run['algorithm']], label="Robot trajectory (last 4s)")
    for k, actor in enumerate(scene.moving):
        ax.plot([p['obstacles'][k][0] for p in tr], [p['obstacles'][k][1] for p in tr],
                ':', c=CART_COLORS[k], label=actor.name)
        end = tr[-1]['obstacles'][k]
        ax.add_patch(Circle(end[:2], end[2], facecolor=CART_COLORS[k], alpha=.35))
    end = tr[-1]
    ax.add_patch(Circle((end['x'], end['y']), .18, facecolor=COLORS[run['algorithm']], alpha=.4))
    ax.annotate(f"Robot at first contact\nt={end['t']:.2f}s", (end['x'], end['y']),
                (-1.6, .65), arrowprops={"arrowstyle":"->"}, fontsize=11)
    ax.set(xlim=(-1.8, 1.9), ylim=(-1.8, 1.3),
           title=f"Recorded collision | {METHOD_LABELS[run['algorithm']]} | seed {run['seed']}")
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
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
    fig.text(.5, .025, "Online global planning + common follower | APF/DWA: common A* reference | 3x playback", ha="center", fontsize=12)

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
            updates = [u for u in run["route_updates"] if u['t'] <= t]
            p = np.asarray(updates[-1]['path'])
            planned.set_data(p[:, 0], p[:, 1])
            label.set_text(f"{run['status'] if t >= ts[-1] else 'Navigating'} | replans {len(updates)-1}")
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
    safety_figure(rows)
    failure_figure(archive)
    if args.video:
        video(runs, scene)


if __name__ == "__main__":
    main()
