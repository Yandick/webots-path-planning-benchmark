"""Draw the public course maps, paired outcomes and synchronized recorded runs."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FFMpegWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle
from matplotlib.ticker import MaxNLocator
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import SCENE_INFO, make_scene
from navigation.settings import EXPERIMENT_GROUPS, METHOD_LABELS

PAPER = "#faf7f0"
FLOOR = "#fffdf8"
INK = "#383c38"
MUTED = "#5f645d"
WALL = "#a69e90"
WALL_EDGE = "#756e61"
CART = "#bf592b"
TRACK = "#207678"
START = "#487348"
GOAL = "#294f70"
available_fonts = {font.name for font in font_manager.fontManager.ttflist}
chinese_font = next((name for name in ("Noto Sans SC", "Noto Sans CJK SC", "Microsoft YaHei",
                                       "PingFang SC", "WenQuanYi Zen Hei") if name in available_fonts), None)
plt.rcParams.update({
    "font.family": chinese_font or "DejaVu Sans", "font.size": 10, "text.color": INK,
    "figure.facecolor": PAPER, "axes.facecolor": FLOOR, "axes.edgecolor": "#cec8bb",
    "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.unicode_minus": False, "savefig.facecolor": PAPER,
})


def caption(chinese, english):
    return chinese if chinese_font else english


def scene_label(scene):
    spec = SCENE_INFO[scene.name]
    return caption(spec["label"], spec["source_label"].split(" · ")[-1])


def map_title(scene):
    source = SCENE_INFO[scene.name]["source_label"].split(" · ")[-1]
    return f"{scene_label(scene)}  ·  {source}" if chinese_font else source


def method_label(method):
    if method == "apf":
        return "APF"
    if method == "dwa":
        return "DWA"
    return METHOD_LABELS[method]


def outcome(run):
    return {
        "success": caption("到达", "Arrived"), "collision": caption("碰撞", "Collision"),
        "timeout": caption("超时", "Timeout"),
        "planning_failed": caption("未找到路线", "No path"),
    }[run["status"]]


def draw_map(ax, scene, *, show_axes=False, markersize=7):
    """Draw the exact XY collision geometry, including the fixed cylinders."""
    for obstacle in scene.static:
        x, y = obstacle.params["center"]
        if obstacle.kind == "box":
            w, h = obstacle.params["size"]
            patch = Rectangle((x - w / 2, y - h / 2), w, h)
        else:
            patch = Circle((x, y), obstacle.params["radius"])
        patch.set(facecolor=WALL, edgecolor=WALL_EDGE, linewidth=.55, zorder=2)
        ax.add_patch(patch)
    for cart in scene.moving:
        route = np.asarray(cart.waypoints)
        ax.plot(route[:, 1], route[:, 2], color=CART, linestyle=(0, (2, 2)), lw=1.45, zorder=3)
    ax.plot(*scene.start, marker="o", ms=markersize, color=START, mec=FLOOR, mew=.9, zorder=6)
    ax.plot(*scene.goal, marker="*", ms=markersize * 1.7, color=GOAL, mec=FLOOR, mew=.75, zorder=6)
    ax.set_aspect("equal")
    ax.margins(.025)
    if show_axes:
        ax.xaxis.set_major_locator(MaxNLocator(4))
        ax.yaxis.set_major_locator(MaxNLocator(4))
        ax.tick_params(length=2.5, width=.55, labelsize=9)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)", labelpad=0)
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    # Freeze bounds before adding recorded paths or moving patches.
    ax.set(xlim=ax.get_xlim(), ylim=ax.get_ylim())


def legend_handles(*, robot=False, cart=False):
    handles = [
        Patch(facecolor=WALL, edgecolor=WALL_EDGE, label=caption("固定障碍", "Fixed obstacle")),
        Line2D([], [], color=CART, linestyle=(0, (2, 2)), lw=2,
               label=caption("小车路线", "Cart route")),
        Line2D([], [], marker="o", linestyle="none", color=START, label=caption("起点", "Start")),
        Line2D([], [], marker="*", ms=12, linestyle="none", color=GOAL, label=caption("终点", "Goal")),
    ]
    if cart:
        handles.append(Line2D([], [], marker="o", linestyle="none", color=CART,
                              label=caption("移动小车", "Moving cart")))
    if robot:
        handles.append(Line2D([], [], color=TRACK, lw=2.3,
                              label=caption("机器人实际路线", "Robot trajectory")))
    return handles


def bottom_legend(fig, handles, y=.025):
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, y),
               ncol=len(handles), frameon=False, fontsize=10,
               columnspacing=1.8, handlelength=1.7, handletextpad=.5)


def map_figure(scenes, output):
    fig, axes = plt.subplots(1, len(scenes), figsize=(4.65 * len(scenes), 5.1), squeeze=False)
    fig.subplots_adjust(left=.055, right=.98, bottom=.20, top=.80, wspace=.27)
    fig.suptitle(caption("实验地图", "Course maps"), fontsize=19, fontweight="bold", y=.96)
    for ax, scene in zip(axes[0], scenes):
        draw_map(ax, scene, show_axes=True, markersize=8)
        ax.set_title(map_title(scene), fontsize=12, pad=9)
        for cart in scene.moving:
            ax.add_patch(Circle(cart.position(0), cart.radius, facecolor=CART, edgecolor=FLOOR, lw=1.1,
                                zorder=4))
    handles = legend_handles()
    handles.append(Line2D([], [], marker="o", linestyle="none", color=CART,
                          label=caption("小车出发位置", "Cart at start")))
    bottom_legend(fig, handles)
    fig.savefig(output, dpi=165)
    plt.close(fig)


def benchmark_figure(scenes, runs, methods, output):
    groups = list(EXPERIMENT_GROUPS.values())
    groups = [[method for method in group if method in methods] for group in groups]
    titles = (caption("全局规划器", "Global planners"), caption("局部控制器", "Local controllers"))
    fig, axes = plt.subplots(2, len(scenes), figsize=(4.8 * len(scenes), 6.2), squeeze=False,
                             gridspec_kw={"height_ratios": [max(len(g), 1) for g in groups]})
    fig.subplots_adjust(left=.14, right=.95, bottom=.12, top=.82, wspace=.28, hspace=.62)
    fig.suptitle(caption("到达次数与用时", "Arrivals and travel time"), fontsize=19, fontweight="bold",
                 y=.985)
    fig.text(.5, .915, caption("横线表示成功到达的平均用时，右列列出到达次数 / 试跑次数。",
                               "Lines show mean time among arrivals. Counts show arrivals / trials."),
             ha="center", fontsize=11, color=MUTED)
    by_pair = {}
    for scene in scenes:
        for method in methods:
            attempts = [r for r in runs if r["scene"] == scene.name and r["algorithm"] == method]
            times = [r["finish_time"] for r in attempts if r["status"] == "success"]
            by_pair[scene.name, method] = (len(attempts), len(times), np.mean(times) if times else None)
    longest = max((t for _, _, t in by_pair.values() if t is not None), default=1)
    for row, group in enumerate(groups):
        for col, scene in enumerate(scenes):
            ax = axes[row, col]
            ax.set_xlim(0, longest * 1.26)
            ax.set_ylim(len(group) - .45, -.6)
            ax.xaxis.set_major_locator(MaxNLocator(4, integer=True))
            ax.tick_params(axis="x", length=0, labelsize=9)
            ax.tick_params(axis="y", length=0, pad=6)
            ax.set_yticks(range(len(group)), [method_label(m) for m in group] if col == 0 else [])
            ax.spines[:].set_visible(False)
            ax.grid(axis="x", linewidth=.55, color="#dedbcf", zorder=0)
            ax.set_facecolor(PAPER)
            if row == 0:
                ax.set_title(scene_label(scene), fontsize=12, pad=10)
            if col == 0:
                ax.text(0, 1.13, titles[row], transform=ax.transAxes, ha="left", color=MUTED,
                        fontsize=10, fontweight="bold")
            for y, method in enumerate(group):
                total, successes, avg = by_pair[scene.name, method]
                ax.text(1.025, y, f"{successes}/{total}", transform=ax.get_yaxis_transform(),
                        ha="left", va="center", fontsize=10, color=INK)
                if avg is None:
                    continue
                ax.plot([0, avg], [y, y], color="#89ada6", lw=2.5, solid_capstyle="round")
                ax.plot(avg, y, marker="o", ms=5, color=TRACK)
                ax.annotate(f"{avg:.1f}", (avg, y), (6, 0), textcoords="offset points",
                            fontsize=9, va="center")
            if row == len(groups) - 1:
                ax.set_xlabel(caption("秒", "Seconds"))
    fig.savefig(output, dpi=165)
    plt.close(fig)


def track(run, scene):
    samples = [[p["t"], p["x"], p["y"]] for p in run["trajectory"]]
    if samples[0][0] > 0:
        samples.insert(0, [0, *scene.start])
    return np.asarray(samples, dtype=float)


def trajectory_figure(scenes, methods, examples, output):
    fig, axes = plt.subplots(len(scenes), len(methods), figsize=(2.65 * len(methods), 2.7 * len(scenes) + 1.1),
                             squeeze=False)
    fig.subplots_adjust(left=.12, right=.985, bottom=.085, top=.895, wspace=.08, hspace=.42)
    fig.suptitle(caption("六种方法的实际路线", "Recorded trajectories")
                 + f"  ·  {caption('种子', 'Seed')} {scenes[0].seed}", fontsize=18, fontweight="bold", y=.982)
    for row, scene in enumerate(scenes):
        for col, method in enumerate(methods):
            ax = axes[row, col]
            run = examples[scene.name, method]
            draw_map(ax, scene, markersize=4.5)
            points = track(run, scene)
            ax.plot(points[:, 1], points[:, 2], color=TRACK, lw=1.75, zorder=4)
            if run["status"] != "success":
                ax.plot(*points[-1, 1:], marker="x", color=CART, ms=6.5, mew=1.6, zorder=8)
            if row == 0:
                ax.set_title(method_label(method), pad=7, fontsize=12)
            if col == 0:
                ax.text(-.08, .5, scene_label(scene), transform=ax.transAxes,
                        ha="right", va="center", fontsize=11)
            ax.set_xlabel(f"{outcome(run)}  {run['finish_time']:.1f} s", fontsize=9, labelpad=4)
    handles = legend_handles(robot=True)
    if any(run["status"] != "success" for run in examples.values()):
        handles.append(Line2D([], [], marker="x", linestyle="none", color=CART,
                              label=caption("未到达时的结束位置", "Final position on failure")))
    bottom_legend(fig, handles, y=.012)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def write_video(scenes, methods, examples, output):
    executable = shutil.which("ffmpeg")
    if not executable:
        raise RuntimeError("Recording the map replay requires ffmpeg on PATH.")
    plt.rcParams["animation.ffmpeg_path"] = executable
    fps, speed = 12, 3
    fig = plt.figure(figsize=(19.2, 10.8))
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=2400,
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    with writer.saving(fig, str(output), dpi=100):
        for scene in scenes:
            fig.clear()
            columns = min(len(methods), 3)
            axes = fig.subplots(int(np.ceil(len(methods) / columns)), columns, squeeze=False)
            fig.subplots_adjust(left=.045, right=.955, top=.875, bottom=.10, wspace=.075, hspace=.29)
            fig.text(.065, .942, map_title(scene), fontsize=23, fontweight="bold")
            fig.text(.065, .902, f"{caption('二维仿真', '2D simulation')}  ·  {caption('种子', 'Seed')} "
                     f"{scene.seed}  ·  {speed}×", fontsize=12, color=MUTED)
            clock = fig.text(.935, .935, "", ha="right", fontsize=19)
            panels = []
            for ax, method in zip(axes.flat, methods):
                run = examples[scene.name, method]
                draw_map(ax, scene)
                ax.set_title(method_label(method), fontsize=14)
                path, = ax.plot([], [], lw=2, color=TRACK, zorder=5)
                body = Circle(scene.start, run["config"]["radius"], facecolor=TRACK, edgecolor=FLOOR,
                              lw=1.15, zorder=7)
                ax.add_patch(body)
                carts = []
                for actor in scene.moving:
                    cart = Circle(actor.position(0), actor.radius, facecolor=CART, edgecolor=FLOOR,
                                  lw=1.15, zorder=6)
                    ax.add_patch(cart)
                    carts.append(cart)
                panels.append((run, track(run, scene), path, body, carts, ax.set_xlabel("", fontsize=11)))
            for ax in list(axes.flat)[len(methods):]:
                ax.set_visible(False)
            bottom_legend(fig, legend_handles(robot=True, cart=True), y=.02)
            end = max(run["finish_time"] for run, *_ in panels)
            for t in np.linspace(0, end, int(np.ceil(end * fps / speed)) + 1):
                clock.set_text(f"{t:05.1f} s")
                for run, points, path, body, carts, status in panels:
                    frame_time = min(t, run["finish_time"])
                    stop = np.searchsorted(points[:, 0], frame_time, side="right")
                    xy = [np.interp(frame_time, points[:, 0], points[:, i]) for i in (1, 2)]
                    path.set_data(np.r_[points[:stop, 1], xy[0]], np.r_[points[:stop, 2], xy[1]])
                    body.center = xy
                    for patch, actor in zip(carts, scene.moving):
                        patch.center = actor.position(frame_time)
                    if t >= run["finish_time"]:
                        status.set_text(f"{outcome(run)}   {run['finish_time']:.1f} s")
                    else:
                        status.set_text(caption("行驶中", "Navigating"))
                writer.grab_frame()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--results", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    with gzip.open(args.results / "raw_runs.json.gz", "rt", encoding="utf-8") as data:
        archive = json.load(data)
    manifest = json.loads((args.results / "manifest.json").read_text(encoding="utf-8"))
    if set(archive) != set(manifest["runs"]):
        raise ValueError("The raw archive and manifest list different runs.")
    methods = manifest["methods"]
    seed = 0 if 0 in manifest["seeds"] else manifest["seeds"][0]
    scenes = [make_scene(name, seed) for name in manifest["scenes"]]
    examples = {(scene.name, m): archive[f"{scene.name}_{m}_seed{seed}.json"]
                for scene in scenes for m in methods}
    for scene in scenes:
        if any(examples[scene.name, m]["scene_sha256"] != scene.signature() for m in methods):
            raise ValueError(f"Recorded paths do not match the current geometry of {scene.name}.")
    map_figure(scenes, args.results / "map.png")
    benchmark_figure(scenes, list(archive.values()), methods, args.results / "benchmark.png")
    trajectory_figure(scenes, methods, examples, args.results / "trajectories.png")
    if args.video:
        write_video(scenes, methods, examples, args.results / "course_maps.mp4")


if __name__ == "__main__":
    main()
