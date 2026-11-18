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
from navigation.settings import METHOD_LABELS, YIELD_BASES

PAPER = "#fbfaf7"
FLOOR = "#fffdf8"
INK = "#383c38"
MUTED = "#5f645d"
WALL = "#b7b1a4"
WALL_EDGE = "#928b7d"
CART = "#bf592b"
BASE = "#505e71"
YIELD = "#177869"
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


def legend_handles(*, paired=False, cart=False):
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
    if paired:
        handles.extend([
            Line2D([], [], color=BASE, ls=(0, (4, 2)), lw=1.7, label=caption("原方法", "Original")),
            Line2D([], [], color=YIELD, lw=2.6, label=caption("提前让行", "Early yield")),
            Line2D([], [], color=YIELD, marker="D", linestyle="none", mfc=FLOOR, ms=6,
                   label=caption("开始让行", "Yield starts")),
        ])
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


def paired_outcomes(scenes, variants, archive, seeds):
    rows = []
    for scene in scenes:
        for variant, base in variants.items():
            trials = [(archive[f"{scene.name}_{base}_seed{seed}.json"],
                       archive[f"{scene.name}_{variant}_seed{seed}.json"]) for seed in seeds]
            if any(a["scene_sha256"] != b["scene_sha256"] for a, b in trials):
                raise ValueError(f"Different maps or cart schedules within a pair on {scene.name}.")
            differences = [b["finish_time"] - a["finish_time"] for a, b in trials
                           if a["status"] == b["status"] == "success"]
            rows.append({
                "label": f"{scene_label(scene)}    {METHOD_LABELS[base]}",
                "before": sum(a["status"] == "success" for a, _ in trials),
                "after": sum(b["status"] == "success" for _, b in trials),
                "matches": len(differences), "delta": np.mean(differences) if differences else None,
                "yield_time": np.mean([b["yield_time_s"] for _, b in trials]),
            })
    return rows


def benchmark_figure(scenes, variants, archive, seeds, output):
    rows = paired_outcomes(scenes, variants, archive, seeds)
    fig, axes = plt.subplots(1, 3, figsize=(13.3, max(4.8, len(rows) * .49 + 2.4)),
                             gridspec_kw={"width_ratios": [1.05, 1.55, .9]})
    fig.subplots_adjust(left=.16, right=.97, bottom=.21, top=.73, wspace=.30)
    fig.suptitle(caption("提前让行前后的到达情况", "Navigation with and without early yielding"),
                 fontsize=20, fontweight="bold", y=.965)
    handles = [
        Line2D([], [], marker="s", mfc=FLOOR, mec=BASE, mew=1.5, ls="none",
               label=caption("原方法", "Original")),
        Line2D([], [], marker="o", color=YIELD, ls="none", label=caption("提前让行", "Early yield")),
    ]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, .90), ncol=2, frameon=False)
    positions = [i + .40 * (i // len(variants)) for i in range(len(rows))]
    arrival, delay, waiting = axes
    limits = [abs(r["delta"]) for r in rows if r["delta"] is not None]
    delta_bound = max([1, *limits])
    for ax in axes:
        ax.set_facecolor(PAPER)
        ax.spines[:].set_visible(False)
        ax.set_ylim(positions[-1] + .55, -.55)
        ax.set_yticks([])
        ax.tick_params(axis="x", length=0, labelsize=9)
        ax.set_axisbelow(True)
        ax.grid(axis="x", linewidth=.6, color="#dedbd3")
        for row_end in range(len(variants) - 1, len(rows) - 1, len(variants)):
            ax.axhline((positions[row_end] + positions[row_end + 1]) / 2, color="#ddd9cf", lw=.65)
    arrival.set_title(caption("到达次数", "Arrivals"), fontsize=13, pad=10)
    arrival.set_yticks(positions, [r["label"] for r in rows])
    arrival.tick_params(axis="y", length=0, pad=11, labelsize=10)
    arrival.set_xlim(-.35, len(seeds) + .35)
    arrival.xaxis.set_major_locator(MaxNLocator(nbins=6, integer=True))
    delay.set_title(caption("到达用时差（秒）", "Paired change in time (s)"), fontsize=13, pad=10)
    delay.set_xlim(-delta_bound * 1.45, delta_bound * 1.45)
    delay.xaxis.set_major_locator(MaxNLocator(5))
    delay.axvline(0, color="#8b8a80", lw=.9)
    waiting.set_title(caption("平均让行时间（秒）", "Mean time yielding (s)"), fontsize=13, pad=10)
    waiting.set_xlim(0, max(1, max(r["yield_time"] for r in rows)) * 1.25)
    waiting.xaxis.set_major_locator(MaxNLocator(4))
    for y, r in zip(positions, rows):
        arrival.plot([r["before"], r["after"]], [y - .11, y + .11], color="#9eaaa6", lw=1.4)
        arrival.plot(r["before"], y - .11, "s", mfc=FLOOR, mec=BASE, ms=6, mew=1.4)
        arrival.plot(r["after"], y + .11, "o", color=YIELD, ms=6)
        if r["delta"] is None:
            delay.text(0, y, "— (0)", ha="center", va="center", color=MUTED,
                       bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1})
        else:
            value = r["delta"]
            delay.plot([0, value], [y, y], color="#8cb5ac", lw=2)
            delay.plot(value, y, marker="o", ms=4.5, color=YIELD)
            align = "right" if value < 0 else "left"
            digits = 2 if abs(value) < 1 else 1
            delay.annotate(f"{value:+.{digits}f} ({r['matches']})", (value, y),
                           xytext=(-5 if value < 0 else 5, 0), textcoords="offset points",
                           ha=align, va="center", fontsize=9)
        waiting.barh(y, r["yield_time"], height=.26, color="#80aaa3", zorder=2)
        digits = 2 if r["yield_time"] < 1 else 1
        waiting.annotate(f"{r['yield_time']:.{digits}f}", (r["yield_time"], y), xytext=(5, 0),
                         textcoords="offset points", va="center", fontsize=9)
    fig.text(.5, .067, caption(
        f"每种方法在每张图上试跑 {len(seeds)} 次。用时差为让行版减原方法，只比较双方都到达的种子，括号内为配对数。",
        f"{len(seeds)} trials per map and method. Time change is early yield minus original, "
        "averaged over seeds where both arrived (paired count in parentheses)."),
        ha="center", color=MUTED, fontsize=10)
    fig.text(.5, .025, caption("让行时间取让行版全部试跑的平均值。", "Mean yielding time includes all early-yield trials."),
             ha="center", color=MUTED, fontsize=10)
    fig.savefig(output, dpi=165)
    plt.close(fig)


def track(run, scene):
    samples = [[p["t"], p["x"], p["y"]] for p in run["trajectory"]]
    if samples[0][0] > 0:
        samples.insert(0, [0, *scene.start])
    return np.asarray(samples, dtype=float)


def trajectory_figure(scenes, variants, examples, output):
    fig, axes = plt.subplots(len(variants), len(scenes),
                             figsize=(4.1 * len(scenes), 2.95 * len(variants) + 1.5), squeeze=False)
    fig.subplots_adjust(left=.10, right=.98, bottom=.17, top=.85, wspace=.12, hspace=.50)
    fig.suptitle(caption("同一张图上的两条路线", "Paths on the same map")
                 + f"  ·  {caption('种子', 'Seed')} {scenes[0].seed}",
                 fontsize=19, fontweight="bold", y=.97)
    for row, (variant, base) in enumerate(variants.items()):
        for col, scene in enumerate(scenes):
            ax = axes[row, col]
            draw_map(ax, scene, markersize=5)
            if row == 0:
                ax.set_title(scene_label(scene), pad=8, fontsize=13)
            if col == 0:
                ax.text(-.14, .5, METHOD_LABELS[base], transform=ax.transAxes,
                        ha="right", va="center", fontsize=14, fontweight="bold")
            for method, color in ((variant, YIELD), (base, BASE)):
                run = examples[scene.name, method]
                points = track(run, scene)
                original = method == base
                ax.plot(points[:, 1], points[:, 2], color=color, ls=(0, (4, 2)) if original else "-",
                        lw=1.6 if original else 2.9, zorder=5 if original else 4)
                if run["status"] != "success":
                    ax.plot(*points[-1, 1:], marker="x", color=color, ms=7, mew=1.7, zorder=8)
                if not original:
                    starts = [event["start"] for event in run["yield_events"]]
                    ax.scatter(np.interp(starts, points[:, 0], points[:, 1]),
                               np.interp(starts, points[:, 0], points[:, 2]), color=FLOOR, edgecolors=YIELD,
                               marker="D", s=24, linewidths=1.2, zorder=7)
                label = caption("原方法", "Original") if original else caption("提前让行", "Early yield")
                record = f"{label}    {outcome(run)}  {run['finish_time']:.1f} s"
                if not original:
                    record += caption(f"  ·  让行 {run['yield_count']} 次", f"  ·  {run['yield_count']} yields")
                ax.text(.5, -.067 if original else -.17, record,
                        transform=ax.transAxes, ha="center", va="top", fontsize=9, color=color)
    handles = legend_handles(paired=True)
    if any(r["status"] != "success" for r in examples.values()):
        handles.append(Line2D([], [], marker="x", ls="none", color=BASE, label=caption("未到达", "Did not arrive")))
    bottom_legend(fig, handles, y=.016)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def replay_status(run, time):
    """Report the logged yielding interval, including braking at its start."""
    elapsed = min(time, run["finish_time"])
    active = time < run["finish_time"] and any(e["start"] <= time < e["end"] for e in run["yield_events"])
    if time >= run["finish_time"]:
        text = f"{outcome(run)}   {run['finish_time']:.1f} s"
    elif active:
        text = caption("正在让行", "Yielding")
    else:
        text = caption("运行中", "Navigating")
    if run["algorithm"] in YIELD_BASES:
        total = sum(max(0., min(elapsed, e["end"]) - e["start"]) for e in run["yield_events"])
        text += f"    ·    {caption('累计让行', 'Time yielding')}  {total:.1f} s"
    return text, active


def write_video(scenes, variants, examples, output):
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
            axes = fig.subplots(len(variants), 2, squeeze=False)
            fig.subplots_adjust(left=.08, right=.92, top=.83, bottom=.10, wspace=.08, hspace=.31)
            fig.text(.065, .942, map_title(scene), fontsize=23, fontweight="bold")
            fig.text(.065, .902, f"{caption('二维仿真', '2D simulation')}  ·  {caption('种子', 'Seed')} "
                     f"{scene.seed}  ·  {speed}×", fontsize=12, color=MUTED)
            clock = fig.text(.935, .933, "", ha="right", fontsize=19)
            panels = []
            for row, (variant, base) in enumerate(variants.items()):
                for col, method in enumerate((base, variant)):
                    ax = axes[row, col]
                    run = examples[scene.name, method]
                    color = BASE if method == base else YIELD
                    draw_map(ax, scene)
                    label = caption("原方法", "Original") if method == base else caption("提前让行", "Early yield")
                    ax.set_title(f"{METHOD_LABELS[base]}  ·  {label}", fontsize=15, color=color)
                    path, = ax.plot([], [], lw=2, color=color, zorder=5)
                    body = Circle(scene.start, run["config"]["radius"], facecolor=color, edgecolor=FLOOR,
                                  lw=1.1, zorder=7)
                    ax.add_patch(body)
                    carts = []
                    for actor in scene.moving:
                        cart = Circle(actor.position(0), actor.radius, facecolor=CART, edgecolor=FLOOR,
                                      lw=1.1, zorder=6)
                        ax.add_patch(cart)
                        carts.append(cart)
                    panels.append((run, track(run, scene), path, body, carts, ax.set_xlabel("", fontsize=12)))
            bottom_legend(fig, legend_handles(cart=True), y=.018)
            end = max(run["finish_time"] for run, *_ in panels)
            for time in np.linspace(0, end, int(np.ceil(end * fps / speed)) + 1):
                clock.set_text(f"{time:.1f} s")
                for run, points, path, body, carts, status in panels:
                    frame_time = min(time, run["finish_time"])
                    stop = np.searchsorted(points[:, 0], frame_time, side="right")
                    xy = [np.interp(frame_time, points[:, 0], points[:, i]) for i in (1, 2)]
                    path.set_data(np.r_[points[:stop, 1], xy[0]], np.r_[points[:stop, 2], xy[1]])
                    body.center = xy
                    finished = time >= run["finish_time"]
                    for patch, actor in zip(carts, scene.moving):
                        patch.set_visible(not finished)
                        patch.center = actor.position(frame_time)
                    text, active = replay_status(run, time)
                    status.set_text(text)
                    status.set_color(YIELD if active else INK)
                    body.set_edgecolor(YIELD if active else FLOOR)
                    body.set_facecolor(FLOOR if active else path.get_color())
                    body.set_linewidth(2.8 if active else 1.1)
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
    variants = {variant: base for variant, base in YIELD_BASES.items()
                if {variant, base}.issubset(manifest["methods"])}
    if not variants:
        raise ValueError("Paired plots require a planner and its early-yield variant.")
    methods = [method for variant, base in variants.items() for method in (base, variant)]
    seed = 0 if 0 in manifest["seeds"] else manifest["seeds"][0]
    scenes = [make_scene(name, seed) for name in manifest["scenes"]]
    examples = {(scene.name, m): archive[f"{scene.name}_{m}_seed{seed}.json"]
                for scene in scenes for m in methods}
    for scene in scenes:
        if any(examples[scene.name, m]["scene_sha256"] != scene.signature() for m in methods):
            raise ValueError(f"Recorded paths do not match the current geometry of {scene.name}.")
    map_figure(scenes, args.results / "map.png")
    benchmark_figure(scenes, variants, archive, manifest["seeds"], args.results / "benchmark.png")
    trajectory_figure(scenes, variants, examples, args.results / "trajectories.png")
    if args.video:
        write_video(scenes, variants, examples, args.results / "course_maps.mp4")


if __name__ == "__main__":
    main()
