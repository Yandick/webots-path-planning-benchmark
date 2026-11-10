"""
实验结果可视化绘图模块 (Result Plotting & Visualization)
读取全流派规划算法实测数据，绘制 2D 轨迹全景对比图、沃罗诺伊拓扑中轴骨架网络、PSO 适应度进化收敛曲线、
多维度量化对比柱状图以及线速度/角速度时域响应图。
"""

import os
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 300

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")

def load_data():
    files = {
        'A* + DWA (Grid Search)': os.path.join(RESULTS_DIR, 'trajectory_a_star_dwa.json'),
        'Voronoi + DWA (GVD Skeleton)': os.path.join(RESULTS_DIR, 'trajectory_voronoi_dwa.json'),
        'PSO + DWA (Swarm Intelligence)': os.path.join(RESULTS_DIR, 'trajectory_pso_dwa.json'),
        'RRT* + DWA (Random Sampling)': os.path.join(RESULTS_DIR, 'trajectory_rrt_star_dwa.json'),
        'Informed Bi-RRT* (Proposed SOTA)': os.path.join(RESULTS_DIR, 'trajectory_informed_rrt_star_dwa.json'),
        'Classical APF (Khatib 1986)': os.path.join(RESULTS_DIR, 'trajectory_apf.json'),
        'Improved APF (Ge & Cui 2000)': os.path.join(RESULTS_DIR, 'trajectory_improved_apf.json'),
        'A* + MPC (Model Predictive)': os.path.join(RESULTS_DIR, 'trajectory_mpc.json'),
    }
    data = {}
    for name, path in files.items():
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                data[name] = json.load(f)
    return data

def plot_trajectories(data):
    """绘制 2D 环境地图、静态/动态障碍物、Voronoi骨架网络与各算法实际执行轨迹"""
    fig, ax = plt.subplots(figsize=(11, 11))

    ax.set_xlim(-5.2, 5.2)
    ax.set_ylim(-5.2, 5.2)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.45, color='#cccccc')
    ax.set_facecolor('#fbfbfb')

    # 1. 边界外墙
    wall_color = '#334155'
    ax.add_patch(patches.Rectangle((-5.1, 4.9), 10.2, 0.2, color=wall_color))
    ax.add_patch(patches.Rectangle((-5.1, -5.1), 10.2, 0.2, color=wall_color))
    ax.add_patch(patches.Rectangle((4.9, -5.1), 0.2, 10.2, color=wall_color))
    ax.add_patch(patches.Rectangle((-5.1, -5.1), 0.2, 10.2, color=wall_color))

    # 2. 静态障碍物
    obs_color = '#64748b'
    static_boxes = [
        ((-0.7, -0.7), 1.4, 1.4),
        ((-2.7, 0.3), 1.0, 2.4),
        ((1.0, -2.0), 2.4, 1.0),
        ((-1.3, 2.85), 2.2, 0.7),
    ]
    for (xy, w, h) in static_boxes:
        ax.add_patch(patches.Rectangle(xy, w, h, facecolor=obs_color, edgecolor='#1e293b', linewidth=1.2, zorder=3))

    static_cylinders = [
        ((-1.8, -1.8), 0.5),
        ((1.8, 1.8), 0.5),
        ((0.2, -3.2), 0.5),
    ]
    for (xy, r) in static_cylinders:
        ax.add_patch(patches.Circle(xy, r, facecolor=obs_color, edgecolor='#1e293b', linewidth=1.2, zorder=3))

    # 3. 动态障碍物扫掠区域
    ax.add_patch(patches.Rectangle((-2.85, -0.95), 3.7, 0.7, facecolor='#fee2e2', edgecolor='#ef4444',
                                   linestyle=':', alpha=0.45, linewidth=1.2, label='Dynamic Obs 1 Sweep (H)'))
    ax.add_patch(patches.Rectangle((0.25, -1.15), 0.7, 3.9, facecolor='#fef3c7', edgecolor='#f59e0b',
                                   linestyle=':', alpha=0.45, linewidth=1.2, label='Dynamic Obs 2 Sweep (V)'))
    ax.plot([-1.8, 0.4], [1.2, 2.4], color='#ec4899', linestyle=':', linewidth=2.0, label='Dynamic Obs 3 Patrol')

    # 4. 绘制 Voronoi 骨架网络底图 (如果 Voronoi 数据中包含)
    vor_data = data.get('Voronoi + DWA (GVD Skeleton)')
    if vor_data and 'global_plan_info' in vor_data:
        edges = vor_data['global_plan_info'].get('voronoi_edges', [])
        for i, (p1, p2) in enumerate(edges):
            lbl = "Voronoi Medial Axis Skeleton" if i == 0 else ""
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#cbd5e1', linestyle='-', linewidth=0.8, alpha=0.7, label=lbl, zorder=2)

    # 5. 起点与终点
    start = (-4.0, -4.0)
    goal = (4.0, 4.0)
    ax.plot(start[0], start[1], marker='o', markersize=11, color='#16a34a', label='Start Point (-4, -4)', zorder=6)
    ax.plot(goal[0], goal[1], marker='*', markersize=15, color='#dc2626', label='Goal Point (4, 4)', zorder=6)

    # 6. 绘制各算法实际行驶轨迹
    colors = {
        'A* + DWA (Grid Search)': '#1d4ed8',          # Blue
        'Voronoi + DWA (GVD Skeleton)': '#0d9488',    # Teal
        'PSO + DWA (Swarm Intelligence)': '#d97706',  # Amber
        'RRT* + DWA (Random Sampling)': '#7c3aed',    # Purple
        'Informed Bi-RRT* (Proposed SOTA)': '#0284c7',# Cyan/Sky Blue
        'Classical APF (Khatib 1986)': '#e11d48',     # Rose Red
        'Improved APF (Ge & Cui 2000)': '#ea580c',    # Orange
        'A* + MPC (Model Predictive)': '#059669'      # Emerald Green
    }
    styles = {
        'A* + DWA (Grid Search)': '-',
        'Voronoi + DWA (GVD Skeleton)': '-',
        'PSO + DWA (Swarm Intelligence)': '-.',
        'RRT* + DWA (Random Sampling)': '--',
        'Informed Bi-RRT* (Proposed SOTA)': '-',
        'Classical APF (Khatib 1986)': ':',
        'Improved APF (Ge & Cui 2000)': '--',
        'A* + MPC (Model Predictive)': '-'
    }

    for name, info in data.items():
        traj = info.get('trajectory', [])
        if not traj:
            continue
        xs = [p['x'] for p in traj]
        ys = [p['y'] for p in traj]

        label_txt = f"{name} ({info.get('actual_path_length', 0):.1f}m, {info.get('finish_time', 0):.1f}s)"
        if not info.get('success'):
            label_txt += " [Trapped]"
        ax.plot(xs, ys, color=colors[name], linestyle=styles[name], linewidth=2.4, label=label_txt, zorder=5)

        if not info.get('success'):
            ax.plot(xs[-1], ys[-1], marker='x', markersize=14, markeredgewidth=3, color='#e11d48', zorder=7)

    ax.set_title("Multi-Algorithm Path Planning & Dynamic Avoidance in Webots", fontsize=15, fontweight='bold', pad=15)
    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    ax.legend(loc='lower right', framealpha=0.92, fontsize=8.8)

    out_path = os.path.join(RESULTS_DIR, "trajectory_comparison.png")
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Generated: {out_path}")

def plot_pso_convergence(data):
    """绘制粒子群算法 (LDIW-PSO) 适应度迭代进化收敛曲线"""
    pso_data = data.get('PSO + DWA (Swarm Intelligence)')
    if not pso_data or 'global_plan_info' not in pso_data:
        return
    history = pso_data['global_plan_info'].get('fitness_history', [])
    if not history:
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(range(len(history)), history, color='#d97706', linewidth=2.2, label='Global Best Fitness $G_{best}$')
    ax.set_title("LDIW-PSO Fitness Evolutionary Convergence Curve", fontsize=13, fontweight='bold')
    ax.set_xlabel("Generation / Iteration $k$", fontsize=11)
    ax.set_ylabel("Multi-Objective Fitness Cost", fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10.5)

    # 标注初始与最终值
    ax.annotate(f"Initial: {history[0]:.1f}", xy=(0, history[0]), xytext=(5, history[0]*0.85),
                arrowprops=dict(arrowstyle="->", color='#d97706'), fontweight='bold', fontsize=9.5)
    ax.annotate(f"Optimal Converged: {history[-1]:.1f}", xy=(len(history)-1, history[-1]),
                xytext=(len(history)*0.6, history[-1] + 25),
                arrowprops=dict(arrowstyle="->", color='#059669'), fontweight='bold', fontsize=9.5)

    out_path = os.path.join(RESULTS_DIR, "pso_fitness_convergence.png")
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Generated: {out_path}")

def plot_metrics(data):
    """绘制量化对比指标柱状图"""
    algos = list(data.keys())
    plan_times = []
    travel_times = []
    path_lengths = []
    clearances = []

    for k in algos:
        d = data[k]
        plan_info = d.get('global_plan_info', {})
        plan_times.append(plan_info.get('planning_time_ms', 0.5))
        travel_times.append(d.get('finish_time', 80.0))
        path_lengths.append(d.get('actual_path_length', 0.0))
        cl = d.get('min_clearance', 0.3)
        if math.isinf(cl): cl = 0.25
        clearances.append(cl)

    name_map = {
        'A* + DWA (Grid Search)': 'A*',
        'Voronoi + DWA (GVD Skeleton)': 'Voronoi',
        'PSO + DWA (Swarm Intelligence)': 'PSO',
        'RRT* + DWA (Random Sampling)': 'RRT*',
        'Classical APF (Khatib 1986)': 'C-APF',
        'Improved APF (Ge & Cui 2000)': 'I-APF',
        'DRL Navigation (Deep RL)': 'DRL'
    }
    short_names = [name_map.get(k, k.split(' ')[0]) for k in algos]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Performance Quantitative Benchmarking across 7 Algorithms", fontsize=15, fontweight='bold')

    palette = ['#1d4ed8', '#0d9488', '#d97706', '#7c3aed', '#0284c7', '#e11d48', '#ea580c', '#059669'][:len(algos)]

    # 1. 规划耗时
    axes[0, 0].bar(short_names, plan_times, color=palette, width=0.55, edgecolor='#1e293b')
    axes[0, 0].set_title("Global Planning Time (ms)", fontweight='bold')
    axes[0, 0].set_ylabel("Milliseconds (ms)")
    for i, v in enumerate(plan_times):
        axes[0, 0].text(i, v + 8, f"{v:.1f}", ha='center', fontweight='bold', fontsize=9)

    # 2. 导航耗时
    axes[0, 1].bar(short_names, travel_times, color=palette, width=0.55, edgecolor='#1e293b')
    axes[0, 1].set_title("Navigation Travel Time (s)", fontweight='bold')
    axes[0, 1].set_ylabel("Seconds (s)")
    for i, v in enumerate(travel_times):
        txt = f"{v:.1f}s" if v < 75 else "Timeout"
        axes[0, 1].text(i, min(v + 1.5, 75), txt, ha='center', fontweight='bold', fontsize=9)

    # 3. 实际行驶轨迹长度
    axes[1, 0].bar(short_names, path_lengths, color=palette, width=0.55, edgecolor='#1e293b')
    axes[1, 0].set_title("Actual Trajectory Length (m)", fontweight='bold')
    axes[1, 0].set_ylabel("Meters (m)")
    for i, v in enumerate(path_lengths):
        axes[1, 0].text(i, v + 0.3, f"{v:.2f}", ha='center', fontweight='bold', fontsize=9)

    # 4. 最小避障间隙
    axes[1, 1].bar(short_names, clearances, color=palette, width=0.55, edgecolor='#1e293b')
    axes[1, 1].set_title("Minimum Clearance to Obstacles (m)", fontweight='bold')
    axes[1, 1].set_ylabel("Meters (m)")
    for i, v in enumerate(clearances):
        axes[1, 1].text(i, v + 0.02, f"{v:.2f}", ha='center', fontweight='bold', fontsize=9)

    for ax in axes.flat:
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        ax.set_axisbelow(True)

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "metrics_comparison.png")
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Generated: {out_path}")

def plot_velocity_profiles(data):
    """绘制线速度与角速度时域曲线"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle("Robot Control Velocity Profiles over Time (v & omega)", fontsize=14, fontweight='bold')

    colors = {
        'A* + DWA (Grid Search)': '#1d4ed8',
        'Voronoi + DWA (GVD Skeleton)': '#0d9488',
        'PSO + DWA (Swarm Intelligence)': '#d97706',
        'RRT* + DWA (Random Sampling)': '#7c3aed',
        'Informed Bi-RRT* (Proposed SOTA)': '#0284c7',
        'Classical APF (Khatib 1986)': '#e11d48',
        'Improved APF (Ge & Cui 2000)': '#ea580c',
        'A* + MPC (Model Predictive)': '#059669'
    }
    name_map = {
        'A* + DWA (Grid Search)': 'A*',
        'Voronoi + DWA (GVD Skeleton)': 'Voronoi',
        'PSO + DWA (Swarm Intelligence)': 'PSO',
        'RRT* + DWA (Random Sampling)': 'RRT*',
        'Informed Bi-RRT* (Proposed SOTA)': 'Informed-RRT*',
        'Classical APF (Khatib 1986)': 'C-APF',
        'Improved APF (Ge & Cui 2000)': 'I-APF',
        'A* + MPC (Model Predictive)': 'MPC'
    }

    for name, info in data.items():
        traj = info.get('trajectory', [])
        if not traj:
            continue
        ts = [p['t'] for p in traj]
        vs = [p['v'] for p in traj]
        ws = [p['omega'] for p in traj]

        lbl = name_map.get(name, name.split(' ')[0])
        ax1.plot(ts, vs, label=lbl, color=colors[name], linewidth=1.5)
        ax2.plot(ts, ws, label=lbl, color=colors[name], linewidth=1.5)

    ax1.set_ylabel("Linear Velocity v (m/s)", fontsize=11)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right', fontsize=8.5)

    ax2.set_ylabel("Angular Velocity omega (rad/s)", fontsize=11)
    ax2.set_xlabel("Simulation Time (s)", fontsize=11)
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "velocity_profiles.png")
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Generated: {out_path}")

def main():
    data = load_data()
    if not data:
        print("No trajectory data found!")
        return
    plot_trajectories(data)
    plot_pso_convergence(data)
    plot_metrics(data)
    plot_velocity_profiles(data)
    print("All enhanced charts generated successfully!")

if __name__ == "__main__":
    main()
