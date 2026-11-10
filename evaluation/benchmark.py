"""
自动化批量仿真与基准评测套件 (Automated Benchmark Suite)
支持运行全部 5 种算法 (A*, Voronoi, PSO, RRT*, APF)，输出量化对比 Markdown 表格并自动绘制全套图表。
"""

import os
import sys
import time
import json
import subprocess

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(PROJECT_DIR, "results")
WORLD_FILE = os.path.join(PROJECT_DIR, "worlds", "dynamic_obstacle_arena.wbt")
WEBOTS_SH = os.environ.get("WEBOTS_HEADLESS")
if not WEBOTS_SH or not os.path.exists(WEBOTS_SH):
    candidates = [
        os.path.join(PROJECT_DIR, "scripts", "webots_headless.sh"),
        os.path.join(PROJECT_DIR, "..", "webots_headless.sh"),
        "/data/yhwu/robot/webots_headless.sh"
    ]
    for c in candidates:
        if os.path.exists(c):
            WEBOTS_SH = c
            break
    else:
        WEBOTS_SH = "webots"

ALGORITHMS = [
    ("a_star_dwa", "A* + DWA (栅格图搜索)"),
    ("voronoi_dwa", "Voronoi + DWA (广义沃罗诺伊骨架图)"),
    ("pso_dwa", "LDIW-PSO + DWA (自适应粒子群优化)"),
    ("rrt_star_dwa", "RRT* + DWA (渐近最优随机采样)"),
    ("informed_rrt_star_dwa", "Informed Bi-RRT* + DWA (改进椭球采样+B样条)"),
    ("apf", "Classical APF (经典人工势场法)"),
    ("improved_apf", "Improved APF (改进人工势场法)"),
    ("mpc", "A* + MPC (模型预测控制)")
]

def run_single_algorithm(algo_name: str, timeout_sec: int = 65):
    print(f"\n========================================================")
    print(f"  [Benchmark] 正在启动算法仿真: {algo_name.upper()}")
    print(f"========================================================")

    env = os.environ.copy()
    env["BENCHMARK_MODE"] = "1"
    env["PLANNER_ALGO"] = algo_name

    cmd = [WEBOTS_SH, "--batch", "--mode=fast", WORLD_FILE]
    
    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
            text=True
        )
        elapsed = time.time() - t0
        print(f"  [Benchmark] 仿真执行完成 (耗时: {elapsed:.2f}s, 退出码: {proc.returncode})")
    except subprocess.TimeoutExpired:
        print(f"  [Benchmark] 仿真超时 ({timeout_sec}s)，强制终止。")

def summarize_results():
    results = []
    for key, display_name in ALGORITHMS:
        res_file = os.path.join(RESULTS_DIR, f"trajectory_{key}.json")
        if not os.path.exists(res_file):
            continue
        with open(res_file, "r", encoding="utf-8") as f:
            d = json.load(f)

        plan_info = d.get("global_plan_info", {})
        plan_time = plan_info.get("planning_time_ms", 0.0)
        success = d.get("success", False)
        travel_time = d.get("finish_time", 0.0)
        dist = d.get("actual_path_length", 0.0)
        clearance = d.get("min_clearance", 0.0)
        if clearance == float("inf"):
            clearance = 0.0

        results.append({
            "algorithm": display_name,
            "plan_time_ms": plan_time,
            "success": "成功到达" if success else "死锁/超时",
            "travel_time_s": travel_time,
            "path_length_m": dist,
            "min_clearance_m": clearance
        })

    print("\n" + "="*88)
    print("                     多流派路径规划与避障算法量化对比综合评测表")
    print("="*88)
    print(f"{'算法流派与名称':<32} | {'规划耗时(ms)':<12} | {'导航耗时(s)':<10} | {'实际轨迹长度(m)':<14} | {'最小安全间隙(m)':<12} | {'状态'}")
    print("-" * 88)
    for r in results:
        print(f"{r['algorithm']:<32} | {r['plan_time_ms']:<12.2f} | {r['travel_time_s']:<10.2f} | {r['path_length_m']:<14.2f} | {r['min_clearance_m']:<12.2f} | {r['success']}")
    print("="*88)

    # 导出 Markdown 表格
    md_table_path = os.path.join(RESULTS_DIR, "comparison_table.md")
    with open(md_table_path, "w", encoding="utf-8") as f:
        f.write("# 路径规划算法横向性能量化对比数据表\n\n")
        f.write("| 技术流派与算法组合 | 全局规划耗时 (ms) | 实际导航到达耗时 (s) | 行驶轨迹长度 (m) | 最小避障间距 (m) | 任务完成状态 |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: | :---: |\n")
        for r in results:
            f.write(f"| **{r['algorithm']}** | {r['plan_time_ms']:.2f} | {r['travel_time_s']:.2f} | {r['path_length_m']:.2f} | {r['min_clearance_m']:.2f} | {r['success']} |\n")
    print(f"\nMarkdown 对比表格已导出至: {md_table_path}")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-all", action="store_true", help="重新运行全部仿真实验")
    args = parser.parse_args()

    if args.run_all:
        for key, _ in ALGORITHMS:
            run_single_algorithm(key)

    summarize_results()

    # 自动重绘所有图表
    from plot_results import main as plot_main
    plot_main()

if __name__ == "__main__":
    main()
