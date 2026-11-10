"""
仿真监控与动态障碍物运动控制模块 (Robot Supervisor Controller)
负责动态障碍物的运动学轨迹推演、碰撞判定、实验指标监控与自动化仿真启停。
"""

import math
import os
import sys
import time
import json
from controller import Supervisor

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    dt = timestep / 1000.0

    # 1. 获取动态障碍物与机器人节点句柄
    obs1_node = supervisor.getFromDef("OBS_DYNAMIC_1")
    obs2_node = supervisor.getFromDef("OBS_DYNAMIC_2")
    obs3_node = supervisor.getFromDef("OBS_DYNAMIC_3")
    robot_node = supervisor.getFromDef("ROBOT")

    trans_field_1 = obs1_node.getField("translation") if obs1_node else None
    trans_field_2 = obs2_node.getField("translation") if obs2_node else None
    trans_field_3 = obs3_node.getField("translation") if obs3_node else None
    robot_trans = robot_node.getField("translation") if robot_node else None

    # 读取环境变量中的仿真模式控制
    is_benchmark_mode = os.environ.get("BENCHMARK_MODE", "0") == "1"
    algo = os.environ.get("PLANNER_ALGO", "a_star_dwa")

    print(f"==================================================")
    print(f"  [Supervisor] 仿真监控器就绪")
    print(f"  动态障碍物 1: 水平往复巡逻 (速度 ~0.35 m/s)")
    print(f"  动态障碍物 2: 垂直往复巡逻 (速度 ~0.40 m/s)")
    print(f"  动态障碍物 3: 对角线往复巡逻 (速度 ~0.25 m/s)")
    print(f"  自动评测模式: {'开启' if is_benchmark_mode else '关闭 (交互演示)'}")
    print(f"==================================================")

    sim_time = 0.0
    collision_count = 0
    min_dist_to_dyn = float('inf')

    # 动态障碍物碰撞半径 (机器人 0.18 + 障碍物 0.35)
    COLLISION_THRESH_1 = 0.50
    COLLISION_THRESH_2 = 0.50
    COLLISION_THRESH_3 = 0.45

    goal_pos = (4.0, 4.0)
    goal_radius = 0.30

    while supervisor.step(timestep) != -1:
        sim_time += dt

        # 1. 动态障碍物 1: 水平正弦往复巡逻
        # 范围 x in [-2.5, 0.5], y = -0.6
        if trans_field_1:
            x1 = -1.0 + 1.5 * math.sin(2.0 * math.pi * sim_time / 10.0)
            y1 = -0.6
            z1 = 0.3
            trans_field_1.setSFVec3f([x1, y1, z1])
            obs1_node.resetPhysics()

        # 2. 动态障碍物 2: 垂直正弦往复巡逻
        # 范围 x = 0.6, y in [-0.8, 2.4]
        if trans_field_2:
            x2 = 0.6
            y2 = 0.8 + 1.6 * math.sin(2.0 * math.pi * sim_time / 8.0)
            z2 = 0.3
            trans_field_2.setSFVec3f([x2, y2, z2])
            obs2_node.resetPhysics()

        # 3. 动态障碍物 3: 对角线慢速巡逻
        # 范围 x in [-1.8, 0.4], y in [1.2, 2.4]
        if trans_field_3:
            phase = math.sin(2.0 * math.pi * sim_time / 12.0)
            x3 = -0.7 + 1.1 * phase
            y3 = 1.8 + 0.6 * phase
            z3 = 0.3
            trans_field_3.setSFVec3f([x3, y3, z3])
            obs3_node.resetPhysics()

        # 4. 碰撞与距离监控
        if robot_trans:
            rpos = robot_trans.getSFVec3f()
            rx, ry = rpos[0], rpos[1]

            # 计算与各个动态障碍物的实时距离
            d1 = math.hypot(rx - x1, ry - y1)
            d2 = math.hypot(rx - x2, ry - y2)
            d3 = math.hypot(rx - x3, ry - y3)
            current_dyn_min = min(d1, d2, d3)
            if current_dyn_min < min_dist_to_dyn:
                min_dist_to_dyn = current_dyn_min

            if d1 < COLLISION_THRESH_1 or d2 < COLLISION_THRESH_2 or d3 < COLLISION_THRESH_3:
                collision_count += 1
                if collision_count == 1 or collision_count % 30 == 0:
                    print(f"[Supervisor WARNING] 发生与动态障碍物碰撞事件! t={sim_time:.2f}s, 最近距离={current_dyn_min:.3f}m")

            # 检查是否成功到达终点
            dist_to_goal = math.hypot(rx - goal_pos[0], ry - goal_pos[1])
            if dist_to_goal <= goal_radius:
                # 延时一小段保证机器人控制器完成数据写入
                if sim_time > 1.0:
                    print(f"[Supervisor] 监控到机器人成功到达目标终点! 总耗时: {sim_time:.2f} s")
                    if is_benchmark_mode:
                        # 留 0.5s 让导航器完成落盘
                        for _ in range(15):
                            supervisor.step(timestep)
                        supervisor.simulationQuit(0)
                        break

            # 超时退出 (Benchmark 保护)
            if is_benchmark_mode and sim_time >= 80.0:
                print(f"[Supervisor] 评测超时，强制终止当前仿真。")
                supervisor.simulationQuit(1)
                break

    print(f"[Supervisor] 监控结束: 动态障碍物碰撞次数 = {collision_count}, 动态最小间隙 = {min_dist_to_dyn:.3f} m")

if __name__ == "__main__":
    main()
