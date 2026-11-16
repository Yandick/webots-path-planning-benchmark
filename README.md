# 机器人避障课设：仓库地图与事件触发重规划

一张 10 m × 10 m 的仓库地图：七组货架、连续转弯、中央短通道、上方绕行通道，以及两辆按固定时序运动的小车。研究的问题是：原路线被占用时，机器人能否及时换路，减少停等？

物理世界只有静态障碍和动态障碍。机器人使用已知静态栅格地图和激光雷达观测；“激光点”是雷达测到的障碍物表面位置。静态地图、传感器障碍更新、安全膨胀这一结构可参照 [Nav2 官方配置](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml)。本项目在 Webots 中独立实现了简化版本，未集成 ROS/Nav2；仓库布局是课设自建场景。

![唯一仓库地图](results/map.png)

三种方法共享 A*、DWA、速度限制、激光观测和短时速度估计，仅改变全局重规划策略：

| 方法 | 全局路径何时更新 |
| --- | --- |
| fixed_dwa | 只在启动时规划一次 |
| periodic_dwa | 有动态障碍观测时，每隔至少 4 s 重规划 |
| replan_dwa | 前方路线被持续占用 0.8 s，或停滞约 2.5 s 时，触发重规划 |

改进的直观动机、方法细节、实验边界和答辩说明见 [课设报告](docs/coursework_path_planning_report.md)。不使用“SOTA”或全场景最优等称谓。

## 运行与可视化

无 GUI、无 Webots 也能跑明确标注的二维运动学实验，并导出 PNG 和 MP4：

```bash
python -m pip install -r requirements.txt
python scripts/generate_worlds.py
python -m unittest discover -s tests -v
python evaluation/benchmark.py --seeds 0 1 2 3 4
python evaluation/plot_results.py --video
```

`--video` 需要 PATH 中存在 ffmpeg；只导出图片时去掉该参数。

![方法比较](results/benchmark.png)
![同图运行轨迹](results/warehouse.png)

[三方法同步回放](results/warehouse.mp4) · [逐次结果](results/summary.csv) · [完整轨迹压缩包](results/raw_runs.json.gz)

当前二维实验共 15 次，三种方法均 5/5 无碰撞到达：

| 方法 | 平均到达时间 | 平均行程 | 五次重规划总数 |
| --- | --- | --- | --- |
| 固定 A*＋DWA | 78.08 s | 11.514 m | 0 |
| 定时重规划 | 40.38 s | 12.561 m | 37 |
| 事件触发重规划 | 37.00 s | 12.514 m | 10 |

事件触发比固定路径节省 52.61% 时间，行程增加约 8.69%；比定时重规划节省 8.37% 时间，重规划次数减少 72.97%。这是单张自建地图上的配对结果。

五个 seed 只改变小车动作的时间偏移（seed 0 为零，其余为 [-1,1] s），布局和起终点不变。每个 seed 的三种方法使用完全相同的运动时序。所有碰撞与超时都保留，不能把单图时间偏移实验解释为多地图泛化。

## Webots 原生仿真

安装匹配本机系统的 Webots R2025a，选择安装了 requirements 的 Python，打开唯一世界文件 `worlds/warehouse.wbt`。默认 `replan_dwa`，可修改 `config.json` 或用环境变量 `PLANNER_ALGO` 切换方法。Windows 不可直接运行项目外的 Linux Webots 安装包。

Linux 图形或无显示环境均可运行：

```bash
PLANNER_ALGO=replan_dwa bash run_simulation.sh
python evaluation/webots_benchmark.py --methods fixed_dwa periodic_dwa replan_dwa
python evaluation/check_webots_smoke.py
```

可用 `WEBOTS_BINARY` 指定原生可执行文件。无显示 Linux 需要 xvfb-run，必须保留渲染以获得真实激光观测。四个 90° 原生雷达扇区组成 360° 扫描，接口依据 [Webots 官方 Lidar 文档](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/lidar.md)。设置 `RECORD_VIDEO=1` 可录制原生视频；每次运行也会导出场景截图。fast 模式截图可能滞后于仿真时刻，精确运动以同步日志为准。

原生验证由 GitHub Actions 执行。二维结果与原生物理结果分别报告，旧简图的验证数字不适用于当前地图。

## 文件依赖

| 内容 | 唯一来源／入口 |
| --- | --- |
| 地图几何、小车时间表 | navigation/scenarios.py |
| 通用栅格与膨胀 | navigation/grid_map.py |
| 方法名称、默认配置解析 | navigation/settings.py |
| 激光观测速度估计 | navigation/tracking.py |
| A*、DWA、重规划触发 | navigation/core.py |
| 原生机器人模型 | assets/robot.wbo.in |
| 世界生成 | scripts/generate_worlds.py → worlds/warehouse.wbt |
| 原生适配 | controllers/coursework_navigator、coursework_supervisor |
| 二维实验／绘图 | evaluation/benchmark.py、plot_results.py |

旧控制器、旧算法、旧地图、旧绘图入口及旧结果已从当前目录移除。当前代码不再导入旧控制器，也不再读取旧地图。原始轨迹只保留一份 gzip 压缩包，绘图直接读取它；Webots 临时日志不提交 Git。历史内容仍在 Git 历史中。
