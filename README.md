# 机器人避障课设：Voronoi–Visibility（VV）复现与对比

在一张 10 m × 10 m 仓库地图上，对比五个经典算法与已有的 Voronoi 改进算法 **Voronoi–Visibility（VV）**。地图包含七组货架、两条主要通路和两辆按固定时间表运动的小车。

本课设的改进方案采用 Niu、Savvaris、Tsourdos、Ji 的 *Voronoi-Visibility Roadmap-based Path Planning Algorithm for Unmanned Surface Vehicles*，发表于 **The Journal of Navigation，2019**。[论文与 DOI](https://doi.org/10.1017/S0373463318001005) · [作者机构公开全文](https://orca.cardiff.ac.uk/id/eprint/118170/1/Voronoi-Visibility%20Roadmap-based%20Path%20Planning%20Algorithm%20for%20Unmanned%20Su.._.pdf)

VV 在 Voronoi 路径节点之间添加安全可见直线边，再用 Dijkstra 搜索总长度更短的路线。本仓库按论文第 4.5–4.6 节在统一膨胀栅格上用 Python 复现。

![仓库地图](results/map.png)

| 方法 | 配置值 | 执行方式 |
| --- | --- | --- |
| A* | astar | 静态栅格搜索＋统一路径跟踪器 |
| RRT* | rrt_star | 随机树采样、重连＋统一路径跟踪器 |
| Voronoi | voronoi | 骨架图搜索、贪心直线简化＋统一路径跟踪器 |
| APF | apf | 引力与斥力直接产生控制指令 |
| DWA | dwa | 对候选速度轨迹进行预测与评分 |
| Voronoi–Visibility | voronoi_visibility | 论文 VV 可见图搜索＋同一统一路径跟踪器 |

Voronoi 与 VV 使用相同的跟踪器、传感器、机器人尺寸、速度限制和障碍膨胀半径。VV 的改动集中在路径规划，初始路径是两者的直接比较对象。

## 运行与演示素材

```bash
python -m pip install -r requirements.txt
python scripts/generate_worlds.py
python -m unittest discover -s tests -v
python evaluation/benchmark.py --seeds 0 1 2 3 4
python evaluation/plot_results.py --video
```

上述命令运行二维运动学后端，无需 GUI 或 Webots。`--video` 需要 PATH 中存在 ffmpeg。

![六方法结果](results/benchmark.png)
![六方法轨迹](results/warehouse.png)
![Voronoi 与 VV 的规划路线](results/voronoi_vs_vv.png)

[1080p 同步视频](results/warehouse.mp4) · [逐次结果](results/summary.csv) · [完整轨迹](results/raw_runs.json.gz) · [方法与答辩说明](docs/coursework_path_planning_report.md)

每种方法运行五个 seed，共 30 次。seed 改变小车动作时间的偏移，RRT* 同时固定对应采样序列；每个 seed 的所有方法使用同一套运动时序。

| 方法 | 无碰撞到达 | 成功运行平均耗时 | 成功运行平均行程 |
| --- | --- | --- | --- |
| A* | 4/5 | 74.28 s | 11.570 m |
| RRT* | 4/5 | 74.80 s | 11.660 m |
| Voronoi | 5/5 | 29.10 s | 12.367 m |
| APF | 0/5 | 5 次超时 | — |
| DWA | 0/5 | 5 次超时 | — |
| VV（论文复现） | 5/5 | 28.64 s | 12.099 m |

VV 相对现有 Voronoi baseline：规划折线长度减少 **2.63%**，实际平均行程减少 **2.17%**，平均到达时间减少 **1.58%**。

## Webots 原生仿真

安装 Webots R2025a，选用安装了 requirements 的 Python，打开唯一世界文件 `worlds/warehouse.wbt`。默认方法为 `voronoi_visibility`，可通过 `config.json` 或环境变量 `PLANNER_ALGO` 切换上表的方法。

```bash
PLANNER_ALGO=voronoi_visibility bash run_simulation.sh
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py
```

可用 `WEBOTS_BINARY` 指定本机 Webots 可执行文件。无显示 Linux 需要 xvfb-run，并保留渲染以获得原生激光观测。四个 90° 雷达扇区构成 360° 扫描，控制与传感器周期为 96 ms，Supervisor 每 32 ms 检查实际几何净空。`RECORD_VIDEO=1` 可录制原生视频，运行过程也会导出截图。

原生实验由 GitHub Actions 运行，六种方法各运行 seed 0。

## 内容与依赖

- `navigation/scenarios.py`：唯一地图几何和小车运动时序。
- `navigation/grid_map.py`：栅格、膨胀与逐格连线碰撞检查。
- `navigation/classical.py`：RRT*、Voronoi、论文 VV、APF。
- `navigation/core.py`：A*、DWA、统一跟踪器和机器人约束。
- `navigation/tracking.py`：DWA 使用的连续激光观测速度估计。
- `navigation/settings.py`：方法列表与配置解析。
- `assets/robot.wbo.in`、`scripts/generate_worlds.py`：模型与世界生成。
- `controllers/coursework_*`：Webots 导航和独立评估。
- `evaluation/`：实验、结果检查与绘图入口。

结果只保留当前算法的汇总、完整压缩轨迹与演示素材。旧改进方法及其数据和说明已移除。
