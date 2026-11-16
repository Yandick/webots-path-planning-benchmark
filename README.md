# 机器人避障课设：仓库地图与事件触发重规划

一张 10 m × 10 m 的仓库地图：七组货架、连续转弯、中央短通道、上方绕行通道，以及两辆按固定时序运动的小车。研究的问题是：原路线被占用时，机器人能否及时换路，减少停等？

物理世界包含静态障碍和动态障碍。机器人使用已知静态栅格地图和激光雷达观测；“激光点”是雷达测到的障碍物表面位置。地图采用静态层、传感器障碍更新和安全膨胀，结构参照 [Nav2 官方配置](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml)。

![唯一仓库地图](results/map.png)

五个经典 baseline 分别运行，再与事件触发改进方法比较：

| 方法 | 配置值 | 工作方式 |
| --- | --- | --- |
| A* | astar | 栅格搜索得到路线，统一跟踪器执行 |
| RRT* | rrt_star | 随机采样与重连得到路线，统一跟踪器执行 |
| Voronoi | voronoi | 沿自由空间骨架搜索路线，统一跟踪器执行 |
| APF | apf | 根据目标引力与障碍斥力直接输出速度 |
| DWA | dwa | 根据目标与扫描，对候选速度轨迹评分 |
| 事件触发改进 | replan_dwa | 前方路线持续占用或停滞时更新 A* 路线，由 DWA 执行 |

前三种方法的跟踪器只负责沿既定路线行驶、限速与遇障制动，不使用 DWA，也不生成替代路线。六种方法使用相同机器人尺寸、速度与加速度限制、传感器和小车运动时序。

改进动机、方法细节、实验结果和答辩说明见 [课设报告](docs/coursework_path_planning_report.md)。

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

[六方法同步回放](results/warehouse.mp4) · [逐次结果](results/summary.csv) · [完整轨迹压缩包](results/raw_runs.json.gz)

五个时序 seed 的二维实验结果如下。时间为成功运行的平均到达时间。

| 方法 | 到达次数 | 平均时间 | 碰撞次数 |
| --- | --- | --- | --- |
| A* | 4/5 | 74.28 s | 1 |
| RRT* | 4/5 | 74.80 s | 1 |
| Voronoi | 5/5 | 29.10 s | 0 |
| APF | 0/5 | 5 次超时 | 0 |
| DWA | 0/5 | 5 次超时 | 0 |
| 事件触发改进 | 5/5 | 38.82 s | 0 |

Voronoi 从起点选择上方通道，在这张地图上最快。改进方法接近中央通道后根据占用换路，缩短了 A*、RRT* 的等待，并完成全部五次任务。


五个 seed 改变小车动作的时间偏移（seed 0 为零，其余为 [-1,1] s），布局和起终点不变；RRT* 使用对应 seed 固定采样序列。每个 seed 的各方法使用相同小车运动时序，结果表包含所有运行。

## Webots 原生仿真

安装匹配本机系统的 Webots R2025a，选择安装了 requirements 的 Python，打开唯一世界文件 `worlds/warehouse.wbt`。默认 `replan_dwa`，可修改 `config.json` 或用环境变量 `PLANNER_ALGO` 切换方法。Windows 不可直接运行项目外的 Linux Webots 安装包。

Linux 图形或无显示环境均可运行：

```bash
PLANNER_ALGO=replan_dwa bash run_simulation.sh
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py
```

可用 `WEBOTS_BINARY` 指定原生可执行文件。无显示 Linux 需要 xvfb-run，必须保留渲染以获得真实激光观测。四个 90° 原生雷达扇区组成 360° 扫描，接口依据 [Webots 官方 Lidar 文档](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/lidar.md)。设置 `RECORD_VIDEO=1` 可录制原生视频；每次运行也会导出场景截图。fast 模式截图可能滞后于仿真时刻，精确运动以同步日志为准。

原生仿真已完成。[CI 记录](https://github.com/Yandick/webots-path-planning-benchmark/actions/runs/43806946635) · [结果摘要](results/webots_validation.json) · [六次完整轨迹](results/webots_raw_runs.json.gz)

| 方法 | Webots seed 0 |
| --- | --- |
| A* | 74.688 s 到达 |
| RRT* | 74.528 s 到达 |
| Voronoi | 29.280 s 到达 |
| APF | 100 s 超时 |
| DWA | 100 s 超时 |
| 事件触发改进 | 38.208 s 到达，重规划 2 次 |

六次原生运行均无几何碰撞。15 项回归检查、六种方法的原生执行和激光点几何核对全部通过。

![Webots 原生场景](results/webots_scene.png)

## 文件依赖

| 内容 | 唯一来源／入口 |
| --- | --- |
| 地图几何、小车时间表 | navigation/scenarios.py |
| 通用栅格与膨胀 | navigation/grid_map.py |
| 方法名称、默认配置解析 | navigation/settings.py |
| 激光观测速度估计 | navigation/tracking.py |
| RRT*、Voronoi、APF | navigation/classical.py |
| A*、DWA、重规划触发 | navigation/core.py |
| 原生机器人模型 | assets/robot.wbo.in |
| 世界生成 | scripts/generate_worlds.py → worlds/warehouse.wbt |
| 原生适配 | controllers/coursework_navigator、coursework_supervisor |
| 二维实验／绘图 | evaluation/benchmark.py、plot_results.py |

旧控制器、旧算法、旧地图、旧绘图入口及旧结果已从当前目录移除。当前代码不再导入旧控制器，也不再读取旧地图。原始轨迹只保留一份 gzip 压缩包，绘图直接读取它；Webots 临时日志不提交 Git。历史内容仍在 Git 历史中。
