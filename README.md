# Webots Path Planning Benchmark

**在线导航与安全路径平滑 · Dev1ce's method**

[项目主页](https://yandick.github.io/webots-path-planning-benchmark/) · [方法与实验报告](docs/coursework_path_planning_report.md) · [网站构建与发布](docs/website.md) · [原始数据](results/summary.csv)

项目主页可以回放六种方法在五个种子下的轨迹，对照每次运行的数据，也可以播放 1080p 同步录像。主页由 GitHub Pages 发布，首次启用与自动部署步骤见[发布说明](docs/website.md)。

在唯一的 10 m × 10 m 仓库地图上，评估五个经典方法 A*、RRT*、Voronoi、APF、DWA，以及 Dev1ce's method。地图包含七组货架和两辆按固定时序运动的小车。

![仓库地图](results/map.png)

## 比较方式

- **全局规划器组：** A*、RRT*、Voronoi、Dev1ce's method 共用方向跟踪器。
- **局部控制器组：** APF、DWA 与公共方向跟踪器采用共同的 A* 引导机制。A* 的运行记录同时作为本组的跟踪器对照。
- 六种方法共用相同的障碍观测层、堵塞触发重规划规则、机器人约束和制动规则。

| 方法 | 配置值 | 角色 / 参考路线 |
| --- | --- | --- |
| A* | astar | 在线全局规划，公共跟踪器 |
| RRT* | rrt_star | 在线全局规划，公共跟踪器 |
| Voronoi | voronoi | 在线全局规划，公共跟踪器 |
| Dev1ce's method | voronoi_qp | Voronoi 参考路线的安全气泡平滑，公共跟踪器 |
| APF | apf | 势场局部控制器，A* 引导 |
| DWA | dwa | 动态窗口局部控制器，A* 引导 |

动态层只使用激光测量：命中点标记、射线清除旧占据、观测最长保留 1 s。每个控制步检查当前路线及机器人到路线的连接段；发现堵塞时重新调用对应规划器，连续规划尝试至少间隔 1 s。物理障碍时序与机器人、算法身份无关。

Dev1ce's method 是本项目中 Voronoi 参考路线与安全气泡 QP 平滑方案的展示名称，采用 Wen 等人 G²VD 论文第 IV-D 节的平滑目标与气泡约束。[论文 DOI](https://doi.org/10.1109/TASE.2024.3398996) · [完整方法与答辩说明](docs/coursework_path_planning_report.md)

网页、文档和图表统一使用 **Dev1ce's method**。配置值、函数名及原始实验数据继续使用稳定标识 `voronoi_qp`，因此已有命令、配置和归档结果仍然兼容。该名称不改变算法实现及其论文来源。

## 正式二维结果

每种方法五个 seed，共 30 次。seed 改变预设小车动作时间，RRT* 同时使用对应固定采样种子。

表中耗时和行程只统计成功到达的运行。时间图把失败记为 100 s。每次运行记下车身边缘离障碍物最近时的距离，再对全部五次取平均。平顺性也统计全部五次。

| 规划器 | 无碰撞到达 | 成功平均耗时 | 成功平均行程 | 碰撞 / 超时 | 平均成功重规划次数 |
| --- | --- | --- | --- | --- | --- |
| A* | 4/5 | 30.40 s | 12.763 m | 1 / 0 | 4.2 |
| RRT* | 5/5 | 30.64 s | 12.987 m | 0 / 0 | 3.6 |
| Voronoi | 5/5 | 30.26 s | 12.475 m | 0 / 0 | 1.0 |
| Dev1ce's method | 5/5 | 30.80 s | 13.172 m | 0 / 0 | 1.2 |

| 控制器（共同 A* 引导） | 无碰撞到达 | 成功平均耗时 | 与障碍物的最小距离（五次平均） | 运动中指令角加速度 RMS |
| --- | --- | --- | --- | --- |
| 公共方向跟踪器 | 4/5 | 30.40 s | 0.0773 m | 0.9304 rad/s² |
| APF 控制器 | 5/5 | 33.44 s | 0.1441 m | 0.4471 rad/s² |
| DWA 控制器 | 5/5 | 49.70 s | 0.1036 m | 0.4572 rad/s² |

A* 在 seed 4 与侧向驶入的装卸车发生一次碰撞，已计入成功率和原始数据。其余方法全部到达。

| 指标（五次平均） | Voronoi | Dev1ce's method | 变化 |
| --- | --- | --- | --- |
| 与障碍物的最小距离 | 0.1316 m | 0.1998 m | 增加 51.8% |
| 初始路径峰值曲率 | 10.574 m⁻¹ | 1.353 m⁻¹ | 降低 87.2% |
| 运动中指令角加速度 RMS | 0.7650 rad/s² | 0.5626 rad/s² | 降低 26.5% |

Dev1ce's method 相对 Voronoi 的成功平均耗时增加 0.54 s，实际平均行程增加 0.698 m。

![分组比较](results/benchmark.png)
![安全和平顺性](results/safety_smoothness.png)
![六方法轨迹与重规划](results/warehouse.png)
![安全气泡与拐角](results/voronoi_smoothing.png)
![A* seed 4 的碰撞记录](results/collision_case.png)

[1080p 同步视频](results/warehouse.mp4) · [逐次汇总](results/summary.csv) · [完整轨迹与路线更新](results/raw_runs.json.gz) · [实验配置与来源](results/manifest.json)

## Webots 原生验证

Webots R2025a 的 seed 0 原生运行中，六种方法均无碰撞到达。对应代码提交为 `3476074`，[CI 验证记录](https://github.com/Yandick/webots-path-planning-benchmark/actions/runs/43825918398)包含 26 项测试和六次原生运行。

| 方法 | 结果 | 耗时 | 与障碍物的最小距离 | 成功重规划 / 尝试 |
| --- | --- | --- | --- | --- |
| A* | 到达 | 31.904 s | 0.1003 m | 6 / 13 |
| RRT* | 到达 | 33.440 s | 0.0997 m | 7 / 14 |
| Voronoi | 到达 | 30.144 s | 0.1850 m | 3 / 12 |
| APF 控制器 | 到达 | 33.856 s | 0.1564 m | 6 / 13 |
| DWA 控制器 | 到达 | 46.816 s | 0.1146 m | 11 / 11 |
| Dev1ce's method | 到达 | 31.968 s | 0.2399 m | 2 / 11 |

RRT* 的最长连续静止为 0.288 s。原生雷达最近测量点的几何误差最大为 0.0110 m。

[原生验证摘要](results/webots_validation.json) · [六次完整原生日志](results/webots_raw_runs.json.gz)

![Webots 原生截图：Dev1ce's method，t=28 s](results/webots_scene.png)

## 运行

```bash
python -m pip install -r requirements.txt
python scripts/generate_worlds.py
python -m unittest discover -s tests -v
python evaluation/benchmark.py --seeds 0 1 2 3 4
python evaluation/plot_results.py --video
```

二维后端无需 GUI 或 Webots，可以在无显示 Linux 服务器上导出图片；视频导出需要 PATH 中存在 ffmpeg。

安装 Webots R2025a，选用安装了 requirements 的 Python，打开唯一世界文件 `worlds/warehouse.wbt`。默认方法为 Dev1ce's method（配置值 `voronoi_qp`），通过 `config.json` 或 `PLANNER_ALGO` 切换。

```bash
PLANNER_ALGO=astar bash run_simulation.sh
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py
```

可通过 `WEBOTS_BINARY` 指定可执行文件。无显示 Linux 使用 xvfb-run 保留雷达渲染。原生导航周期 96 ms，监督器每 32 ms 检查车身边缘到最近障碍物的距离。`RECORD_VIDEO=1` 可录制原生视频，也会自动导出截图。

## 内容与依赖

- `navigation/scenarios.py`：唯一物理地图、小车时序与独立几何评估。
- `navigation/grid_map.py`、`observation_map.py`：静态栅格、观测占据层和射线清除。
- `navigation/classical.py`：RRT*、Voronoi 骨架与直线简化。
- `navigation/core.py`：A*、在线重规划、公共跟踪器和物理约束。
- `navigation/local.py`：带阻尼的 APF、归一化 DWA 与连续圆弧碰撞距离。
- `navigation/smoothing.py`：Dev1ce's method 的安全气泡 QP 和样条插值。
- `navigation/metrics.py`：弧长采样、曲率、角加速度与停留指标。
- `navigation/settings.py`：方法来源、参数和分组协议。
- `controllers/coursework_*`：原生导航和独立监督评估。
- `evaluation/`：实验、检查、图片与视频导出。

Python 依赖为 NumPy、SciPy、Matplotlib。结果保留当前比较、一次平滑消融和对应可视化。
