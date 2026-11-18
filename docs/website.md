# 主页构建与发布

[项目主页](https://yandick.github.io/webots-path-planning-benchmark/) 由 GitHub Pages 发布。网页源码在 `site/`，静态构建输出在 `dist/`。

## 本地预览

```bash
python -m pip install -r requirements.txt
python scripts/build_site.py
python -m http.server 8000 --directory dist
```

打开 <http://localhost:8000/>。首页左右两个回放共用一条时间轴，默认从 0 s 开始。可以选 A* 或 RRT* 的原版和提前让行版，也可以自己换方法、地图和种子。点让行或碰撞时刻能跳到对应画面。结果表覆盖全部八种方法，二维结果来自五个种子，Webots 结果来自种子 0。

## 更新地图或实验

公开布局和小车时间表位于 [地图目录](../assets/maps/README.md)。选择新的起终点或修改小车路线后，在项目根目录重新生成世界和结果。

```bash
python scripts/generate_worlds.py
python evaluation/benchmark.py --workers 4
python evaluation/plot_results.py --video
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py --export results
python -m unittest discover -s tests -v
node --check site/app.js
python scripts/build_site.py
```

网页构建读取 `results/manifest.json`、汇总表和真实轨迹。每条运行的地图、种子、几何与小车时序都要与场景签名相符。构建时还会核对每对方法的初始路线以及让行区间和总时长。Webots 单独归档并校验。应用脚本、样式和数据文件带有按内容计算的缓存版本。[构建脚本](../scripts/build_site.py)

## GitHub Pages

`main` 分支上的推送会触发 Project website 工作流。构建作业运行测试并生成 `dist/`，发布作业将其部署到 Pages。首次部署到新仓库时，在 **Settings / Pages / Build and deployment** 选择 **GitHub Actions**。Pull Request 会运行网站检查。[项目发布记录](https://github.com/Yandick/webots-path-planning-benchmark/actions/workflows/pages.yml)

Navigation checks 另行运行三图的 Webots R2025a 检查，在工作流产物中保存完整控制器日志和验证摘要。[原生验证工作流](../.github/workflows/navigation.yml)
