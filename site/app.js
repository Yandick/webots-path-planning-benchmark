/* Archived trajectories only: this page does not simulate or alter benchmark results. */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const data = window.BENCHMARK_DATA;
  const menuButton = document.querySelector(".menu-toggle");
  menuButton.addEventListener("click", function () {
    const expanded = this.getAttribute("aria-expanded") !== "true";
    this.setAttribute("aria-expanded", String(expanded));
    this.setAttribute("aria-label", expanded ? "收起导航" : "展开导航");
    $("main-nav").classList.toggle("open", expanded);
  });
  document.querySelectorAll("#main-nav a").forEach((link) => {
    link.addEventListener("click", () => {
      $("main-nav").classList.remove("open");
      menuButton.setAttribute("aria-expanded", "false");
      menuButton.setAttribute("aria-label", "展开导航");
    });
  });

  $("copy-commands").addEventListener("click", async () => {
    const code = $("quickstart-code");
    try {
      if (!navigator.clipboard || !window.isSecureContext) throw new Error("Clipboard unavailable");
      await navigator.clipboard.writeText(code.textContent);
      $("copy-feedback").textContent = "命令已复制。";
      $("copy-commands").textContent = "已复制 ✓";
    } catch (_) {
      const range = document.createRange();
      range.selectNodeContents(code);
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(range);
      $("copy-feedback").textContent = "已选中命令，请按 Ctrl+C / ⌘C 复制。";
    }
  });

  if (!data || data.schema_version !== 1 || !Array.isArray(data.runs) || !data.runs.length) {
    $("data-error").hidden = false;
    document.querySelectorAll(".playback button, .playback input, .playback select, .method-switcher button, #seed-select, .results-tabs button, #metric-select").forEach((element) => { element.disabled = true; });
    $("group-description").textContent = "交互数据暂时不可用。下方保留已归档的实验图、视频和原始数据下载。";
    return;
  }

  const methodById = new Map(data.methods.map((method) => [method.id, method]));
  const summaryById = new Map(data.summary.map((summary) => [summary.method, summary]));
  const runsByKey = new Map(data.runs.map((run) => [run.method + ":" + run.seed, run]));
  const colors = {
    astar: "#8366a5", rrt_star: "#477a9c", voronoi: "#a9853d",
    apf: "#b86b56", dwa: "#7a8f4c", voronoi_qp: "#20644b"
  };
  const state = {
    method: "voronoi_qp", seed: data.seeds[0], time: 18, playing: false,
    speed: 2, group: "global", metric: "clearance", previousFrame: null
  };
  const canvas = $("trajectory-canvas");
  const ctx = canvas.getContext("2d");
  let frameRequest = null;
  let canvasWidth = 1;
  let canvasHeight = 1;

  function label(id, localGroup) {
    if (localGroup && id === "astar") return "公共方向跟踪器";
    return methodById.get(id).label.replace(" controller", "");
  }
  function fixed(value, digits) {
    return value === null || value === undefined || !Number.isFinite(value) ? "—" : value.toFixed(digits);
  }
  function run() {
    return runsByKey.get(state.method + ":" + state.seed);
  }
  function element(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  /* Binary search plus interpolation preserves the recorded clock. */
  function sampleAt(trajectory, time) {
    const first = trajectory[0];
    if (time <= first[0]) {
      const ratio = first[0] > 0 ? Math.max(0, time / first[0]) : 1;
      return [time, data.scene.start[0] + ratio * (first[1] - data.scene.start[0]),
        data.scene.start[1] + ratio * (first[2] - data.scene.start[1]), first[3], first[4]];
    }
    const last = trajectory[trajectory.length - 1];
    if (time >= last[0]) return last.slice();
    let low = 0;
    let high = trajectory.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (trajectory[middle][0] <= time) low = middle;
      else high = middle;
    }
    const a = trajectory[low];
    const b = trajectory[high];
    const ratio = (time - a[0]) / (b[0] - a[0]);
    const angleDifference = Math.atan2(Math.sin(b[3] - a[3]), Math.cos(b[3] - a[3]));
    return [time, a[1] + (b[1] - a[1]) * ratio, a[2] + (b[2] - a[2]) * ratio,
      a[3] + angleDifference * ratio, a[4] + (b[4] - a[4]) * ratio];
  }

  function cartPosition(waypoints, time) {
    if (time <= waypoints[0][0]) return waypoints[0].slice(1);
    for (let i = 1; i < waypoints.length; i += 1) {
      if (time <= waypoints[i][0]) {
        const a = waypoints[i - 1];
        const b = waypoints[i];
        const ratio = (time - a[0]) / (b[0] - a[0]);
        return [a[1] + ratio * (b[1] - a[1]), a[2] + ratio * (b[2] - a[2])];
      }
    }
    return waypoints[waypoints.length - 1].slice(1);
  }

  function draw() {
    if (!ctx) return;
    const active = run();
    const bounds = data.scene.bounds;
    const spanX = bounds[1] - bounds[0];
    const spanY = bounds[3] - bounds[2];
    const side = Math.min(canvasWidth - 52, canvasHeight - 43);
    if (side <= 0) return;
    const scale = side / Math.max(spanX, spanY);
    const left = (canvasWidth - side) / 2;
    const top = (canvasHeight - side) / 2 + 3;
    const X = (x) => left + (x - bounds[0]) * scale;
    const Y = (y) => top + (bounds[3] - y) * scale;
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    ctx.fillStyle = "#fffef9";
    ctx.fillRect(0, 0, canvasWidth, canvasHeight);
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, top, side, side);
    ctx.clip();
    ctx.fillStyle = "#f1f3e9";
    ctx.fillRect(left, top, side, side);
    ctx.strokeStyle = "#dfe5d5";
    ctx.lineWidth = 0.6;
    for (let value = -5; value <= 5; value += 1) {
      ctx.beginPath(); ctx.moveTo(X(value), top); ctx.lineTo(X(value), top + side); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(left, Y(value)); ctx.lineTo(left + side, Y(value)); ctx.stroke();
    }
    data.scene.static.forEach((obstacle) => {
      const x = X(obstacle.center[0] - obstacle.size[0] / 2);
      const y = Y(obstacle.center[1] + obstacle.size[1] / 2);
      const width = obstacle.size[0] * scale;
      const height = obstacle.size[1] * scale;
      const wall = Math.abs(obstacle.center[0]) >= 5 || Math.abs(obstacle.center[1]) >= 5;
      ctx.fillStyle = wall ? "#a8b699" : "#ccd6bd";
      ctx.strokeStyle = wall ? "#95a781" : "#a7b794";
      ctx.lineWidth = 0.9;
      ctx.fillRect(x, y, width, height);
      ctx.strokeRect(x, y, width, height);
      if (!wall && width > 10 && height > 10) {
        ctx.fillStyle = "#dce3d1";
        ctx.fillRect(x + 3, y + 3, width - 6, Math.min(3, height - 6));
        ctx.strokeStyle = "#b8c6a6";
        ctx.lineWidth = 0.6;
        if (width > height) {
          for (let offset = 12; offset < width - 4; offset += 14) {
            ctx.beginPath(); ctx.moveTo(x + offset, y + 3); ctx.lineTo(x + offset, y + height - 3); ctx.stroke();
          }
        } else {
          for (let offset = 12; offset < height - 4; offset += 14) {
            ctx.beginPath(); ctx.moveTo(x + 3, y + offset); ctx.lineTo(x + width - 3, y + offset); ctx.stroke();
          }
        }
      }
    });

    function line(points, stroke, width, dash) {
      if (!points.length) return;
      ctx.beginPath();
      points.forEach((point, index) => {
        if (index === 0) ctx.moveTo(X(point[0]), Y(point[1]));
        else ctx.lineTo(X(point[0]), Y(point[1]));
      });
      ctx.strokeStyle = stroke;
      ctx.lineWidth = width;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.setLineDash(dash || []);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    let reference = active.initial_path;
    active.route_updates.forEach((update) => { if (update.t <= state.time) reference = update.path; });
    line(reference, "#7f9274", 1.2, [4, 4]);

    const sample = sampleAt(active.trajectory, state.time);
    const executed = [data.scene.start];
    for (const point of active.trajectory) {
      if (point[0] > state.time) break;
      executed.push([point[1], point[2]]);
    }
    executed.push([sample[1], sample[2]]);
    line(executed, colors[state.method] + "22", 8);
    line(executed, colors[state.method], 2.6);

    (data.moving[String(state.seed)] || []).forEach((cart, index) => {
      const position = cartPosition(cart.waypoints, state.time);
      const x = X(position[0]);
      const y = Y(position[1]);
      const radius = cart.radius * scale;
      ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fillStyle = index ? "#d7a95e" : "#ce8b4c";
      ctx.fill(); ctx.strokeStyle = "#a8743c"; ctx.lineWidth = 1; ctx.stroke();
      ctx.beginPath(); ctx.arc(x - radius * 0.13, y - radius * 0.17, radius * 0.46, 0, Math.PI * 2);
      ctx.fillStyle = "#ffffff33"; ctx.fill();
      ctx.fillStyle = "#fff8e6";
      ctx.font = Math.max(7, radius * 0.75) + "px monospace";
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(index === 0 ? "A" : "B", x, y);
    });

    const startX = X(data.scene.start[0]);
    const startY = Y(data.scene.start[1]);
    ctx.beginPath(); ctx.arc(startX, startY, 4, 0, Math.PI * 2);
    ctx.fillStyle = "#fcfff7"; ctx.fill(); ctx.strokeStyle = "#4d845c"; ctx.lineWidth = 1.8; ctx.stroke();
    ctx.textAlign = "left"; ctx.textBaseline = "middle"; ctx.font = "8px monospace"; ctx.fillStyle = "#55774d";
    ctx.fillText("START", startX + 9, startY + 12);
    const goalX = X(data.scene.goal[0]);
    const goalY = Y(data.scene.goal[1]);
    ctx.strokeStyle = "#b16d3e"; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(goalX, goalY, 7, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.arc(goalX, goalY, 3, 0, Math.PI * 2); ctx.fillStyle = "#c3834b"; ctx.fill();
    ctx.fillStyle = "#9b704e"; ctx.textAlign = "right"; ctx.fillText("GOAL", goalX - 11, goalY - 12);

    const bodyX = X(sample[1]);
    const bodyY = Y(sample[2]);
    const radius = data.scene.robot_radius * scale;
    const endedInCollision = state.time >= active.finish_time && active.status === "collision";
    ctx.save();
    ctx.translate(bodyX, bodyY);
    ctx.rotate(-sample[3]);
    ctx.beginPath(); ctx.arc(0, 0, radius + 4, 0, Math.PI * 2); ctx.fillStyle = colors[state.method] + "25"; ctx.fill();
    ctx.beginPath(); ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fillStyle = endedInCollision ? "#b44f39" : colors[state.method]; ctx.fill();
    ctx.strokeStyle = "#fffef9"; ctx.lineWidth = 1.4; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(radius * 0.65, 0); ctx.lineTo(-radius * 0.35, -radius * 0.42); ctx.lineTo(-radius * 0.35, radius * 0.42); ctx.closePath(); ctx.fillStyle = "#fffef9"; ctx.fill();
    ctx.restore();
    ctx.restore();

    ctx.strokeStyle = "#c4cfb7"; ctx.lineWidth = 0.8; ctx.strokeRect(left, top, side, side);
    ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillStyle = "#94a185"; ctx.font = "7px monospace";
    [-4, -2, 0, 2, 4].forEach((value) => { ctx.fillText(String(value), X(value), top + side + 12); });
    ctx.textAlign = "right";
    [-4, -2, 0, 2, 4].forEach((value) => { ctx.fillText(String(value), left - 7, Y(value)); });
    ctx.textAlign = "left"; ctx.fillText("m", left + side + 4, top + side + 12);
  }

  function resizeCanvas() {
    const bounds = canvas.getBoundingClientRect();
    canvasWidth = Math.max(100, bounds.width);
    canvasHeight = Math.max(100, bounds.height);
    const ratio = Math.min(window.devicePixelRatio || 1, 3);
    canvas.width = Math.round(canvasWidth * ratio);
    canvas.height = Math.round(canvasHeight * ratio);
    if (ctx) ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }

  function updateClock() {
    const active = run();
    $("timeline").value = String(state.time);
    $("clock").textContent = fixed(state.time, 2) + " / " + fixed(active.finish_time, 2) + " s";
    $("timeline").setAttribute("aria-valuetext", fixed(state.time, 2) + " 秒，共 " + fixed(active.finish_time, 2) + " 秒");
    const finished = state.time >= active.finish_time;
    const descriptions = { success: "无碰撞到达", collision: "碰撞结束", timeout: "超时结束" };
    $("run-status").textContent = finished ? descriptions[active.status] : (state.playing ? "正在回放" : "已暂停");
    $("run-status").classList.toggle("collision", finished && active.status === "collision");
  }

  function setPlaying(value) {
    state.playing = value;
    state.previousFrame = null;
    $("play-icon").textContent = value ? "Ⅱ" : "▶";
    $("play-toggle").setAttribute("aria-pressed", String(value));
    $("play-toggle").setAttribute("aria-label", value ? "暂停轨迹" : "播放轨迹");
    updateClock();
    if (frameRequest !== null) cancelAnimationFrame(frameRequest);
    frameRequest = value ? requestAnimationFrame(animate) : null;
  }

  function animate(timestamp) {
    if (!state.playing) return;
    if (state.previousFrame !== null) {
      state.time = Math.min(run().finish_time, state.time + Math.min((timestamp - state.previousFrame) / 1000, 0.1) * state.speed);
    }
    state.previousFrame = timestamp;
    updateClock();
    draw();
    if (state.time >= run().finish_time) setPlaying(false);
    else frameRequest = requestAnimationFrame(animate);
  }

  function chooseRun() {
    setPlaying(false);
    const active = run();
    state.time = Math.min(state.time, active.finish_time);
    $("timeline").max = String(active.finish_time);
    $("seed-select").value = String(state.seed);
    $("selected-method").textContent = label(state.method);
    $("run-time-label").textContent = active.status === "success" ? "本次到达耗时" : "本次终止时间";
    $("run-time").textContent = fixed(active.finish_time, 2) + " s";
    $("run-clearance").textContent = fixed(active.min_clearance, 4) + " m";
    $("run-replans").textContent = active.replan_count + " 次";
    canvas.setAttribute("aria-label", label(state.method) + "，seed " + state.seed + "，10 米乘 10 米仓库中的已归档二维轨迹。运行结果：" + (active.status === "success" ? "无碰撞到达" : active.status === "collision" ? "碰撞" : "超时") + "。");
    document.querySelectorAll("[data-method]").forEach((button) => {
      button.setAttribute("aria-pressed", String(button.dataset.method === state.method));
    });
    document.querySelector(".legend-line.executed").style.borderTopColor = colors[state.method];
    updateClock();
    draw();
  }

  $("play-toggle").addEventListener("click", () => {
    if (!state.playing && state.time >= run().finish_time) state.time = 0;
    setPlaying(!state.playing);
  });
  $("reset-playback").addEventListener("click", () => {
    setPlaying(false); state.time = 0; updateClock(); draw();
  });
  $("timeline").addEventListener("input", (event) => {
    const requestedTime = Number(event.target.value);
    setPlaying(false); state.time = Math.min(requestedTime, run().finish_time); updateClock(); draw();
  });
  $("speed-select").addEventListener("change", (event) => { state.speed = Number(event.target.value); });
  $("seed-select").addEventListener("change", (event) => { state.seed = Number(event.target.value); chooseRun(); });
  document.querySelectorAll("[data-method]").forEach((button) => {
    button.addEventListener("click", () => { state.method = button.dataset.method; chooseRun(); });
  });
  document.addEventListener("visibilitychange", () => { if (document.hidden) setPlaying(false); });

  const groups = {
    global: ["astar", "rrt_star", "voronoi", "voronoi_qp"],
    local: ["astar", "apf", "dwa"],
    webots: ["astar", "rrt_star", "voronoi", "apf", "dwa", "voronoi_qp"]
  };
  function resultRows() {
    return groups[state.group].map((id) => {
      if (state.group === "webots") {
        const native = data.webots.runs.find((item) => item.method === id);
        return { method: id, success: native.status === "success" ? 1 : 0, n: 1,
          mean_time: native.finish_time, mean_path: native.path_length, mean_clearance: native.min_clearance };
      }
      return summaryById.get(id);
    });
  }
  function renderResults() {
    const native = state.group === "webots";
    const local = state.group === "local";
    const rows = resultRows();
    const descriptions = {
      global: "四种全局规划器共用方向跟踪器，每种方法五个 seed。",
      local: "APF、DWA 与公共方向跟踪器共用 A* 引导；A* 的运行同时作为跟踪器对照。",
      webots: "Webots R2025a 的 seed 0 原生验证，六种方法分别运行一次。"
    };
    $("group-description").textContent = descriptions[state.group];
    $("dataset-label").textContent = native ? "NATIVE WEBOTS / SEED 0" : "2D KINEMATIC / 5 SEEDS";
    $("time-header").textContent = native ? "耗时" : "平均耗时";
    $("length-header").textContent = native ? "实际行程" : "平均行程";
    $("clearance-header").textContent = native ? "最低净空" : "平均最低净空";
    $("table-note").textContent = native
      ? "原生结果独立展示；归档源提交 " + data.webots.source_commit.slice(0, 7) + "，非本次页面构建重跑。"
      : "耗时、行程仅统计成功运行；净空统计全部五次。";
    const body = $("results-body");
    body.replaceChildren();
    rows.forEach((row) => {
      const tr = element("tr", row.method === "voronoi_qp" ? "featured" : "");
      tr.append(element("td", "", label(row.method, local)));
      const successCell = element("td");
      successCell.append(element("span", "success-count" + (row.success < row.n ? " partial" : ""), row.success + "/" + row.n));
      tr.append(successCell);
      tr.append(element("td", "", fixed(row.mean_time, native ? 3 : 2) + " s"));
      tr.append(element("td", "", fixed(row.mean_path, 3) + " m"));
      tr.append(element("td", "", fixed(row.mean_clearance, 4) + " m"));
      body.append(tr);
    });
    if (native) {
      $("evidence-text").textContent = "这是独立的原生验证，不与五个 seed 的二维统计混算。已归档结果包含完整日志、传感器几何检查和对应 CI 记录。";
      $("evidence-link").href = data.webots.ci_url;
      $("evidence-link").textContent = "查看归档 CI ↗";
    } else {
      $("evidence-text").textContent = "A* 在 seed 4 与侧向驶入的装卸车发生一次碰撞，已计入 4/5 的成功率。其余方法在五次二维实验中均无碰撞到达。";
      $("evidence-link").href = "results/collision_case.png";
      $("evidence-link").textContent = "查看记录 ↗";
    }
    const settings = {
      clearance: { key: "mean_clearance", title: native ? "最低实际净空" : "平均最低实际净空", unit: "m", digits: 4,
        subtitle: native ? "单次原生运行 · 越高表示距障碍更远" : "五次运行均值 · 越高表示距障碍更远" },
      time: { key: "mean_time", title: native ? "运行耗时" : "成功平均耗时", unit: "s", digits: native ? 3 : 2,
        subtitle: native ? "单次原生运行 · 越低表示到达更快" : "仅成功运行 · 越低表示到达更快" },
      length: { key: "mean_path", title: native ? "实际行程" : "成功平均行程", unit: "m", digits: 3,
        subtitle: native ? "单次原生运行 · 越低表示行程更短" : "仅成功运行 · 越低表示行程更短" }
    };
    const metric = settings[state.metric];
    $("chart-title").textContent = metric.title;
    $("chart-subtitle").textContent = metric.subtitle;
    const bars = $("metric-bars");
    bars.replaceChildren();
    const max = Math.max(...rows.map((row) => row[metric.key] || 0), 0.0001);
    rows.forEach((row) => {
      const wrapper = element("div", "metric-row" + (row.method === "voronoi_qp" ? " featured" : ""));
      const heading = element("div", "bar-label", label(row.method, local));
      const value = element("span", "bar-value", fixed(row[metric.key], metric.digits) + " " + metric.unit);
      const track = element("div", "bar-track");
      track.setAttribute("aria-hidden", "true");
      const fill = element("div", "bar-fill");
      fill.style.width = Math.max(0, (row[metric.key] || 0) / max * 100) + "%";
      track.append(fill); wrapper.append(heading, track, value); bars.append(wrapper);
    });
  }

  const tabs = Array.from(document.querySelectorAll("[data-group]"));
  function chooseGroup(button) {
    state.group = button.dataset.group;
    tabs.forEach((tab) => {
      const selected = tab === button;
      tab.setAttribute("aria-selected", String(selected));
      tab.tabIndex = selected ? 0 : -1;
    });
    $("results-panel").setAttribute("aria-labelledby", button.id);
    renderResults();
  }
  tabs.forEach((button, index) => {
    button.addEventListener("click", () => chooseGroup(button));
    button.addEventListener("keydown", (event) => {
      let next = index;
      if (event.key === "ArrowRight") next = (index + 1) % tabs.length;
      else if (event.key === "ArrowLeft") next = (index + tabs.length - 1) % tabs.length;
      else if (event.key === "Home") next = 0;
      else if (event.key === "End") next = tabs.length - 1;
      else return;
      event.preventDefault(); tabs[next].focus(); chooseGroup(tabs[next]);
    });
  });
  $("metric-select").addEventListener("change", (event) => {
    state.metric = event.target.value; renderResults();
  });

  const baseline = summaryById.get("voronoi");
  const featured = summaryById.get("voronoi_qp");
  function percentage(id, number) {
    $(id).replaceChildren(document.createTextNode(fixed(number, 1)), element("span", "", "%"));
  }
  percentage("clearance-gain", (featured.mean_clearance / baseline.mean_clearance - 1) * 100);
  percentage("curvature-gain", (1 - featured.mean_curvature / baseline.mean_curvature) * 100);
  percentage("accel-gain", (1 - featured.mean_angular_accel / baseline.mean_angular_accel) * 100);
  $("extra-time").textContent = fixed(featured.mean_time - baseline.mean_time, 2) + " s";
  $("extra-distance").textContent = fixed(featured.mean_path - baseline.mean_path, 3) + " m";

  resizeCanvas();
  chooseRun();
  renderResults();
  if (window.ResizeObserver) new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  else window.addEventListener("resize", resizeCanvas);
  if (!ctx) {
    $("data-error").hidden = false;
    $("data-error").textContent = "当前浏览器无法显示交互画布。实验表格、视频和原始数据仍可使用。";
  }
}());
