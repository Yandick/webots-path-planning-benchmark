/* Archived trajectories only: this page does not simulate or alter benchmark results. */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const data = window.BENCHMARK_DATA;
  const menuButton = document.querySelector(".menu-toggle");
  const nav = $("main-nav");
  document.documentElement.classList.add("has-menu");
  menuButton.hidden = false;
  function showMenu(open) {
    menuButton.setAttribute("aria-expanded", String(open));
    menuButton.setAttribute("aria-label", open ? "收起导航" : "展开导航");
    nav.classList.toggle("open", open);
  }
  menuButton.addEventListener("click", function () {
    showMenu(this.getAttribute("aria-expanded") !== "true");
  });
  document.querySelectorAll("#main-nav a").forEach((link) => {
    link.addEventListener("click", () => showMenu(false));
  });
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && nav.classList.contains("open")) {
      showMenu(false);
      menuButton.focus();
    }
  });

  $("copy-commands").hidden = false;
  $("copy-commands").addEventListener("click", async () => {
    const code = $("quickstart-code");
    try {
      if (!navigator.clipboard || !window.isSecureContext) throw new Error("Clipboard unavailable");
      await navigator.clipboard.writeText(code.textContent);
      $("copy-feedback").textContent = "已复制命令。";
    } catch (_) {
      const range = document.createRange();
      range.selectNodeContents(code);
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(range);
      $("copy-feedback").textContent = "已选中命令，可以用系统的复制功能拷贝。";
    }
  });

  if (data?.schema_version !== 2 || !data.scenes?.length || !data.runs?.length) {
    $("data-error").hidden = false;
    return;
  }
  document.documentElement.classList.add("has-data");

  const methodById = new Map(data.methods.map((method) => [method.id, method]));
  const sceneById = new Map(data.scenes.map((scene) => [scene.id, scene]));
  const summaryByKey = new Map(data.summary.map((item) => [[item.scene, item.method].join(":"), item]));
  const runsByKey = new Map(data.runs.map((item) => [[item.scene, item.method, item.seed].join(":"), item]));
  const outcomes = { success: "到达", collision: "碰撞", timeout: "超时", planning_failed: "未找到路线" };
  const state = {
    scene: data.scenes[0].id, method: "voronoi_qp", seed: data.seeds[0], time: 0, playing: false,
    speed: 2, view: "iso", group: "global", metric: "clearance", previousFrame: null
  };
  function scene() { return sceneById.get(state.scene); }
  function summary(method) { return summaryByKey.get([state.scene, method].join(":")); }
  const canvas = $("trajectory-canvas");
  const ctx = canvas.getContext("2d");
  if (ctx) document.documentElement.classList.add("has-replay");
  const motionPreference = window.matchMedia("(prefers-reduced-motion: reduce)");
  let playbackRequested = !motionPreference.matches;
  let viewerVisible = !window.IntersectionObserver;
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
  function measured(value, unit, digits) {
    return Number.isFinite(value) ? fixed(value, digits) + " " + unit : "—";
  }
  function run(method = state.method, seed = state.seed) {
    return runsByKey.get([state.scene, method, seed].join(":"));
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
      return [time, scene().start[0] + ratio * (first[1] - scene().start[0]),
        scene().start[1] + ratio * (first[2] - scene().start[1]), first[3], first[4]];
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

  function svgNode(tag, attributes) {
    const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
    return node;
  }

  function planView(map, routes = []) {
    const [left, right, bottom, top] = map.bounds;
    const width = right - left;
    const height = top - bottom;
    const drawing = svgNode("svg", { viewBox: [left, -top, width, height].join(" "), "aria-hidden": "true" });
    drawing.append(svgNode("rect", { x: left, y: -top, width, height, class: "plan-floor" }));
    map.static.forEach((obstacle) => {
      const [x, y] = obstacle.center;
      if (obstacle.kind === "cylinder") {
        drawing.append(svgNode("circle", { cx: x, cy: -y, r: obstacle.radius, class: "plan-wall" }));
      } else {
        const [w, h] = obstacle.size;
        drawing.append(svgNode("rect", { x: x - w / 2, y: -y - h / 2, width: w, height: h, class: "plan-wall" }));
      }
    });
    routes.forEach(({ path, style }) => {
      drawing.append(svgNode("polyline", { points: path.map(([x, y]) => [x, -y].join(",")).join(" "),
        class: style, "vector-effect": "non-scaling-stroke" }));
    });
    for (const [point, name] of [[map.start, "plan-start"], [map.goal, "plan-goal"]]) {
      drawing.append(svgNode("circle", { cx: point[0], cy: -point[1], r: map.robot_radius, class: name }));
    }
    return drawing;
  }

  function corners(x, y, width, depth, z) {
    const left = x - width / 2;
    const right = x + width / 2;
    const front = y - depth / 2;
    const back = y + depth / 2;
    return [[left, front, z], [right, front, z], [right, back, z], [left, back, z]];
  }

  function draw() {
    if (!ctx || canvasWidth < 2 || canvasHeight < 2) return;
    const map = scene();
    const { start, goal, robot_radius: robotRadius } = map;
    const [minX, maxX, minY, maxY] = map.bounds;
    const spanX = maxX - minX;
    const spanY = maxY - minY;
    const active = run();
    const iso = state.view === "iso";
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const rise = 0.55;
    const scale = iso
      ? Math.min((canvasWidth - 36) / (0.8 * spanX + 0.6 * spanY + 0.6),
        (canvasHeight - 22) / ((0.6 * spanX + 0.8 * spanY) * rise + 0.95))
      : Math.min((canvasWidth - 48) / (spanX + 0.3), (canvasHeight - 30) / (spanY + 0.3));
    const centerV = canvasHeight / 2 + (iso ? scale * 0.15 : 0);
    function depth(x, y) { return 0.6 * x - 0.8 * y; }
    function project(x, y, z = 0) {
      const dx = x - centerX;
      const dy = y - centerY;
      return iso
        ? [canvasWidth / 2 + scale * (0.8 * dx + 0.6 * dy), centerV + scale * (depth(dx, dy) * rise - z * 0.835)]
        : [canvasWidth / 2 + scale * dx, centerV - scale * dy];
    }
    function trace(points) {
      ctx.beginPath();
      points.forEach((point, index) => {
        const [x, y] = project(...point);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
    }
    function face(points, color) {
      trace(points);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
    }
    function line(points, color, width, dash = []) {
      trace(points);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // The cap and two visible faces use the same footprint in both views.
    function block(x, y, w, h, base, top, colors) {
      const lid = corners(x, y, w, h, top);
      if (iso) {
        const foot = corners(x, y, w, h, base);
        face([foot[0], foot[1], lid[1], lid[0]], colors[1]);
        face([foot[1], foot[2], lid[2], lid[1]], colors[2]);
      }
      face(lid, colors[0]);
    }
    function fixedObstacle(obstacle) {
      const [x, y] = obstacle.center;
      if (obstacle.kind === "cylinder") {
        disc(x, y, obstacle.radius, 0.5, "#ded6ba", "#a5997d");
      } else {
        block(x, y, ...obstacle.size, 0, 0.28, ["#c7bd9f", "#958a70", "#847962"]);
      }
    }
    function disc(x, y, radius, height, cap, side) {
      const [u, v] = project(x, y, height);
      const bottom = project(x, y)[1];
      const rx = radius * scale;
      const ry = rx * (iso ? rise : 1);
      ctx.beginPath();
      ctx.ellipse(u, bottom, rx, ry, 0, 0, Math.PI);
      ctx.ellipse(u, v, rx, ry, 0, Math.PI, 2 * Math.PI);
      ctx.closePath();
      ctx.fillStyle = side;
      ctx.fill();
      ctx.beginPath();
      ctx.ellipse(u, v, rx, ry, 0, 0, 2 * Math.PI);
      ctx.fillStyle = cap;
      ctx.fill();
      return [u, v];
    }
    function ring(x, y, radius, color, width) {
      const [u, v] = project(x, y);
      ctx.beginPath();
      ctx.ellipse(u, v, radius * scale, radius * scale * (iso ? rise : 1), 0, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.stroke();
    }
    function pin(point, name, direction) {
      const [x, y] = project(...point);
      const tagX = Math.max(17, Math.min(canvasWidth - 17, x + direction * 23));
      const tagY = Math.max(11, Math.min(canvasHeight - 11, y + (iso ? 18 : -direction * 22)));
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(tagX, tagY);
      ctx.strokeStyle = "#dce5d2";
      ctx.lineWidth = 0.9;
      ctx.stroke();
      ctx.fillStyle = "#27312e";
      ctx.fillRect(tagX - 15, tagY - 9, 30, 18);
      ctx.fillStyle = "#f0f2e7";
      ctx.font = '11px "Microsoft YaHei", sans-serif';
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(name, tagX, tagY);
    }

    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    block(centerX, centerY, spanX + 0.24, spanY + 0.24, -0.24, 0, ["#515f59", "#3c4943", "#35423e"]);
    for (let x = Math.ceil(minX); x <= maxX; x += 1) {
      line([[x, minY], [x, maxY]], "#728077", 0.7);
    }
    for (let y = Math.ceil(minY); y <= maxY; y += 1) {
      line([[minX, y], [maxX, y]], "#728077", 0.7);
    }
    const carts = map.moving[String(state.seed)];
    carts.forEach((cart) => {
      line(cart.waypoints.map((point) => point.slice(1)), "#86bdba", 1, [1, 4]);
    });

    let reference = active.initial_path;
    for (const update of active.route_updates) {
      if (update.t <= state.time) reference = update.path;
    }
    if (reference.length > 1) line(reference, "#e0e6cf", 1.5, [4, 4]);
    const sample = sampleAt(active.trajectory, state.time);
    const [, robotX, robotY, heading] = sample;
    const executed = [start];
    for (const [t, x, y] of active.trajectory) {
      if (t > state.time) break;
      executed.push([x, y]);
    }
    executed.push([robotX, robotY]);
    line(executed, "#343a30", 5.5);
    line(executed, "#ff8954", 3.3);
    ring(...start, 0.16, "#fff1d7", 1.4);
    ring(...goal, 0.22, "#ecf2de", 1.5);
    ring(...goal, 0.08, "#ecf2de", 1.5);

    const objects = map.static.map((obstacle) => ({
      depth: depth(...obstacle.center), paint: () => fixedObstacle(obstacle)
    }));
    carts.forEach((cart) => {
      const [x, y] = cartPosition(cart.waypoints, state.time);
      objects.push({ depth: depth(x, y), paint() {
        disc(x, y, cart.radius, 0.7, "#91d2d1", "#50888b");
        line([[x - cart.radius * 0.45, y, 0.7], [x + cart.radius * 0.45, y, 0.7]], "#458284", 1.6);
      }});
    });
    objects.push({ depth: depth(robotX, robotY), paint() {
      const collision = state.time >= active.finish_time && active.status === "collision";
      const robotColor = collision ? "#ff6956" : "#ffab70";
      ring(robotX, robotY, robotRadius + 0.13, robotColor, 1.3);
      disc(robotX, robotY, robotRadius, 0.14, robotColor, "#b75935");
      const arrow = [[0.78, 0], [-0.38, 0.5], [-0.38, -0.5]].map(([dx, dy]) =>
        [robotX + robotRadius * (dx * Math.cos(heading) - dy * Math.sin(heading)),
          robotY + robotRadius * (dx * Math.sin(heading) + dy * Math.cos(heading)), 0.14]);
      face(arrow, "#3c392b");
    }});
    objects.sort((a, b) => a.depth - b.depth).forEach((object) => object.paint());
    pin(start, "起点", -1);
    pin(goal, "终点", 1);

    // Keep the position visible when a wall occludes the robot in the angled view.
    if (iso) {
      const [x, y] = project(robotX, robotY, 0.14);
      const tagY = Math.max(11, y - Math.max(21, scale * 0.94));
      ctx.beginPath();
      ctx.moveTo(x, y - 2);
      ctx.lineTo(x, tagY);
      ctx.strokeStyle = "#ffb27e";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = "#ffb27e";
      ctx.fillRect(x - 20, tagY - 8, 40, 16);
      ctx.fillStyle = "#242924";
      ctx.textAlign = "center";
      ctx.fillText("机器人", x, tagY);
    }

    if (!iso) {
      ctx.fillStyle = "#b8c5bb";
      ctx.font = "10px Consolas, monospace";
      ctx.textAlign = "center";
      for (let tick = Math.ceil(minX / 2) * 2; tick < maxX; tick += 2) {
        const [x, y] = project(tick, minY);
        ctx.fillText(String(tick), x, y + 10);
      }
      ctx.textAlign = "right";
      for (let tick = Math.ceil(minY / 2) * 2; tick < maxY; tick += 2) {
        const [x, y] = project(minX, tick);
        ctx.fillText(String(tick), x - 7, y);
      }
    }
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
    $("timeline").style.setProperty("--played", (active.finish_time ? 100 * state.time / active.finish_time : 0) + "%");
    $("clock").textContent = fixed(state.time, 2) + " / " + fixed(active.finish_time, 2) + " s";
    $("timeline").setAttribute("aria-valuetext", fixed(state.time, 2) + " 秒，共 " + fixed(active.finish_time, 2) + " 秒");
    const finished = state.time >= active.finish_time;
    const descriptions = { success: "已到达", collision: "发生碰撞", timeout: "已超时", planning_failed: "未找到路线" };
    const position = state.time === 0 ? "起点" : "已暂停";
    $("run-status").textContent = finished ? descriptions[active.status] : (state.playing ? "正在回放" : position);
    $("run-status").classList.toggle("collision", finished && active.status === "collision");
  }

  function syncPlayback() {
    const playing = Boolean(ctx && playbackRequested && viewerVisible && !document.hidden && state.time < run().finish_time);
    if (playing === state.playing) return;
    state.playing = playing;
    state.previousFrame = null;
    $("play-toggle").setAttribute("aria-pressed", String(playing));
    $("play-toggle").setAttribute("aria-label", playing ? "暂停轨迹" : "播放轨迹");
    updateClock();
    if (frameRequest !== null) cancelAnimationFrame(frameRequest);
    frameRequest = playing ? requestAnimationFrame(animate) : null;
  }

  function requestPlayback(play) {
    playbackRequested = play;
    syncPlayback();
  }

  function animate(timestamp) {
    if (!state.playing) return;
    if (state.previousFrame !== null) {
      state.time = Math.min(run().finish_time, state.time + Math.min((timestamp - state.previousFrame) / 1000, 0.1) * state.speed);
    }
    state.previousFrame = timestamp;
    updateClock();
    draw();
    if (state.time >= run().finish_time) requestPlayback(false);
    else frameRequest = requestAnimationFrame(animate);
  }

  function chooseRun() {
    const active = run();
    state.time = Math.min(state.time, active.finish_time);
    state.previousFrame = null;
    if (state.time >= active.finish_time) playbackRequested = false;
    $("timeline").max = String(active.finish_time);
    $("seed-select").value = String(state.seed);
    $("method-select").value = state.method;
    $("run-time-label").textContent = active.status === "success" ? "到达耗时" : "终止时间";
    $("run-time").textContent = fixed(active.finish_time, 2) + " s";
    $("run-clearance").textContent = fixed(active.min_clearance, 4) + " m";
    $("run-replans").textContent = active.replan_count + " 次";
    canvas.setAttribute("aria-label", scene().label + "，" + label(state.method) + "，种子 " + state.seed + "，" + outcomes[active.status] + "。");
    syncPlayback();
    updateClock();
    draw();
  }

  $("play-toggle").addEventListener("click", () => {
    if (!state.playing && state.time >= run().finish_time) state.time = 0;
    requestPlayback(!state.playing);
  });
  $("reset-playback").addEventListener("click", () => {
    state.time = 0;
    requestPlayback(false);
    updateClock();
    draw();
  });
  $("timeline").addEventListener("input", (event) => {
    const requestedTime = Number(event.target.value);
    state.time = Math.min(requestedTime, run().finish_time);
    requestPlayback(false);
    updateClock();
    draw();
  });
  $("speed-select").addEventListener("change", (event) => { state.speed = Number(event.target.value); });
  data.methods.forEach(({ id }) => $("method-select").add(new Option(label(id), id)));
  data.seeds.forEach((seed) => $("seed-select").add(new Option(String(seed), String(seed))));
  $("seed-select").addEventListener("change", (event) => { state.seed = Number(event.target.value); chooseRun(); renderSmoothing(); });
  $("method-select").addEventListener("change", (event) => { state.method = event.target.value; chooseRun(); });
  const viewButtons = document.querySelectorAll("[data-view]");
  viewButtons.forEach((button) => {
    button.addEventListener("click", () => {
      state.view = button.dataset.view;
      viewButtons.forEach((option) => option.setAttribute("aria-pressed", String(option === button)));
      draw();
    });
  });
  document.addEventListener("visibilitychange", syncPlayback);
  motionPreference.addEventListener("change", (event) => {
    if (event.matches) requestPlayback(false);
  });
  if (window.IntersectionObserver && ctx) {
    const observer = new IntersectionObserver(([entry]) => {
      viewerVisible = entry.isIntersecting;
      syncPlayback();
    });
    observer.observe(document.querySelector(".instrument"));
  }

  function collisionCase() {
    return data.runs.find((item) => item.scene === state.scene && groups[state.group].includes(item.method) && item.status === "collision");
  }
  $("replay-collision").addEventListener("click", () => {
    const incident = collisionCase();
    if (!incident) return;
    state.method = incident.method;
    state.seed = incident.seed;
    state.time = Math.max(0, incident.finish_time - 5);
    chooseRun();
    renderSmoothing();
    $("play-toggle").focus({ preventScroll: true });
    document.querySelector(".instrument").scrollIntoView({ block: "center" });
    requestPlayback(true);
  });

  const groups = {
    global: ["astar", "rrt_star", "voronoi", "voronoi_qp"],
    local: ["astar", "apf", "dwa"],
    webots: ["astar", "rrt_star", "voronoi", "apf", "dwa", "voronoi_qp"]
  };
  function renderRecords() {
    const method = $("record-method").value;
    const rows = data.seeds.map((seed) => {
      const record = run(method, seed);
      const row = element("tr");
      row.append(
        element("td", "", seed),
        element("td", record.status === "success" ? "" : "failed-run", outcomes[record.status]),
        element("td", "", fixed(record.finish_time, 2) + " s"),
        element("td", "", fixed(record.path_length, 3) + " m"),
        element("td", "", fixed(record.min_clearance, 4) + " m"),
        element("td", "", record.replan_count)
      );
      return row;
    });
    $("records-body").replaceChildren(...rows);
  }

  function setRecordMethods() {
    const native = state.group === "webots";
    $("run-records").hidden = native;
    if (native) return;
    const select = $("record-method");
    const previous = select.value;
    const methods = groups[state.group];
    select.replaceChildren(...methods.map((id) => new Option(label(id, state.group === "local"), id)));
    if (methods.includes(previous)) select.value = previous;
    renderRecords();
  }
  $("record-method").addEventListener("change", renderRecords);

  function resultRows() {
    if (state.group === "webots") {
      return data.webots.runs.filter((item) => item.scene === state.scene).map((item) => ({
        method: item.method, status: item.status, success: item.status === "success" ? 1 : 0, n: 1,
        mean_time: item.finish_time, mean_path: item.path_length, mean_clearance: item.min_clearance
      }));
    }
    return groups[state.group].map(summary);
  }

  function resultCounts(records) {
    return Object.entries(outcomes).map(([status, title]) => {
      const count = records.filter((row) => row.status === status).length;
      return count ? count + " 次" + title : "";
    }).filter(Boolean).join("，");
  }
  $("all-outcomes").textContent = "全部 " + data.runs.length + " 次二维运行中，" + resultCounts(data.runs) + "。";

  function renderResults() {
    const native = state.group === "webots";
    const local = state.group === "local";
    const rows = resultRows();
    const descriptions = {
      global: "四种规划器共用同一个方向跟踪器。",
      local: "三种控制器都用 A* 引导。公共方向跟踪器一行取自 A* 的运行记录。"
    };
    $("group-description").textContent = native
      ? scene().label + "的 Webots 原生记录，每种方法使用种子 " + data.webots.seed + " 跑一次。"
      : descriptions[state.group] + "每种方法在" + scene().label + "上跑 " + data.seeds.length + " 次。";
    $("time-header").textContent = native ? "结束时间" : "平均耗时";
    $("length-header").textContent = native ? "实际行程" : "平均行程";
    $("distance-average").textContent = native ? "" : "（" + data.seeds.length + " 次平均）";
    const incident = native ? null : collisionCase();
    $("replay-collision").hidden = !incident;
    if (incident) $("replay-collision").textContent = "重放 " + label(incident.method) + " 碰撞前 5 秒";
    if (native) {
      $("table-note").textContent = "每行是一次原生运行，耗时从出发算到运行结束。";
      if (data.webots.ci_url) {
        const link = element("a", "citation", "原生运行记录");
        link.href = data.webots.ci_url;
        $("table-note").append(" ", link);
      }
    } else {
      $("table-note").textContent = "耗时和行程只统计成功到达的运行。最小距离从车身边缘量起，再对所有运行取平均。";
    }
    const body = $("results-body");
    body.replaceChildren();
    rows.forEach((row) => {
      const tr = element("tr", row.method === "voronoi_qp" ? "featured" : "");
      tr.append(element("td", "", label(row.method, local)));
      const successCell = element("td");
      successCell.append(element("span", "success-count" + (row.success < row.n ? " partial" : ""), row.success + "/" + row.n));
      tr.append(successCell);
      tr.append(element("td", "", measured(row.mean_time, "s", native ? 3 : 2)));
      tr.append(element("td", "", measured(row.mean_path, "m", 3)));
      tr.append(element("td", "", measured(row.mean_clearance, "m", 4)));
      body.append(tr);
    });
    const selected = native ? rows : data.runs.filter((item) =>
      item.scene === state.scene && groups[state.group].includes(item.method));
    $("evidence-text").textContent = scene().label + "，本组 " + selected.length + " 次运行中，" + resultCounts(selected) + "。";
    const settings = {
      clearance: { key: "mean_clearance", title: "与障碍物的最小距离", unit: "m", digits: 4,
        subtitle: native ? "每种方法跑一次，记下车身边缘离障碍物最近时的距离。" : "每次先记下车身边缘离障碍物最近时的距离，再取平均值。" },
      time: { key: "mean_time", title: native ? "运行耗时" : "平均到达耗时", unit: "s", digits: native ? 3 : 2,
        subtitle: native ? "运行从出发到结束所用的仿真时间。" : "从出发到进入目标范围所用的仿真时间。" },
      length: { key: "mean_path", title: native ? "实际行程" : "平均实际行程", unit: "m", digits: 3,
        subtitle: "沿机器人实际走过的轨迹计算距离。" }
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
      const value = element("span", "bar-value", measured(row[metric.key], metric.unit, metric.digits));
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
    setRecordMethods();
  }
  tabs.forEach((button) => {
    button.addEventListener("click", () => chooseGroup(button));
    button.addEventListener("keydown", (event) => {
      const visible = tabs.filter((tab) => !tab.hidden);
      const index = visible.indexOf(button);
      let next = index;
      if (event.key === "ArrowRight") next = (index + 1) % visible.length;
      else if (event.key === "ArrowLeft") next = (index + visible.length - 1) % visible.length;
      else if (event.key === "Home") next = 0;
      else if (event.key === "End") next = visible.length - 1;
      else return;
      event.preventDefault(); visible[next].focus(); chooseGroup(visible[next]);
    });
  });
  $("metric-select").addEventListener("change", (event) => {
    state.metric = event.target.value; renderResults();
  });

  function renderSmoothing() {
    const baseline = summary("voronoi");
    const featured = summary("voronoi_qp");
    const rawPath = run("voronoi").initial_path;
    const smoothPath = run("voronoi_qp").initial_path;
    const drawing = planView(scene(), [
      {path: smoothPath, style: "plan-smooth-path"},
      {path: rawPath, style: "plan-raw-path"}
    ]);
    $("smoothing-map").replaceChildren(drawing);
    $("smoothing-caption").textContent = scene().label + "，种子 " + state.seed + " 的初始规划路线。";
    $("comparison-map").textContent = scene().label + "，Dev1ce's method 相对 Voronoi 的平均变化";
    for (const [id, metric] of [["clearance-gain", "mean_clearance"], ["curvature-gain", "mean_curvature"], ["accel-gain", "mean_angular_accel"]]) {
      const base = baseline[metric], smooth = featured[metric];
      const change = Number.isFinite(base) && Number.isFinite(smooth) && base > 0 ? (smooth / base - 1) * 100 : null;
      $(id).textContent = change === null ? "—" : (change > 0 ? "+" : change < 0 ? "−" : "") + fixed(Math.abs(change), 1) + "%";
    }
  }

  function selectScene(id, initial = false) {
    state.scene = id;
    state.time = 0;
    if (!initial) requestPlayback(false);
    const map = scene();
    $("result-scene").value = id;
    for (const button of $("scene-options").children) {
      button.setAttribute("aria-pressed", String(button.dataset.scene === id));
    }
    $("scene-description").textContent = map.description;
    $("scene-source").textContent = map.source_label;
    $("scene-source").href = map.source;
    $("map-caption").textContent = map.label + " · 二维回放";
    $("world-file").textContent = "worlds/" + id + ".wbt";
    const nativeTab = $("tab-webots");
    nativeTab.hidden = !data.webots?.runs.some((row) => row.scene === id);
    const currentTab = tabs.find((tab) => tab.dataset.group === state.group && !tab.hidden);
    chooseGroup(currentTab || tabs[0]);
    chooseRun();
    renderSmoothing();
  }
  data.scenes.forEach((map) => {
    const button = element("button");
    button.type = "button";
    button.dataset.scene = map.id;
    button.append(planView(map), element("span", "", map.label));
    button.addEventListener("click", () => selectScene(map.id));
    $("scene-options").append(button);
    $("result-scene").add(new Option(map.label, map.id));
  });
  $("result-scene").addEventListener("change", (event) => selectScene(event.target.value));

  resizeCanvas();
  selectScene(state.scene, true);
  if (window.ResizeObserver) new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  else window.addEventListener("resize", resizeCanvas);
  if (!ctx) {
    $("data-error").hidden = false;
    $("data-error").textContent = "当前浏览器用静态图显示三张地图，小车和机器人的完整轨迹可在下方录像中对照。";
  }
}());
