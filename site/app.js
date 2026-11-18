/* Both views share a clock and read unmodified trajectories from the archive. */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const data = window.BENCHMARK_DATA;
  const menu = document.querySelector(".menu-button");
  const navigation = $("main-nav");
  function showMenu(open) {
    menu.setAttribute("aria-expanded", String(open));
    navigation.classList.toggle("open", open);
  }
  menu.hidden = false;
  document.documentElement.classList.add("has-menu");
  menu.addEventListener("click", () => showMenu(menu.getAttribute("aria-expanded") !== "true"));
  navigation.addEventListener("click", () => showMenu(false));
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && navigation.classList.contains("open")) {
      showMenu(false);
      menu.focus();
    }
  });

  if (!data?.pairs?.length || !data.scenes?.length || !data.runs?.length) {
    $("load-error").hidden = false;
    return;
  }
  const sides = ["left", "right"];
  const methods = new Map(data.methods.map((item) => [item.id, item.label.replace(" controller", "")]));
  const maps = new Map(data.scenes.map((item) => [item.id, item]));
  const runIndex = new Map(data.runs.map((item) => [[item.scene, item.method, item.seed].join(":"), item]));
  const summaries = new Map(data.summary.map((item) => [[item.scene, item.method].join(":"), item]));
  const revised = new Set(data.pairs.map((item) => item.improved));
  const outcomes = {success: "到达", collision: "碰撞", timeout: "超时", planning_failed: "规划失败"};
  const state = {
    scene: data.scenes[0].id, seed: data.seeds[0], left: data.pairs[0].base, right: data.pairs[0].improved,
    view: "iso", backend: "2d", time: 0, speed: 2, playing: false
  };
  const screens = {};
  let animation = null;
  let previousFrame = null;

  function scene() { return maps.get(state.scene); }
  function run(method, seed = state.seed, map = state.scene) {
    return runIndex.get([map, method, seed].join(":"));
  }
  function create(tag, className, content) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (content !== undefined) node.textContent = content;
    return node;
  }
  function measure(n, unit, digits) {
    return Number.isFinite(n) ? n.toFixed(digits) + " " + unit : "—";
  }
  function mean(values) {
    const numbers = values.filter(Number.isFinite);
    return numbers.length ? numbers.reduce((total, n) => total + n, 0) / numbers.length : null;
  }
  function isYielding(record) {
    return state.time < record.finish_time && record.yield_events?.some((event) => event.start <= state.time && state.time < event.end);
  }

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

  function draw(side) {
    const {ctx, width: canvasWidth, height: canvasHeight} = screens[side];
    if (!ctx || canvasWidth < 2 || canvasHeight < 2) return;
    const map = scene();
    const { start, goal, robot_radius: robotRadius } = map;
    const [minX, maxX, minY, maxY] = map.bounds;
    const spanX = maxX - minX;
    const spanY = maxY - minY;
    const active = run(state[side]);
    const iso = state.view === "iso";
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const rise = 0.55;
    const scale = iso
      ? Math.min((canvasWidth - 18) / (0.8 * spanX + 0.6 * spanY + 0.6),
        (canvasHeight - 22) / ((0.6 * spanX + 0.8 * spanY) * rise + 0.95))
      : Math.min((canvasWidth - 36) / (spanX + 0.3), (canvasHeight - 30) / (spanY + 0.3));
    const centerV = canvasHeight / 2 + (iso ? scale * 0.07 : 0);
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
      const robotColor = collision ? "#ff6956" : isYielding(active) ? "#ffcf68" : "#ffab70";
      ring(robotX, robotY, robotRadius + 0.13, robotColor, 1.3);
      disc(robotX, robotY, robotRadius, 0.14, robotColor, "#b75935");
      const arrow = [[0.78, 0], [-0.38, 0.5], [-0.38, -0.5]].map(([dx, dy]) =>
        [robotX + robotRadius * (dx * Math.cos(heading) - dy * Math.sin(heading)),
          robotY + robotRadius * (dx * Math.sin(heading) + dy * Math.cos(heading)), 0.14]);
      face(arrow, "#3c392b");
    }});
    objects.sort((a, b) => a.depth - b.depth).forEach((object) => object.paint());
    if (canvasWidth >= 210) pin(start, "起点", -1);
    pin(goal, "终点", 1);

    // Keep the position visible when a wall occludes the robot in the angled view.
    if (iso && canvasWidth >= 210) {
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

  function resize() {
    for (const side of sides) {
      const screen = screens[side];
      const rect = screen.canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 3);
      screen.width = rect.width;
      screen.height = rect.height;
      const width = Math.max(1, Math.round(rect.width * ratio));
      const height = Math.max(1, Math.round(rect.height * ratio));
      if (screen.canvas.width !== width) screen.canvas.width = width;
      if (screen.canvas.height !== height) screen.canvas.height = height;
      if (screen.ctx) screen.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw(side);
    }
  }
  function totalTime() { return Math.max(...sides.map((side) => run(state[side]).finish_time)); }

  function paintFrame() {
    const duration = totalTime();
    $("timeline").value = state.time;
    $("timeline").style.setProperty("--played", (duration ? state.time / duration * 100 : 0) + "%");
    $("timeline").setAttribute("aria-valuetext", state.time.toFixed(2) + " 秒，共 " + duration.toFixed(2) + " 秒");
    $("clock").textContent = state.time.toFixed(2) + " / " + duration.toFixed(2) + " s";
    for (const side of sides) {
      const record = run(state[side]);
      const finished = state.time >= record.finish_time;
      const yielding = isYielding(record);
      const status = $(side + "-status");
      status.textContent = finished ? outcomes[record.status] : yielding ? "让行中" : state.time === 0 ? "起点" : state.playing ? "行驶中" : "已暂停";
      status.dataset.state = finished ? record.status : yielding ? "yield" : "active";
      draw(side);
    }
  }

  function stop() {
    state.playing = false;
    previousFrame = null;
    if (animation !== null) cancelAnimationFrame(animation);
    animation = null;
    const button = $("play-toggle");
    button.setAttribute("aria-pressed", "false");
    button.setAttribute("aria-label", "播放同步轨迹");
  }
  function animate(timestamp) {
    if (!state.playing) return;
    if (previousFrame !== null) {
      state.time = Math.min(totalTime(), state.time + Math.min((timestamp - previousFrame) / 1000, .15) * state.speed);
    }
    previousFrame = timestamp;
    if (state.time >= totalTime()) stop();
    paintFrame();
    if (state.playing) animation = requestAnimationFrame(animate);
  }
  function togglePlayback() {
    if (state.playing) {
      stop();
      paintFrame();
      return;
    }
    if (!totalTime()) return;
    if (state.time >= totalTime()) state.time = 0;
    state.playing = true;
    previousFrame = null;
    $("play-toggle").setAttribute("aria-pressed", "true");
    $("play-toggle").setAttribute("aria-label", "暂停同步轨迹");
    paintFrame();
    animation = requestAnimationFrame(animate);
  }
  function seek(t) {
    stop();
    state.time = Math.max(0, Math.min(totalTime(), t));
    paintFrame();
  }

  function renderEvents() {
    const markers = [];
    for (const side of sides) {
      const item = run(state[side]);
      const channel = side === "left" ? "左侧" : "右侧";
      const first = item.yield_events?.[0];
      if (first) {
        markers.push([first.start, channel + "首次让行"]);
        if (first.end < item.finish_time) markers.push([first.end, channel + "让行结束"]);
      }
      if (item.status === "collision") markers.push([item.finish_time, channel + "碰撞"]);
    }
    markers.sort((a, b) => a[0] - b[0]);
    $("events").replaceChildren(...markers.map(([t, name]) => {
      const button = create("button", "", name + " " + t.toFixed(2) + " s");
      button.type = "button";
      button.addEventListener("click", () => seek(t));
      return button;
    }));
    $("event-bar").hidden = !markers.length;
  }

  function renderReplay() {
    stop();
    state.time = 0;
    $("timeline").max = totalTime();
    $("scene-select").value = state.scene;
    $("seed-select").value = state.seed;
    for (const side of sides) {
      const method = state[side];
      const item = run(method);
      $(side + "-method").value = method;
      const endpoint = measure(item.finish_time, "s", 2) + " " + outcomes[item.status];
      const stats = [endpoint, "行程 " + measure(item.path_length, "m", 2)];
      if (revised.has(method)) stats.push("让行 " + measure(item.yield_time_s, "s", 2));
      $(side + "-outcome").textContent = stats.join(" · ");
      screens[side].canvas.setAttribute("aria-label", scene().label + "，种子 " + state.seed + "，" + methods.get(method) + "。" + stats.join("，") + "。");
    }
    for (const button of $("pair-switch").children) {
      button.setAttribute("aria-pressed", String(button.dataset.base === state.left && button.dataset.improved === state.right));
    }
    renderEvents();
    paintFrame();
  }

  function populateAtlas() {
    const cards = data.scenes.map((map) => {
      const card = create("article", "map-card");
      const select = create("button", "map-pick");
      select.type = "button";
      select.dataset.scene = map.id;
      const routes = map.moving[String(data.seeds[0])].map((cart) => ({path: cart.waypoints.map((point) => point.slice(1)), style: "map-cart-track"}));
      select.append(planView(map, routes), create("span", "map-title", map.label));
      select.setAttribute("aria-label", "选择" + map.label);
      select.addEventListener("click", () => changeScene(map.id));
      const attribution = create("a", "attribution", map.source_label);
      attribution.href = map.source;
      card.append(select, create("p", "", map.description), attribution);
      return card;
    });
    $("map-atlas").replaceChildren(...cards);
  }

  function renderSmoothing() {
    const routes = [
      {path: run("voronoi_qp").initial_path, style: "map-smooth"},
      {path: run("voronoi").initial_path, style: "map-raw"}
    ];
    $("smoothing-map").replaceChildren(planView(scene(), routes));
    $("smoothing-caption").textContent = scene().label + " · 种子 " + state.seed + "，两种方法在出发时规划的路线。";
  }
  function backendRuns() {
    return state.backend === "webots" ? data.webots.runs : data.runs;
  }
  function displayRecord() {
    const method = $("record-method").value;
    const records = backendRuns().filter((item) => item.scene === state.scene && item.method === method).sort((a,b) => a.seed - b.seed);
    $("records-body").replaceChildren(...records.map((item) => {
      const row = create("tr");
      [item.seed, outcomes[item.status], measure(item.finish_time, "s", 2), measure(item.path_length, "m", 2),
        measure(item.min_clearance, "m", 3), item.replan_count,
        revised.has(method) ? measure(item.yield_time_s, "s", 2) : "—"].forEach((value) => row.append(create("td", "", value)));
      return row;
    }));
  }
  function renderPairedResults(records) {
    const key = (item) => item.scene + ":" + item.seed;
    const notes = data.pairs.map(({base, improved}) => {
      const left = records.filter((r) => r.method === base);
      const right = new Map(records.filter((r) => r.method === improved).map((r) => [key(r), r]));
      const matched = left.filter((r) => r.status === "success" && right.get(key(r))?.status === "success");
      const delta = mean(matched.map((r) => right.get(key(r)).finish_time - r.finish_time));
      const note = create("article");
      note.append(create("h3", "", methods.get(base) + " · 三张地图合计"));
      const successes = left.filter((r) => r.status === "success").length;
      const revisedSuccesses = Array.from(right.values()).filter((r) => r.status === "success").length;
      const counts = create("p", "paired-counts");
      counts.append("原版 ", create("strong", "", successes + "/" + left.length), " 次到达，提前让行版 ",
        create("strong", "", revisedSuccesses + "/" + right.size), " 次到达。");
      note.append(counts);
      if (matched.length) {
        const change = Math.abs(delta) < .005 ? "平均耗时相同" : "改进版平均" + (delta > 0 ? "多用 " : "少用 ") + measure(Math.abs(delta), "s", 2);
        note.append(create("p", "", "在双方都到达的 " + matched.length + " 组对照中，" + change + "。"));
      } else {
        note.append(create("p", "", "本组没有双方都到达的运行。"));
      }
      return note;
    });
    $("paired-notes").replaceChildren(...notes);
  }
  function renderOverview() {
    $("hero-total").textContent = data.scenes.length + " 张地图 · " + methods.size + " 种方法 · " + data.runs.length + " 次二维运行";
    for (const {base, improved} of data.pairs) {
      const row = create("tr");
      const heading = create("th", "", methods.get(base));
      heading.scope = "row";
      row.append(heading);
      for (const id of [base, improved]) {
        const records = data.runs.filter((item) => item.method === id);
        row.append(create("td", "", records.filter((item) => item.status === "success").length + "/" + records.length));
      }
      $("hero-pairs").append(row);
    }
  }
  function renderResults() {
    const native = state.backend === "webots";
    const allRecords = backendRuns();
    const records = allRecords.filter((item) => item.scene === state.scene);
    $("result-scene").value = state.scene;
    document.querySelectorAll("[data-backend]").forEach((button) => {
      button.setAttribute("aria-pressed", String(button.dataset.backend === state.backend));
    });
    const n = records.filter((item) => item.status === "success").length;
    $("results-caption").textContent = scene().label + "，" + (native ? "Webots 原生" : "二维仿真") + " " + records.length + " 次，" + n + " 次到达。";
    const body = $("results-body");
    body.replaceChildren();
    for (const [id, label] of methods) {
      const selected = records.filter((item) => item.method === id);
      if (!selected.length) continue;
      const ok = selected.filter((item) => item.status === "success");
      const summary = native ? null : summaries.get([state.scene,id].join(":"));
      const clearances = selected.map((item) => item.min_clearance);
      const values = [
        ok.length + "/" + selected.length, selected.filter((item) => item.status === "collision").length,
        selected.filter((item) => item.status === "timeout").length,
        measure(summary ? summary.mean_time : mean(ok.map((item) => item.finish_time)), "s", 2),
        measure(summary ? summary.mean_path : mean(ok.map((item) => item.path_length)), "m", 2),
        measure(summary ? summary.mean_clearance : mean(clearances), "m", 3),
        revised.has(id) ? measure(mean(selected.map((item) => item.yield_time_s)), "s", 2) : "—"
      ];
      const row = create("tr", revised.has(id) ? "revised" : "");
      const heading = create("th", "", label);
      heading.scope = "row";
      row.append(heading, ...values.map((value) => create("td", "", value)));
      body.append(row);
    }
    const failedPlans = records.filter((item) => item.status === "planning_failed").length;
    const scope = native ? "每种方法使用种子 " + data.webots.seed + " 运行一次。" : "每种方法运行 " + data.seeds.length + " 次。";
    const yielding = native && !records.some((item) => Number.isFinite(item.yield_time_s)) ? "原生记录中，提前让行时间未单列。" : "";
    $("result-note").textContent = scope + "耗时和行程只统计成功到达的运行。最小距离从车身边缘量起，与让行时间均对全部运行取平均。" + yielding + (failedPlans ? "另有 " + failedPlans + " 次初始规划失败。" : "");
    renderPairedResults(allRecords);
    displayRecord();
  }

  function changeScene(id) {
    state.scene = id;
    const nativeButton = document.querySelector('[data-backend="webots"]');
    nativeButton.hidden = !data.webots?.runs.some((item) => item.scene === id);
    if (nativeButton.hidden) state.backend = "2d";
    document.querySelectorAll("[data-scene]").forEach((button) => {
      button.setAttribute("aria-pressed", String(button.dataset.scene === id));
    });
    $("world-file").textContent = "worlds/" + id + ".wbt";
    renderReplay();
    renderResults();
    renderSmoothing();
  }

  // Choose an actual paired encounter for the opening example. Aggregates include every seed.
  function initialCase() {
    let score = 0;
    for (const pair of data.pairs) {
      for (const map of data.scenes) {
        for (const seed of data.seeds) {
          const a = run(pair.base, seed, map.id), b = run(pair.improved, seed, map.id);
          const priority = b.yield_events?.length && b.status === "success" ? (a.status === "collision" ? 3 : a.status !== "success" ? 2 : 1) : 0;
          if (priority > score) {
            score = priority;
            Object.assign(state, {scene: map.id, seed, left: pair.base, right: pair.improved});
          }
        }
      }
    }
  }

  function attachRecordedExample() {
    const map = "pointmaze_u", seed = 1;
    const pair = data.pairs.find((item) => item.base === "rrt_star");
    if (!pair) return;
    const original = run(pair.base, seed, map);
    const improved = run(pair.improved, seed, map);
    if (original?.status !== "collision" || improved?.status !== "success") return;
    const tail = original.trajectory.filter((sample) => sample[0] >= original.finish_time - .3);
    const stopped = tail.length > 1 && tail.every((sample) => Math.hypot(sample[1] - tail[0][1], sample[2] - tail[0][2]) < .005);
    $("rrt-example-text").textContent = maps.get(map).label + "的种子 " + seed + " 中，RRT* 原版" +
      (stopped ? "已经停住，仍在 " : "在 ") + measure(original.finish_time, "s", 2) + " 与小车相撞。提前让行版在 " +
      measure(improved.finish_time, "s", 2) + " 到达终点。";
    $("rrt-example").hidden = false;
    $("rrt-example-link").addEventListener("click", (event) => {
      event.preventDefault();
      Object.assign(state, {seed, left: pair.base, right: pair.improved});
      changeScene(map);
      const player = document.querySelector(".console");
      player.focus({preventScroll: true});
      player.scrollIntoView({block: "center"});
    });
  }

  function attachControls() {
    for (const map of data.scenes) {
      $("scene-select").add(new Option(map.label, map.id));
      $("result-scene").add(new Option(map.label, map.id));
    }
    for (const seed of data.seeds) $("seed-select").add(new Option(seed, seed));
    for (const [id, label] of methods) {
      for (const control of ["left-method","right-method","record-method"]) $(control).add(new Option(label, id));
    }
    for (const pair of data.pairs) {
      const button = create("button", "", methods.get(pair.base) + " 对照");
      button.type = "button";
      button.dataset.base = pair.base;
      button.dataset.improved = pair.improved;
      button.addEventListener("click", () => {
        state.left = pair.base;
        state.right = pair.improved;
        renderReplay();
      });
      $("pair-switch").append(button);
    }
    for (const id of ["scene-select","result-scene"]) $(id).addEventListener("change", (event) => changeScene(event.target.value));
    $("seed-select").addEventListener("change", (event) => {
      state.seed = Number(event.target.value);
      renderReplay();
      renderSmoothing();
    });
    for (const side of sides) {
      $(side + "-method").addEventListener("change", (event) => { state[side] = event.target.value; renderReplay(); });
    }
    $("view-select").addEventListener("change", (event) => { state.view = event.target.value; paintFrame(); });
    $("speed-select").addEventListener("change", (event) => { state.speed = Number(event.target.value); });
    $("timeline").addEventListener("input", (event) => seek(Number(event.target.value)));
    $("play-toggle").addEventListener("click", togglePlayback);
    $("reset-playback").addEventListener("click", () => seek(0));
    const consoleBox = document.querySelector(".console");
    consoleBox.tabIndex = 0;
    consoleBox.addEventListener("keydown", (event) => {
      if (event.target !== consoleBox) return;
      if (event.key === " ") { event.preventDefault(); togglePlayback(); }
      if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
        event.preventDefault();
        seek(state.time + (event.key === "ArrowRight" ? 1 : -1));
      }
    });
    document.addEventListener("visibilitychange", () => { if (document.hidden) { stop(); paintFrame(); } });
    document.querySelectorAll("[data-backend]").forEach((button) => {
      button.addEventListener("click", () => { state.backend = button.dataset.backend; renderResults(); });
    });
    $("record-method").addEventListener("change", displayRecord);
  }

  for (const side of sides) {
    const canvas = $(side + "-canvas");
    screens[side] = {canvas, ctx: canvas.getContext("2d"), width: 0, height: 0};
  }
  if (!screens.left.ctx || !screens.right.ctx) {
    document.documentElement.classList.add("no-canvas");
    $("load-error").textContent = "当前浏览器无法绘制轨迹，图中保留了完整路线。";
    $("load-error").hidden = false;
  }
  document.documentElement.classList.add("has-data");
  if (data.webots?.version) $("simulator-version").textContent = "Webots " + data.webots.version;
  if (data.webots?.ci_url) {
    $("native-source").href = data.webots.ci_url;
    $("native-source").textContent = "本次 Webots 运行记录";
  }
  renderOverview();
  initialCase();
  attachControls();
  attachRecordedExample();
  populateAtlas();
  changeScene(state.scene);
  resize();
  if (window.ResizeObserver) {
    const observer = new ResizeObserver(resize);
    observer.observe($("screens"));
  } else {
    window.addEventListener("resize", resize);
  }
}());
