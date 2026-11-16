"""Classical global planners and a goal-directed artificial potential field."""
import heapq
import math

import numpy as np
from scipy.spatial import Voronoi


def shortcut(grid, points):
    out, i = [points[0]], 0
    while i < len(points)-1:
        j = len(points)-1
        while j > i+1 and not grid.is_line_collision_free(points[i], points[j], step=.04):
            j -= 1
        out.append(points[j])
        i = j
    return np.asarray(out)


def rrt_star(grid, start, goal, seed=0, iterations=1800):
    rng = np.random.default_rng(seed)
    nodes = [np.asarray(start, dtype=float)]
    parents, costs, children = [-1], [0.], [set()]
    goal = np.asarray(goal, dtype=float)
    for _ in range(iterations):
        sample = goal if rng.random() < .15 else rng.uniform(
            [grid.min_x+.2, grid.min_y+.2], [grid.max_x-.2, grid.max_y-.2])
        points = np.asarray(nodes)
        distances = np.linalg.norm(points-sample, axis=1)
        nearest = int(np.argmin(distances))
        delta = sample-nodes[nearest]
        point = nodes[nearest]+delta*min(1., .4/max(np.linalg.norm(delta), 1e-9))
        if not grid.is_line_collision_free(nodes[nearest], point, step=.04):
            continue
        ds = np.linalg.norm(points-point, axis=1)
        if ds.min() < 1e-6:
            continue
        radius = max(.4, min(1.2, 5.6*math.sqrt(math.log(len(nodes)+1)/(len(nodes)+1))))
        near = np.flatnonzero(ds <= radius)
        parent, cost = nearest, costs[nearest]+ds[nearest]
        for i in near:
            candidate = costs[i]+ds[i]
            if candidate < cost and grid.is_line_collision_free(nodes[i], point, step=.04):
                parent, cost = int(i), candidate
        index = len(nodes)
        nodes.append(point)
        parents.append(parent)
        costs.append(cost)
        children.append(set())
        children[parent].add(index)
        for i in near:
            i = int(i)
            if i == 0 or i == parent:
                continue
            new_cost = cost+ds[i]
            if new_cost+1e-9 < costs[i] and grid.is_line_collision_free(point, nodes[i], step=.04):
                children[parents[i]].remove(i)
                parents[i] = index
                children[index].add(i)
                adjustment = new_cost-costs[i]
                stack = [i]
                while stack:
                    j = stack.pop()
                    costs[j] += adjustment
                    stack.extend(children[j])
    candidates = [i for i, p in enumerate(nodes)
                  if np.linalg.norm(p-goal) <= .45 and grid.is_line_collision_free(p, goal, step=.04)]
    if not candidates:
        return None
    index = min(candidates, key=lambda i: costs[i]+np.linalg.norm(nodes[i]-goal))
    path = [goal]
    while index != -1:
        path.append(nodes[index])
        index = parents[index]
    path.reverse()
    return shortcut(grid, path)


def voronoi_route(grid, start, goal):
    boundary = []
    for obs in grid.obstacles:
        cx, cy = obs.params["center"]
        if obs.kind == "box":
            w, h = obs.params["size"]
            for x in np.linspace(cx-w/2, cx+w/2, max(4, int(w/.12)+1)):
                boundary.extend([(x, cy-h/2), (x, cy+h/2)])
            for y in np.linspace(cy-h/2, cy+h/2, max(4, int(h/.12)+1)):
                boundary.extend([(cx-w/2, y), (cx+w/2, y)])
        else:
            r = obs.params["radius"]
            boundary.extend((cx+r*math.cos(t), cy+r*math.sin(t))
                            for t in np.linspace(0, 2*math.pi, max(16, int(2*math.pi*r/.12)), endpoint=False))
    diagram = Voronoi(np.unique(np.asarray(boundary), axis=0))
    positions = {i: p for i, p in enumerate(diagram.vertices) if grid.is_collision_free_point(*p)}
    graph = {i: [] for i in positions}
    for a, b in diagram.ridge_vertices:
        if a in positions and b in positions and grid.is_line_collision_free(positions[a], positions[b], step=.04):
            distance = float(np.linalg.norm(positions[a]-positions[b]))
            graph[a].append((b, distance))
            graph[b].append((a, distance))
    for index, point in ((-1, np.asarray(start)), (-2, np.asarray(goal))):
        graph[index] = []
        candidates = sorted(positions, key=lambda i: np.linalg.norm(positions[i]-point))
        for i in candidates:
            if grid.is_line_collision_free(point, positions[i], step=.04):
                distance = float(np.linalg.norm(point-positions[i]))
                graph[index].append((i, distance))
                graph[i].append((index, distance))
                if len(graph[index]) >= 12:
                    break
    positions.update({-1: np.asarray(start), -2: np.asarray(goal)})
    queue, costs, parents = [(0., -1)], {-1: 0.}, {}
    while queue:
        cost, node = heapq.heappop(queue)
        if node == -2:
            path = [positions[node]]
            while node != -1:
                node = parents[node]
                path.append(positions[node])
            return np.asarray(path[::-1])
        if cost > costs[node]:
            continue
        for other, distance in graph[node]:
            new_cost = cost+distance
            if new_cost < costs.get(other, math.inf):
                costs[other], parents[other] = new_cost, node
                heapq.heappush(queue, (new_cost, other))
    return None


def voronoi_path(grid, start, goal):
    path = voronoi_route(grid, start, goal)
    return None if path is None else shortcut(grid, path)


def voronoi_visibility_path(grid, start, goal):
    """Niu et al. (2019), Sections 4.5-4.6: visibility graph on Voronoi path.

    DOI: 10.1017/S0373463318001005. A Python reproduction using the shared
    inflated robot configuration-space grid for every visibility test.
    """
    route = voronoi_route(grid, start, goal)
    if route is None:
        return None
    graph = [[] for _ in route]
    for i in range(len(route)):
        for j in range(i+1, len(route)):
            if grid.is_line_collision_free(route[i], route[j]):
                distance = float(np.linalg.norm(route[j]-route[i]))
                graph[i].append((j, distance))
                graph[j].append((i, distance))
    target = len(route)-1
    queue, costs, parents = [(0., 0)], {0: 0.}, {}
    while queue:
        cost, node = heapq.heappop(queue)
        if node == target:
            path = [route[node]]
            while node != 0:
                node = parents[node]
                path.append(route[node])
            return np.asarray(path[::-1])
        if cost > costs[node]:
            continue
        for other, distance in graph[node]:
            new_cost = cost+distance
            if new_cost < costs.get(other, math.inf):
                costs[other], parents[other] = new_cost, node
                heapq.heappush(queue, (new_cost, other))
    return None


def apf_command(pose, velocity, goal, points, dt, config):
    p = np.asarray(pose[:2])
    delta = np.asarray(goal)-p
    distance = np.linalg.norm(delta)
    force = 1.2*delta/max(distance, 1e-9)*min(distance, 2.)
    if len(points):
        away = p-points
        center_dist = np.linalg.norm(away, axis=1)
        gap = np.maximum(center_dist-config.radius, .03)
        active = gap < 1.2
        magnitude = .8*(1/gap[active]-1/1.2)/gap[active]**2
        directions = away[active]/np.maximum(center_dist[active, None], 1e-9)
        force += np.sum(magnitude[:, None]*directions, axis=0)*(2*np.pi/len(points))
    desired = math.atan2(force[1], force[0])
    error = (desired-pose[2]+math.pi) % (2*math.pi)-math.pi
    v = config.max_speed*max(0., math.cos(error))*min(1., distance/.8)
    w = float(np.clip(2*error, -config.max_omega, config.max_omega))
    return v, w
