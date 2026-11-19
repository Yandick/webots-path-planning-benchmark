"""Space-time A* with explicit wait actions, planned from observed obstacle tracks.

Time is quantised into ticks. Besides the eight grid moves, staying in place for
one tick is a first-class action, so the planner can decide *when* to cross a
predicted moving-obstacle corridor instead of braking reactively at the edge.
Predictions come only from lidar-derived tracks (position + velocity + radius),
never from scripted actor states.
"""
import heapq
import math

import numpy as np

TICK = .12
EXPANSION_CAP = 150_000
# A stopped cart may simply be dwelling; its dwell length is unpredictable from
# lidar. Plan around it conservatively for a bounded window only, and let the
# reactive replan layer take over if it is still there afterwards.
DWELL_CAP = 5.


def _blocked_masks(grid, t0, ticks, tracks, clearance_extra):
    """Per-tick boolean masks of inflated static cells plus predicted track discs."""
    masks = [grid.inflated_grid.astype(bool) for _ in range(ticks)]
    for track in tracks:
        origin = track.predict(t0)
        radius = grid.robot_radius+track.radius+clearance_extra
        stopped = np.linalg.norm(track.velocity) < .05
        span = min(ticks, int(DWELL_CAP/TICK)+1) if stopped else ticks
        for k in range(span):
            center = origin+k*TICK*track.velocity
            _stamp_disc(masks[k], grid, center, radius)
    return masks


def _stamp_disc(mask, grid, center, radius):
    x0 = max(int((center[0]-radius-grid.min_x)/grid.resolution), 0)
    x1 = min(int((center[0]+radius-grid.min_x)/grid.resolution)+1, grid.width)
    y0 = max(int((center[1]-radius-grid.min_y)/grid.resolution), 0)
    y1 = min(int((center[1]+radius-grid.min_y)/grid.resolution)+1, grid.height)
    if x0 >= x1 or y0 >= y1:
        return
    xs = grid.min_x+(np.arange(x0, x1)+.5)*grid.resolution
    ys = grid.min_y+(np.arange(y0, y1)+.5)*grid.resolution
    hit = (xs[None, :]-center[0])**2+(ys[:, None]-center[1])**2 <= radius**2
    mask[y0:y1, x0:x1] |= hit


def nominal_schedule(points, t0, speed=.45):
    """Reach waypoint i at t0 + arclength_i / speed, the no-wait reference."""
    points = np.asarray(points)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    return t0+distance/speed


def effective_speed(grid, speed):
    """Fastest sustained speed representable by whole-tick cell moves."""
    ticks = max(1, math.ceil(grid.resolution/speed/TICK))
    return grid.resolution/ticks/TICK


def timed_astar(grid, start, goal, t0, tracks=(), clearance_extra=.10,
                speed=.45, horizon=None):
    """Plan a collision-free (space, time) route.

    Returns (points, times): waypoints and the absolute time each is scheduled
    to be reached, or None when no schedule within the horizon exists.
    """
    source, target = grid.world_to_grid(*start), grid.world_to_grid(*goal)
    if grid.is_occupied(*source) or grid.is_occupied(*target):
        return None
    if horizon is None:
        distance = math.hypot(goal[0]-start[0], goal[1]-start[1])
        horizon = min(90., max(25., 1.6*distance/effective_speed(grid, speed)))
    ticks = int(math.ceil(horizon/TICK))+1
    masks = _blocked_masks(grid, t0, ticks, tracks, clearance_extra)
    queue = [(0., (source, 0))]
    cost, parent, closed = {(source, 0): 0}, {}, set()
    expanded = 0
    while queue:
        _, (p, k) = heapq.heappop(queue)
        if (p, k) in closed:
            continue
        if p == target:
            states = [(p, k)]
            while (p, k) != (source, 0):
                p, k = parent[(p, k)]
                states.append((p, k))
            states.reverse()
            states = _shortcut(grid, states, masks, speed)
            return _schedule(grid, states, t0)
        closed.add((p, k))
        expanded += 1
        if expanded > EXPANSION_CAP:
            return None
        if k+1 >= ticks:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1), (0, 0)):
            if dx or dy:
                distance = grid.resolution*math.sqrt(2) if dx and dy else grid.resolution
                nk = k+max(1, math.ceil(distance/speed/TICK))
            else:
                nk = k+1
            if nk >= ticks:
                continue
            q = p[0]+dx, p[1]+dy
            if masks[nk][q[1], q[0]]:
                continue
            if dx and dy and (masks[nk][p[1], q[0]] or masks[nk][q[1], p[0]]):
                continue
            state = (q, nk)
            if state in closed or cost.get(state, math.inf) <= nk:
                continue
            cost[state] = nk
            parent[state] = (p, k)
            h = math.hypot(q[0]-target[0], q[1]-target[1])*grid.resolution/speed/TICK
            heapq.heappush(queue, (nk+h, state))
    return None


def _shortcut(grid, states, masks, speed):
    """Line-of-sight simplification preserving both static inflation and the
    per-tick dynamic masks along each segment's own traversal window."""
    kept = [states[0]]
    i = 0
    while i < len(states)-1:
        j = len(states)-1
        while j > i+1 and not _segment_safe(grid, states[i], states[j], masks):
            j -= 1
        kept.append(states[j])
        i = j
    return kept


def _segment_safe(grid, a, b, masks):
    (pa, ka), (pb, kb) = a, b
    if not grid.is_line_collision_free(grid.grid_to_world(*pa), grid.grid_to_world(*pb)):
        return False
    if pb == pa:
        return True
    span = max(kb-ka, 1)
    for k in range(ka+1, kb+1):
        f = min((k-ka)/span, 1.)
        qx, qy = pa[0]+(pb[0]-pa[0])*f, pa[1]+(pb[1]-pa[1])*f
        if masks[min(k, len(masks)-1)][int(qy), int(qx)]:
            return False
    return True


def _schedule(grid, states, t0):
    points, times = [], []
    for p, k in states:
        point = grid.grid_to_world(*p)
        if points and np.allclose(point, points[-1]):
            continue
        points.append(point)
        times.append(t0+k*TICK)
    return np.asarray(points), np.asarray(times)


def timed_conflict(points, times, t0, tracks, robot_radius=.18, clearance_extra=.10, horizon=30.):
    """True when a scheduled route violates a track's constant-velocity prediction."""
    horizon_ticks = np.arange(t0, t0+horizon, .12)
    for track in tracks:
        clearance = robot_radius+track.radius+clearance_extra
        other = track.predict(horizon_ticks[0])+np.outer(horizon_ticks-t0, track.velocity)
        samples = np.column_stack([np.interp(horizon_ticks, times, points[:, k],
                                             right=points[-1, k]) for k in range(2)])
        if (np.linalg.norm(samples-other, axis=1)-clearance <= 0).any():
            return True
    return False
