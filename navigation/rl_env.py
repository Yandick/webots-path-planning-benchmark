"""Step-level training environment driving a Navigator with a PPOGuard.

Mirrors navigation.simulation.run's integration and swept collision checks,
with per-step rewards and wider cart-schedule randomization than the benchmark
uses, so the policy cannot memorise the five evaluation seeds.
"""
import math

import numpy as np

from .core import Navigator, angle
from .scenarios import make_scene, clearance, validate_scene
from .simulation import scan

SCENES = ("turtlebot3_stage2", "pointmaze_u", "pointmaze_medium")


class YieldEnv:
    def __init__(self, seeds=(0, 1, 2, 3, 4, 5, 6, 7), time_shift=2.0,
                 method="astar_ppo", dt=.10):
        self.seeds = tuple(seeds)
        self.time_shift = time_shift
        self.method = method
        self.dt = dt
        self.obs_dim = 20

    def reset(self, rng):
        scene_name = SCENES[int(rng.integers(len(SCENES)))]
        scene = make_scene(scene_name, self.seeds[int(rng.integers(len(self.seeds)))])
        waypoints = scene.moving[0].waypoints
        earliest = min(p[0] for p in waypoints[1:-1])
        low = max(-self.time_shift, -(earliest-.1))
        shift = float(rng.uniform(low, self.time_shift))
        for point in waypoints[1:-1]:
            point[0] += shift
        validate_scene(scene)
        self.scene = scene
        self.nav = Navigator(scene, self.method)
        self.pose = np.array([*scene.start, math.atan2(scene.goal[1]-scene.start[1],
                                                       scene.goal[0]-scene.start[0])])
        self.velocity = (0., 0.)
        self.t = 0.
        self.steps = 0
        self.goal_distance = float(np.linalg.norm(self.pose[:2]-scene.goal))
        return self.nav.crossing_guard.last_obs.copy()

    def step(self):
        points = scan(self.scene, self.pose, self.t)
        v, w = self.nav.step(self.pose, self.velocity, points, self.t, self.dt)
        old = self.pose.copy()
        collision = False
        gap = math.inf
        substeps = max(1, int(math.ceil(self.dt/.01)))
        for j in range(substeps):
            small = self.dt/substeps
            self.pose[2] = angle(self.pose[2]+w*small)
            self.pose[:2] += small*v*np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
            gap = clearance(self.scene, self.pose[:2], self.t+(j+1)*small)
            if gap <= 0:
                collision = True
                break
        self.t += self.dt
        self.steps += 1
        self.velocity = v, w
        distance = float(np.linalg.norm(self.pose[:2]-self.scene.goal))
        reward = .1*(self.goal_distance-distance)-.005
        self.goal_distance = distance
        if self.nav.crossing_guard.holding:
            reward -= .01
        done, terminal = False, 0.
        if collision:
            done, terminal = True, -10.
        elif distance < .25:
            done, terminal = True, 1.
        elif self.t >= self.scene.duration or self.steps >= int(self.scene.duration/self.dt):
            done = True
        reward += terminal
        info = {"collision": collision, "success": terminal == 1.}
        self.last_v = v
        return self.nav.crossing_guard.last_obs.copy(), reward, done, info
