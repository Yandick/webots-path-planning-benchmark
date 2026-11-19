"""Learned yield decision: a PPO policy replaces the rule-based conflict threshold.

The guard keeps everything from CrossingGuard except the judgment of *when* to
yield: a small policy maps frame-invariant track features to {GO, SLOW, YIELD}.
A hard shield (physical surface clearance under a constant-velocity prediction)
can force YIELD but can never be overridden by the policy, so learning tunes
efficiency while the classic layer keeps the safety floor. The unsafe-stop
rule (never park inside the crossing) is likewise kept unconditionally.
"""
import os
from pathlib import Path

import numpy as np

from .yielding import CrossingGuard
from .ppo import PPOPolicy

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS_PATH = Path(os.environ.get("PPO_YIELD_WEIGHTS", ROOT/"assets"/"ppo_yield_weights.npz"))
OBS_DIM = 20
N_TRACKS = 2
GO, SLOW, YIELD = 0, 1, 2

# Set by scripts/train_ppo.py so each new Navigator shares the live policy
# under training instead of reloading weights from disk.
ACTIVE_POLICY = None


def guard_factory(config):
    if ACTIVE_POLICY is not None:
        return PPOGuard(config, policy=ACTIVE_POLICY, train=True)
    return PPOGuard(config)


def build_obs(guard, position, speed, goal_distance):
    """Frame-invariant features from the guard's track bookkeeping.

    Per track: relative position/velocity (world frame), radius, predicted
    minimum gap along the robot's route, time to that gap, velocity
    uncertainty, stopped flag. Tracks are sorted by predicted gap; padded.
    """
    feats = [min(speed/.45, 2.), min(goal_distance/15., 3.)]
    gaps = sorted(guard.last_gaps, key=lambda item: item["gap"])
    for item in gaps[:N_TRACKS]:
        rel, vel = np.asarray(item["rel"]), np.asarray(item["vel"])
        feats += [rel[0]/5., rel[1]/5., vel[0]/2., vel[1]/2., item["radius"]/.3,
                  item["gap"]/2., item["t_min"]/4., item["sigma_v"]/.3, item["stopped"]]
    while len(feats) < OBS_DIM:
        feats.append(0.)
    return np.clip(np.asarray(feats[:OBS_DIM], dtype=float), -6., 6.)


class PPOGuard(CrossingGuard):
    def __init__(self, config, policy=None, weights=None, train=False, seed=0):
        super().__init__(config)
        self.include_stopped = True
        self.policy = policy or PPOPolicy.load(weights or WEIGHTS_PATH)
        self.train = train
        self.rng = np.random.default_rng(seed)
        self.pending_action = None
        self.speed_scale = 1.
        self.last_obs = np.zeros(OBS_DIM)
        self.last_action, self.last_logp, self.last_value = GO, 0., 0.

    def should_yield(self, t, dt, position, speed, route):
        conflict, unsafe_stop = self._gaps(t, position, speed, route)
        goal_distance = float(np.linalg.norm(route[-1]-position)) if len(route) else 0.
        self.last_obs = build_obs(self, position, speed, goal_distance)
        if self.pending_action is not None:
            action = self.pending_action
            self.pending_action = None
        elif self.train:
            action, logp, value = self.policy.act(self.last_obs[None, :], rng=self.rng)
            action, self.last_logp, self.last_value = int(action[0]), float(logp[0]), float(value[0])
        else:
            action, _, _ = self.policy.act(self.last_obs[None, :], deterministic=True)
            action = int(action[0])
        self.last_action = action
        # Hard shield: constant-velocity prediction with physical-clearance floor.
        # Hard shield: constant-velocity prediction with physical-clearance floor.
        hard = any(item["gap"] < self.config.radius+item["radius"]+.02
                   for item in self.last_gaps)
        if hard:
            action = YIELD
        self.speed_scale = .5 if action == SLOW else 1.
        if unsafe_stop:
            self.hold_until = t
        elif action == YIELD:
            self.hold_until = t+.55
        hold = t < self.hold_until
        if hold and not self.holding:
            self.events.append({"start": float(t), "end": float(t)})
        if hold:
            self.events[-1]["end"] = float(t+dt)
        self.holding = hold
        return hold
