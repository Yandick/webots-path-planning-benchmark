"""Train the PPO yield policy in the 2D simulator and checkpoint the best weights.

On-policy PPO: no dataset is collected beforehand. Each update rolls out a batch
of control steps across several simulator environments whose cart schedules are
randomised beyond the benchmark's five seeds, then evaluates the deterministic
policy on the exact benchmark conditions and keeps the best checkpoint.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from navigation.ppo import PPOPolicy, gae
from navigation.ppo_yield import OBS_DIM
import navigation.ppo_yield as ppo_yield
from navigation.rl_env import YieldEnv
from navigation.scenarios import SCENES, make_scene
from navigation.simulation import run

EVAL_CONDITIONS = [(name, seed) for name in SCENES for seed in range(5)]


def evaluate(policy):
    """Deterministic benchmark run; returns (arrivals, mean finish time, mean yield time)."""
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/"weights.npz"
        policy.save(path)
        ppo_yield.ACTIVE_POLICY = None
        ppo_yield.WEIGHTS_PATH = path
        results = [run(make_scene(name, seed), "astar_ppo") for name, seed in EVAL_CONDITIONS]
        arrivals = sum(r["success"] for r in results)
        times = [r["finish_time"] for r in results if r["success"]]
        yield_time = [r["yield_time_s"] for r in results]
        return arrivals, float(np.mean(times)), float(np.mean(yield_time))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--updates", type=int, default=60)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--rollout", type=int, default=512)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-every", type=int, default=5)
    parser.add_argument("--output", type=Path, default=ROOT/"assets"/"ppo_yield_weights.npz")
    parser.add_argument("--log", type=Path, default=ROOT/"results_extension"/"ppo_training_log.json")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    policy = PPOPolicy(OBS_DIM, 3, seed=args.seed)
    ppo_yield.ACTIVE_POLICY = policy
    envs = [YieldEnv() for _ in range(args.envs)]
    observations = [env.reset(rng) for env in envs]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    best = (-1, float("inf"))
    history = []
    begin = time.perf_counter()
    for update in range(1, args.updates+1):
        keys = ("obs", "action", "logp", "value", "reward", "done")
        batch = {k: np.zeros((args.rollout, args.envs)) for k in keys}
        batch["obs"] = np.zeros((args.rollout, args.envs, OBS_DIM))
        for t in range(args.rollout):
            for i, env in enumerate(envs):
                guard = env.nav.crossing_guard
                batch["obs"][t, i] = observations[i]
                obs, reward, done, _ = env.step()
                batch["action"][t, i] = guard.last_action
                batch["logp"][t, i] = guard.last_logp
                batch["value"][t, i] = guard.last_value
                batch["reward"][t, i] = reward
                batch["done"][t, i] = done
                observations[i] = env.reset(rng) if done else obs
        _, _, bootstrap = policy.act(np.asarray(observations), deterministic=True)
        adv, ret = gae(batch["reward"], np.vstack([batch["value"], bootstrap[None, :]]),
                       batch["done"])
        stats = policy.learn({**batch, "adv": adv.reshape(-1), "ret": ret.reshape(-1)},
                             rng, lr=args.lr)
        record = {"update": update,
                  "mean_reward": float(batch["reward"].mean()),
                  "yield_rate": float((batch["action"] == 2).mean()),
                  "slow_rate": float((batch["action"] == 1).mean()),
                  **stats}
        if update % args.eval_every == 0 or update == args.updates:
            arrivals, mean_time, yield_time = evaluate(policy)
            record.update({"eval_arrivals": arrivals, "eval_mean_time": mean_time,
                           "eval_yield_time": yield_time})
            marker = ""
            if (arrivals, -mean_time) > best:
                best = (arrivals, -mean_time)
                policy.save(args.output)
                marker = " *best*"
            print(f"update {update:3d} reward={record['mean_reward']:+.4f} "
                  f"yield%={record['yield_rate']:.2f} kl={stats['kl']:.4f} "
                  f"eval={arrivals}/15 t={mean_time:.1f}s y={yield_time:.1f}s{marker}",
                  flush=True)
            ppo_yield.ACTIVE_POLICY = policy
        else:
            print(f"update {update:3d} reward={record['mean_reward']:+.4f} "
                  f"yield%={record['yield_rate']:.2f} kl={stats['kl']:.4f}", flush=True)
        history.append(record)
        args.log.write_text(json.dumps(history, indent=1), encoding="utf-8")
    if best[0] < 0:
        policy.save(args.output)
    print(f"done in {time.perf_counter()-begin:.0f}s, best {best[0]}/15, "
          f"weights -> {args.output}")


if __name__ == "__main__":
    main()
