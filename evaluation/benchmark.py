"""Run paired schedules on the single coursework map using the explicit 2D backend."""
import argparse
import csv
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.core import Config
from navigation.scenarios import DEFAULT_SCENE, make_scene
from navigation.settings import METHODS, METHOD_REFERENCES, EXPERIMENT_GROUPS, NAVIGATION_PROTOCOL
from navigation.simulation import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(5)))
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    rows, archive = [], {}
    for seed in args.seeds:
        for method in args.methods:
            result = run(make_scene(seed=seed), method)
            filename = f"{DEFAULT_SCENE}_{method}_seed{seed}.json"
            archive[filename] = result
            rows.append({k: v for k, v in result.items() if k not in
                         ("trajectory", "initial_path", "config", "route_updates", "replan_attempts")})
            print(f"seed={seed} {method}: {result['status']} "
                  f"t={result['finish_time']:.1f}s", flush=True)
    with (out / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    manifest = {"backend": "kinematic_2d", "scene": DEFAULT_SCENE,
                "seeds": args.seeds, "methods": args.methods, "runs": list(archive),
                "time_step_s": .10, "collision_check_step_s": .01,
                "time_budget_s": make_scene().duration, "config": asdict(Config()),
                "comparison_groups": EXPERIMENT_GROUPS, "navigation_protocol": NAVIGATION_PROTOCOL,
                "method_references": {m: METHOD_REFERENCES[m] for m in args.methods if m in METHOD_REFERENCES}}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    with gzip.open(out / "raw_runs.json.gz", "wt", encoding="utf-8") as f:
        json.dump(archive, f, separators=(",", ":"))


if __name__ == "__main__":
    main()
