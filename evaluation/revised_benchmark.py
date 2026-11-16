"""Run the declared paired experiment matrix; never load stale files as new runs."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import SCENES, make_scene
from navigation.simulation import run

METHODS = ("fixed_dwa", "periodic_dwa", "replan_dwa")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(5)))
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=list(SCENES))
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    args = parser.parse_args()
    out = ROOT / "results" / "revised"
    out.mkdir(parents=True, exist_ok=True)
    rows, runs = [], []
    for name in args.scenes:
        for seed in args.seeds:
            for method in args.methods:
                result = run(make_scene(name, seed), method)
                filename = f"{name}_{method}_seed{seed}.json"
                (out / filename).write_text(json.dumps(result, indent=2), encoding="utf-8")
                row = {k: v for k, v in result.items() if k not in
                       ("trajectory", "path_history", "initial_path", "config")}
                rows.append(row)
                runs.append(filename)
                print(f"{name} seed={seed} {method}: {result['status']} "
                      f"t={result['finish_time']:.1f}s replans={result['replans']}", flush=True)
    with (out / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    (out / "manifest.json").write_text(json.dumps({"backend": "kinematic_2d",
        "seeds": args.seeds, "scenes": args.scenes, "methods": args.methods, "runs": runs,
        "time_step_s": .10, "collision_check_step_s": .01}, indent=2), encoding="utf-8")
    archive = {name: json.loads((out / name).read_text(encoding="utf-8")) for name in runs}
    with gzip.open(out / "raw_runs.json.gz", "wt", encoding="utf-8") as f:
        json.dump(archive, f, separators=(",", ":"))


if __name__ == "__main__":
    main()
