"""Run all planners on the selected public layouts with paired obstacle schedules."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.classical import voronoi_route
from navigation.core import Config
from navigation.scenarios import MAP_FILE, SCENES, SCENE_INFO, make_scene
from navigation.settings import (EXTENSION_METHODS, METHODS, METHOD_REFERENCES,
                                 EXPERIMENT_GROUPS, NAVIGATION_PROTOCOL)
from navigation.simulation import run


def simulate(case):
    name, seed, method = case
    return run(make_scene(name, seed), method)


def write_archive(path, data):
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(data, stream, separators=(",", ":"), allow_nan=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=list(SCENES))
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(5)))
    parser.add_argument("--methods", nargs="+", choices=list(METHODS)+list(EXTENSION_METHODS),
                        default=list(METHODS))
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    if args.workers < 1 or any(len(items) != len(set(items)) for items in (args.scenes, args.seeds, args.methods)):
        parser.error("Use unique scenes, methods and seeds, and at least one worker.")
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [(name, seed, method) for name in args.scenes for seed in args.seeds for method in args.methods]
    rows, archive = [], {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for (name, seed, method), result in zip(cases, pool.map(simulate, cases)):
            archive[f"{name}_{method}_seed{seed}.json"] = result
            rows.append({k: v for k, v in result.items() if k not in
                         ("trajectory", "initial_path", "config", "route_updates", "replan_attempts", "yield_events")})
            print(f"{name} seed={seed} {method}: {result['status']} {result['finish_time']:.2f}s", flush=True)
    with TemporaryDirectory(dir=args.output) as directory:
        stage = Path(directory)
        with (stage / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        raw = stage / "raw_runs.json.gz"
        write_archive(raw, archive)
        manifest = {
            "backend": "kinematic_2d", "scenes": args.scenes, "seeds": args.seeds,
            "methods": args.methods, "runs": list(archive),
            "raw_runs_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
            "map_definitions_sha256": hashlib.sha256(MAP_FILE.read_bytes().replace(b"\r\n", b"\n")).hexdigest(),
            "map_sources": {name: SCENE_INFO[name]["source"] for name in args.scenes},
            "time_step_s": .10, "collision_check_step_s": .01,
            "time_budget_s": make_scene().duration, "config": asdict(Config()),
            "comparison_groups": EXPERIMENT_GROUPS, "navigation_protocol": NAVIGATION_PROTOCOL,
            "method_references": {m: METHOD_REFERENCES[m] for m in args.methods},
        }
        if "voronoi_qp" in args.methods and 0 in args.seeds:
            with patch("navigation.core.voronoi_qp_path", voronoi_route):
                reference = {name: run(make_scene(name, 0), "voronoi_qp") for name in args.scenes}
            write_archive(stage / "smoothing_ablation.json.gz", reference)
        for path in stage.iterdir():
            path.replace(args.output / path.name)
        # Changing the manifest last lets the website detect mismatched files.
        final = stage / "manifest.json"
        final.write_text(json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
        final.replace(args.output / final.name)


if __name__ == "__main__":
    main()
