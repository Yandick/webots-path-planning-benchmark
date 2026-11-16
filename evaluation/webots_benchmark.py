"""Run generated worlds with Webots, requiring fresh navigator AND supervisor logs."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import SCENES, make_scene, validate_scene
from scripts.generate_worlds import world_text
from navigation.settings import METHODS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0])
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--timeout", type=int, default=360)
    args = parser.parse_args()
    executable = os.environ.get("WEBOTS_BINARY") or shutil.which("webots") or shutil.which("webots.exe")
    if not executable:
        raise SystemExit("Webots not found. Set WEBOTS_BINARY to the native executable; 2D runs use evaluation/benchmark.py.")
    out = ROOT / "results" / "webots"
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"backend": "webots", "runs": []}
    for name in SCENES:
        for seed in args.seeds:
            scene = make_scene(name, seed); validate_scene(scene)
            world = ROOT / "worlds" / "benchmark_generated.wbt"
            world.write_text(world_text(scene), encoding="utf-8")
            for method in args.methods:
                tag = f"{name}_{method}_seed{seed}_{uuid.uuid4().hex[:8]}"
                env = os.environ.copy()
                env.update(PLANNER_ALGO=method, BENCHMARK_MODE="1", RUN_ID=tag, RUN_OUTPUT=str(out))
                command = [executable, "--batch", "--mode=fast", "--stdout", "--stderr", str(world)]
                if os.name!="nt" and not env.get("DISPLAY"):
                    xvfb = shutil.which("xvfb-run")
                    if not xvfb:
                        raise SystemExit("Headless Linux needs xvfb-run and rendering enabled for lidar.")
                    command = [xvfb, "-a"]+command
                try:
                    proc = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                          stderr=subprocess.STDOUT, timeout=args.timeout)
                except subprocess.TimeoutExpired as error:
                    (out / f"{tag}.log").write_bytes(error.stdout or b"")
                    raise RuntimeError(f"Webots infrastructure timeout: {tag}") from error
                (out / f"{tag}.log").write_bytes(proc.stdout)
                files = [out / f"{tag}_{suffix}.json" for suffix in ("navigator", "supervisor")]
                if not all(p.exists() for p in files):
                    raise RuntimeError(f"Fresh controller logs missing for {tag}; inspect {tag}.log")
                navigator, truth = [json.loads(p.read_text(encoding="utf-8")) for p in files]
                for result in (navigator, truth):
                    if result["algorithm"]!=method or result["scene_sha256"]!=scene.signature():
                        raise RuntimeError("Controller algorithm/scene mismatch")
                result = {**navigator, **truth, "control_trajectory": navigator["trajectory"]}
                path = out / f"{tag}.json"
                path.write_text(json.dumps(result, indent=2), encoding="utf-8")
                manifest["runs"].append(path.name)
                print(f"{tag}: {truth['status']} t={truth['finish_time']:.2f}s", flush=True)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
