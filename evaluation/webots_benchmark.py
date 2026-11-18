"""Run generated worlds with Webots, requiring fresh navigator AND supervisor logs."""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import SCENES, make_scene, validate_scene
from scripts.generate_worlds import world_text
from navigation.settings import METHODS


@contextmanager
def native_world(scene):
    content = world_text(scene)
    registered = ROOT / "worlds" / f"{scene.name}.wbt"
    if registered.is_file() and registered.read_text(encoding="utf-8") == content:
        yield registered
        return
    with tempfile.NamedTemporaryFile(mode="w", dir=registered.parent, prefix="benchmark_generated_",
                                     suffix=".wbt", encoding="utf-8", delete=False) as stream:
        stream.write(content)
        generated = Path(stream.name)
    try:
        yield generated
    finally:
        generated.unlink(missing_ok=True)


def version_of(executable):
    roots = list(Path(executable).resolve().parents)
    if os.environ.get("WEBOTS_HOME"):
        roots.insert(0, Path(os.environ["WEBOTS_HOME"]))
    for root in roots:
        version = root / "resources" / "version.txt"
        if version.is_file():
            return version.read_text(encoding="utf-8").strip()
    raise RuntimeError("Could not read Webots resources/version.txt; set WEBOTS_HOME to the installation directory.")


def provenance():
    repository, run_id = os.getenv("GITHUB_REPOSITORY"), os.getenv("GITHUB_RUN_ID")
    if repository and run_id and os.getenv("GITHUB_ACTIONS") == "true":
        return {"source_commit": os.getenv("GITHUB_SHA"),
                "ci_url": f"{os.getenv('GITHUB_SERVER_URL', 'https://github.com')}/{repository}/actions/runs/{run_id}"}
    commit = None
    if shutil.which("git"):
        relevant = ("navigation", "controllers", "assets", "worlds", "scripts/generate_worlds.py",
                    "evaluation/webots_benchmark.py")
        status = subprocess.run(["git", "status", "--porcelain", "--", *relevant],
                                cwd=ROOT, capture_output=True)
        if status.returncode == 0 and not status.stdout.strip():
            head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True)
            if head.returncode == 0:
                commit = head.stdout.strip()
    return {"source_commit": commit, "ci_url": None}


def store_manifest(path, manifest):
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", delete=False, dir=path.parent) as stream:
        json.dump(manifest, stream, indent=2)
        temporary = Path(stream.name)
    temporary.replace(path)


def run_method(executable, world, scene, method, out, timeout):
    tag = f"{scene.name}_{method}_seed{scene.seed}_{uuid.uuid4().hex[:8]}"
    env = {**os.environ, "PLANNER_ALGO": method, "BENCHMARK_MODE": "1", "RUN_ID": tag, "RUN_OUTPUT": str(out)}
    command = [executable, "--batch", "--minimize", "--mode=fast", "--stdout", "--stderr", str(world)]
    if os.name != "nt" and not env.get("DISPLAY"):
        xvfb = shutil.which("xvfb-run")
        if not xvfb:
            raise SystemExit("Headless Linux needs xvfb-run and rendering enabled for lidar.")
        command = [xvfb, "-a", *command]
    log_path = out / f"{tag}.log"
    with log_path.open("wb") as log:
        try:
            process = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(f"Webots infrastructure timeout; inspect {log_path}") from error
    files = [out / f"{tag}_{suffix}.json" for suffix in ("navigator", "supervisor")]
    if not all(path.is_file() for path in files):
        raise RuntimeError(f"Missing controller logs (Webots exit {process.returncode}); inspect {log_path}")
    navigator, truth = [json.loads(path.read_text(encoding="utf-8")) for path in files]
    for record in (navigator, truth):
        if (record["algorithm"], record["scene"], record["seed"], record["scene_sha256"]) != (
                method, scene.name, scene.seed, scene.signature()):
            raise RuntimeError(f"Controller algorithm/scene mismatch in {tag}")
    result = {**navigator, **truth, "control_trajectory": navigator["trajectory"]}
    path = out / f"{tag}.json"
    path.write_text(json.dumps(result, allow_nan=False), encoding="utf-8")
    print(f"{scene.name} {method} seed {scene.seed}: {truth['status']} at {truth['finish_time']:.2f}s", flush=True)
    return path.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0])
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=list(SCENES))
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "webots")
    parser.add_argument("--timeout", type=int, default=360)
    args = parser.parse_args()
    executable = os.environ.get("WEBOTS_BINARY") or shutil.which("webots") or shutil.which("webots.exe")
    if not executable:
        raise SystemExit("Webots not found. Set WEBOTS_BINARY to the native executable; 2D runs use evaluation/benchmark.py.")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"backend": "webots", "webots_version": version_of(executable), **provenance(),
                "seeds": args.seeds, "scenes": args.scenes, "methods": args.methods, "runs": []}
    checkpoint = out / "manifest.json"
    store_manifest(checkpoint, manifest)
    for name in args.scenes:
        for seed in args.seeds:
            scene = make_scene(name, seed)
            validate_scene(scene)
            with native_world(scene) as world:
                for method in args.methods:
                    manifest["runs"].append(run_method(executable, world, scene, method, out, args.timeout))
                    store_manifest(checkpoint, manifest)


if __name__ == "__main__":
    main()
