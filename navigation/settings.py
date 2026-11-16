"""Supported methods and shared controller configuration."""
import json
import os
from pathlib import Path

METHOD_LABELS = {
    "astar": "A*",
    "rrt_star": "RRT*",
    "voronoi": "Voronoi",
    "apf": "APF",
    "dwa": "DWA",
    "replan_dwa": "Event-triggered replanning",
}
METHODS = tuple(METHOD_LABELS)
DEFAULT_METHOD = "replan_dwa"
ROOT = Path(__file__).resolve().parents[1]


def resolve_method(environ=None, config_path=None):
    env = os.environ if environ is None else environ
    config = json.loads(Path(config_path or ROOT / "config.json").read_text(encoding="utf-8"))
    method = env.get("PLANNER_ALGO", config.get("algorithm", DEFAULT_METHOD))
    if method not in METHODS:
        raise ValueError(f"Unknown algorithm {method!r}; choose from {METHODS}")
    return method
