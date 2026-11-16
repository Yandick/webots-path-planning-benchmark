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
    "voronoi_qp": "Voronoi + safe QP smoothing",
}
METHODS = tuple(METHOD_LABELS)
METHOD_REFERENCES = {
    "voronoi_qp": {
        "authors": "Wen, Zhang, Bi, Liu, Yuan, Fang",
        "year": 2024,
        "title": "G2VD Planner: Efficient Motion Planning With Grid-based Generalized Voronoi Diagrams",
        "doi": "10.1109/TASE.2024.3398996",
        "reproduced_component": "Section IV-D, safe-bubble QP smoothing, Equations (5)-(8)",
        "adaptation": "geometric Voronoi reference; smoothness/reference weights 1000/1; shared inflated grid and follower",
        "implementation": "navigation/smoothing.py:voronoi_qp_path",
    }
}
DEFAULT_METHOD = "voronoi_qp"
ROOT = Path(__file__).resolve().parents[1]


def resolve_method(environ=None, config_path=None):
    env = os.environ if environ is None else environ
    config = json.loads(Path(config_path or ROOT / "config.json").read_text(encoding="utf-8"))
    method = env.get("PLANNER_ALGO", config.get("algorithm", DEFAULT_METHOD))
    if method not in METHODS:
        raise ValueError(f"Unknown algorithm {method!r}; choose from {METHODS}")
    return method
