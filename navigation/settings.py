"""Supported methods and shared controller configuration."""
import json
import os
from pathlib import Path

# Display names are independent of the stable IDs used in configs and archives.
METHOD_LABELS = {
    "astar": "A*",
    "astar_yield": "A*（提前让行）",
    "rrt_star": "RRT*",
    "rrt_star_yield": "RRT*（提前让行）",
    "voronoi": "Voronoi",
    "apf": "APF controller",
    "dwa": "DWA controller",
    "voronoi_qp": "G²VD Planner",
}
# Literature-inspired coursework extensions; kept outside the canonical set so
# the published site data (8 methods) and its validation stay untouched.
EXTENSION_LABELS = {
    "astar_risk": "A*（风险自适应让行）",
    "astar_hypo": "A*（多假设让行）",
    "astar_recip": "A*（互惠让行）",
    "astar_t": "A*（时空规划）",
    "astar_ppo": "A*（PPO 让行）",
}
METHODS = tuple(METHOD_LABELS)
EXTENSION_METHODS = tuple(EXTENSION_LABELS)
YIELD_BASES = {"astar_yield": "astar", "rrt_star_yield": "rrt_star"}
# Guard-parameterised variants of the base planners; same follower, same conditions.
GUARD_VARIANTS = {
    "astar_risk": {"base": "astar",
                   "params": {"risk_coeff": .5}},
    "astar_hypo": {"base": "astar",
                   "params": {"frozen_hypothesis": True, "include_stopped": True}},
    "astar_recip": {"base": "astar",
                    "params": {"reciprocity": .5}},
}
# Methods whose guide planner produces an explicit (space, time) schedule.
TIMED_PLANNERS = {"astar_t": "astar"}
# Learned guards constructed by a factory, outside the CrossingGuard parameter space.
GUARD_FACTORIES = {"astar_ppo": {"factory": "navigation.ppo_yield:guard_factory",
                                  "base": "astar"}}
EXPERIMENT_GROUPS = {
    "crossing_comparisons": [[base, improved] for improved, base in YIELD_BASES.items()],
    "global_planning_common_follower": ["astar", "rrt_star", "voronoi", "voronoi_qp"],
    "local_control_common_astar_guide": ["astar", "apf", "dwa"],
}
NAVIGATION_PROTOCOL = {
    "dynamic_map": "static occupancy + observed hit marking, free-ray clearing and 1s expiry",
    "dynamic_map_source": "https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/",
    "replan": "check remaining route and robot-to-route connection every control step; observed blockage triggers replanning with a 1s minimum interval between attempts",
    "local_controller_guide": "A* with the same dynamic observation layer and replan trigger",
    "shared_safety": "same speed, wheel, acceleration limits and scan-based braking",
    "crossing_comparisons": {
        "input": "rolling 0.65 s history of non-static lidar hits in world coordinates; tracks expire after 0.55 s without a detection",
        "prediction": "compare predicted motion along the current route with braking to a stop at 0.12 s intervals up to 3.48 s",
        "action": "brake before a predicted encounter when the stopping location is safe; continue through if stopping in the crossing is more dangerous",
        "stalled_after_blockage": "after yielding or an observed blockage, replan from the current pose if displacement stays below 0.10 m for 2.5 s",
        "yield_time_s": "sum of intervals requesting braking for a crossing, including deceleration",
    },
    "inflation_escape": "checked connection and increasing-clearance motion at radius+0.03m; occupied cells are retained",
    "schedules": "one slow cart per public layout; intermediate waypoint times shift together by up to 0.6 s per seed; fixed robot configuration",
}
METHOD_REFERENCES = {
    "voronoi": {"title": "Geometric Voronoi roadmap of sampled obstacle boundaries",
                "implementation": "navigation/classical.py:voronoi_path",
                "geometry": "scipy.spatial.Voronoi finite free-space ridges and line-of-sight simplification"},
    "astar": {"authors": "Hart, Nilsson, Raphael", "year": 1968,
              "title": "A Formal Basis for the Heuristic Determination of Minimum Cost Paths",
              "doi": "10.1109/TSSC.1968.300136", "implementation": "navigation/core.py:astar"},
    "rrt_star": {"authors": "Karaman, Frazzoli", "year": 2011,
                 "title": "Sampling-based Algorithms for Optimal Motion Planning",
                 "url": "https://arxiv.org/abs/1105.1186", "implementation": "navigation/classical.py:rrt_star",
                 "iterations": 1800, "step_m": .4, "goal_bias": .15},
    "apf": {"authors": "Khatib", "year": 1986,
            "title": "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots",
            "doi": "10.1177/027836498600500106", "implementation": "navigation/local.py:apf_command",
            "nearest_surface_reference": "https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py",
            "adaptation": "damped virtual velocity projected to differential-drive heading; nearest measured surface potential; common A* reference",
            "parameters": {"attraction": 2., "damping": 2., "repulsion": .008, "influence_m": .5}},
    "dwa": {"authors": "Fox, Burgard, Thrun", "year": 1997,
            "title": "The Dynamic Window Approach to Collision Avoidance",
            "url": "https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf",
            "implementation": "navigation/local.py:dwa_command",
            "adaptation": "circular footprint and measured hit disks; continuous arc contact and braking test; normalized three-term objective; common A* reference",
            "parameters": {"heading_weight": .60, "clearance_weight": .25, "speed_weight": .15,
                           "clearance_saturation_m": 1., "velocity_samples": 7, "yaw_samples": 31}},
    "voronoi_qp": {
        "authors": "Wen, Zhang, Bi, Liu, Yuan, Fang",
        "year": 2024,
        "title": "G²VD Planner: Efficient Motion Planning With Grid-based Generalized Voronoi Diagrams",
        "url": "https://arxiv.org/abs/2201.12981",
        "doi": "10.1109/TASE.2024.3398996",
        "reproduced_component": "Section IV-D, safe-bubble QP smoothing, Equations (5)-(8)",
        "adaptation": "geometric Voronoi reference; smoothness/reference weights 1000/1; shared inflated grid and follower",
        "implementation": "navigation/smoothing.py:voronoi_qp_path",
    }
}
for improved, base in YIELD_BASES.items():
    METHOD_REFERENCES[improved] = {"base_planner": base, "name": METHOD_LABELS[improved],
                                   "implementation": "navigation/yielding.py:CrossingGuard",
                                   "contribution": "coursework extension for crossings observed by lidar"}
for variant, spec in GUARD_VARIANTS.items():
    METHOD_REFERENCES[variant] = {"base_planner": spec["base"], "name": EXTENSION_LABELS[variant],
                                  "implementation": "navigation/yielding.py:CrossingGuard",
                                  "guard_parameters": spec["params"]}
METHOD_REFERENCES["astar_risk"].update({
    "idea": "inflate the yield clearance by the velocity-estimate standard error, "
            "which grows with the prediction horizon",
    "references": [{"title": "Risk-Aware Adaptive Safety Margins for Model Predictive Control "
                             "with Orientation-Motion Coupled Barrier Functions in Dynamic Environments",
                    "year": 2026, "url": "https://www.mdpi.com/2076-0825/15/2/116"},
                   {"title": "Improved Obstacle Avoidance for Autonomous Robots with ORCA-FLC",
                    "year": 2025, "url": "https://www.alphaxiv.org/abs/2508.06722"}]})
METHOD_REFERENCES["astar_hypo"].update({
    "idea": "evaluate both the constant-velocity and the frozen-position hypotheses per track "
            "and yield on the worst case; stopped tracks are no longer ignored",
    "references": [{"title": "Motion Planning in Dynamic Environments: A Survey",
                    "year": 2026, "url": "https://arxiv.org/html/2606.02677v1"}]})
METHOD_REFERENCES["astar_recip"].update({
    "idea": "ablation: assume the crossing cart reciprocates and provides half of the required "
            "separation; the scripted carts never avoid, so collisions are expected",
    "references": [{"title": "Reciprocal n-Body Collision Avoidance (ORCA)",
                    "authors": "van den Berg, Guy, Lin, Manocha", "year": 2011},
                   {"title": "VR-ORCA: Variable Responsibility Optimal Reciprocal Collision Avoidance",
                    "year": 2021, "url": "https://www.mdpi.com/2076-0825/15/2/116"}]})
METHOD_REFERENCES["astar_t"] = {
    "base_planner": "astar", "name": EXTENSION_LABELS["astar_t"],
    "implementation": "navigation/timed.py:timed_astar",
    "idea": "plan in quantised (space, time) with explicit wait actions from lidar-derived "
            "track predictions, instead of braking reactively at the crossing edge",
    "references": [{"title": "Unifying Decision-Making and Trajectory-Planning in Unsignalized "
                             "Intersections Using Time-Varying Potential Fields",
                    "year": 2026, "url": "https://www.alphaxiv.org/abs/2607.13626"},
                   {"title": "Motion Planning in Dynamic Environments: A Survey",
                    "year": 2026, "url": "https://arxiv.org/html/2606.02677v1"}]}
METHOD_REFERENCES["astar_ppo"] = {
    "base_planner": "astar", "name": EXTENSION_LABELS["astar_ppo"],
    "implementation": "navigation/ppo_yield.py:PPOGuard",
    "idea": "a PPO policy replaces the rule-based yield threshold, mapping frame-invariant "
            "track features to {go, slow, yield}; a physical-clearance shield and the "
            "unsafe-stop rule can force a hold but are never overridden",
    "references": [{"authors": "Schulman, Wolski, Dhariwal, Radford, Klimov", "year": 2017,
                    "title": "Proximal Policy Optimization Algorithms",
                    "url": "https://arxiv.org/abs/1707.06347"},
                   {"title": "Safe Efficient Policy Optimization for Unsignalized "
                             "Intersection Decision-Making (RL high-level, MPC low-level)",
                    "year": 2024, "url": "https://www.ieee-jas.com/article/doi/10.1109/JAS.2024.124287"}],
    "training": "scripts/train_ppo.py; on-policy PPO in the 2D simulator with cart schedules "
                "randomised +-2 s; weights in assets/ppo_yield_weights.npz"}
DEFAULT_METHOD = "astar_yield"
ROOT = Path(__file__).resolve().parents[1]


def resolve_method(environ=None, config_path=None):
    env = os.environ if environ is None else environ
    config = json.loads(Path(config_path or ROOT / "config.json").read_text(encoding="utf-8"))
    method = env.get("PLANNER_ALGO", config.get("algorithm", DEFAULT_METHOD))
    if method not in METHODS:
        raise ValueError(f"Unknown algorithm {method!r}; choose from {METHODS}")
    return method
