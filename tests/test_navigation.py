"""Critical geometry, sensing, control and world-consistency regression tests."""
import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.core import Navigator, astar
from navigation.classical import voronoi_path, voronoi_visibility_path
from navigation.scenarios import SCENES, Scene, MovingDisc, make_scene, clearance, validate_scene
from navigation.grid_map import GridMap, StaticObstacle
from navigation.settings import resolve_method
from navigation.tracking import ScanTracker
from navigation.simulation import scan, run
from scripts.generate_worlds import world_text


class NavigationTests(unittest.TestCase):
    def test_all_actor_schedules_clear_static_geometry(self):
        for name in SCENES:
            for seed in range(5):
                validate_scene(make_scene(name, seed))

    def test_native_world_matches_registry_and_enables_near_lidar(self):
        for name in SCENES:
            scene = make_scene(name)
            text = world_text(scene)
            self.assertEqual(text.count("DEF STATIC_"), len(scene.static))
            self.assertEqual(text.count("DEF MOVING_"), len(scene.moving))
            self.assertIn('controller "coursework_navigator"', text)
            self.assertIn("minRange 0.03", text)
            self.assertNotIn("resetPhysics", text)
            self.assertEqual(text, (ROOT / "worlds" / f"{name}.wbt").read_text(encoding="utf-8"))

    def test_near_obstacle_is_not_filtered_away(self):
        scene = Scene("sensor", moving=[MovingDisc("disc", .1, [[0, .3, 0], [1, .3, 0]])])
        points = scan(scene, (0., 0., 0.), 0)
        self.assertAlmostEqual(float(np.linalg.norm(points, axis=1).min()), .2)
        self.assertLess(clearance(scene, np.array([.03, 0.]), 0), 0)

    def test_static_astar_respects_robot_inflation(self):
        scene = make_scene()
        grid = scene.grid()
        path = astar(grid, scene.start, scene.goal)
        self.assertIsNotNone(path)
        self.assertTrue(all(grid.is_line_collision_free(a, b, step=.02) for a, b in zip(path[:-1], path[1:])))

    def test_global_path_follower_completes_static_map(self):
        scene = replace(make_scene(), moving=[])
        a, b = run(scene, "astar"), run(scene, "voronoi_visibility")
        self.assertEqual(a["status"], "success")
        self.assertEqual(b["status"], "success")

    def test_vv_visibility_search_shortens_the_greedy_voronoi_route(self):
        scene = make_scene()
        grid = scene.grid()
        baseline = voronoi_path(grid, scene.start, scene.goal)
        vv = voronoi_visibility_path(grid, scene.start, scene.goal)
        length = lambda p: np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        self.assertLess(length(vv), length(baseline))
        self.assertTrue(np.array_equal(vv[0], scene.start))
        self.assertTrue(np.array_equal(vv[-1], scene.goal))

    def test_one_map_has_turns_and_a_genuinely_blocked_short_route(self):
        scene = make_scene()
        self.assertEqual(SCENES, ("warehouse",))
        path = astar(scene.grid(), scene.start, scene.goal)
        self.assertGreaterEqual(len(path), 5)
        self.assertFalse(scene.grid().is_line_collision_free(scene.start, scene.goal))
        cart = scene.moving[0]
        self.assertLess(np.linalg.norm(cart.position(30)), .01)
        # The robot cannot fit beside the parked cart in the 1.4m passage.
        self.assertLess(.7-cart.radius, 2*.18)
        for actor in scene.moving:
            points = np.asarray(actor.waypoints)
            speeds = np.linalg.norm(np.diff(points[:, 1:], axis=0), axis=1)/np.diff(points[:, 0])
            self.assertLessEqual(max(speeds), .4)

    def test_rotation_can_recover_inside_comfort_margin(self):
        nav = Navigator(make_scene(), "dwa")
        v, w = nav.dwa((0., 0., 0.), (0., 0.), (-1., 0.), np.array([[.25, 0.]]), .1)
        self.assertEqual(v, 0.)
        self.assertGreater(abs(w), 0.)

    def test_lidar_velocity_estimate_uses_observed_surface_motion(self):
        tracker = ScanTracker()
        points = np.array([[0., -.1], [.05, 0.], [0., .1]])
        for i in range(20):
            velocity = tracker.update(points+[1.+.02*i, 0.], [], .1*i)
        self.assertTrue(np.allclose(velocity, [.2, 0.], atol=.02))

    def test_algorithm_override_is_validated_by_both_adapters(self):
        self.assertEqual(resolve_method({"PLANNER_ALGO": "astar"}), "astar")
        with self.assertRaises(ValueError):
            resolve_method({"PLANNER_ALGO": "informed_rrt_star_dwa"})

    def test_classical_global_planners_return_collision_free_paths(self):
        for method in ("astar", "rrt_star", "voronoi", "voronoi_visibility"):
            nav = Navigator(make_scene(), method)
            self.assertFalse(nav.planning_failed, method)
            self.assertTrue(all(nav.grid.is_line_collision_free(a, b, step=.02)
                                for a, b in zip(nav.path[:-1], nav.path[1:])), method)
            with patch.object(nav, "dwa", side_effect=AssertionError("Global baseline called DWA")):
                nav.step(np.array([*nav.scene.start, 0.]), (0., 0.), np.empty((0, 2)), 0., .1)

    def test_line_check_catches_a_very_short_cell_intersection(self):
        grid = GridMap([], min_x=0., max_x=3., min_y=0., max_y=3., resolution=1.)
        grid.dist_map[:] = 2.
        grid.dist_map[1, 1] = 0.
        # x+y=2.0001 crosses the occupied cell's lower-left corner for 0.14mm.
        for spacing in (.5, .05, .00001):
            self.assertFalse(grid.is_line_collision_free((.1, 1.9001), (1.9, .1001), step=spacing))
        self.assertTrue(grid.is_line_collision_free((.1, .1), (.9, .9)))
        grid.dist_map[:] = 2.
        self.assertTrue(grid.is_line_collision_free((.975, 1.025), (1., 1.)))

    def test_local_baselines_use_goal_without_a_global_planner(self):
        for method in ("apf", "dwa"):
            scene = make_scene()
            nav = Navigator(scene, method)
            self.assertTrue(np.array_equal(nav.path, [scene.start, scene.goal]))

    def test_sampling_planning_failure_is_recorded(self):
        with patch("navigation.core.rrt_star", return_value=None):
            result = run(make_scene(), "rrt_star")
        self.assertEqual(result["status"], "planning_failed")
        self.assertFalse(result["success"])
        self.assertEqual(result["control_p95_ms"], 0.)

    def test_astar_rejects_an_occupied_start(self):
        grid = GridMap(robot_radius=.18, safety_margin=.22,
                       obstacles=[StaticObstacle("cylinder", {"center": (0.,0.), "radius": .48})])
        self.assertTrue(grid.is_occupied(*grid.world_to_grid(-.8, 0.)))
        self.assertIsNone(astar(grid, (-.8, 0.), (4., 0.)))



if __name__ == "__main__":
    unittest.main()
