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
from navigation.core import Navigator, Config, astar
from navigation.yielding import CrossingGuard
from navigation.classical import voronoi_path
from navigation.smoothing import voronoi_qp_path
from navigation.metrics import resample_path, path_metrics
from navigation.scenarios import SCENES, Scene, MovingDisc, make_scene, clearance, validate_scene
from navigation.grid_map import GridMap, StaticObstacle
from navigation.settings import resolve_method
from navigation.observation_map import ObservationMap
from navigation.local import apf_command, collision_distance
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

    def test_collision_log_uses_the_actual_swept_substep_time(self):
        scene = Scene("collision_clock", start=(0., 0.), goal=(4., 0.), duration=1.,
                      moving=[MovingDisc("disc", .1, [[0., .3, 0.], [1., .31, 0.]])])
        with patch.object(Navigator, "step", return_value=(.45, 0.)):
            result = run(scene, "astar")
        last = result["trajectory"][-1]
        self.assertEqual(result["status"], "collision")
        self.assertLess(last["t"], .1)
        self.assertAlmostEqual(last["clearance"], clearance(scene, np.array([last['x'], last['y']]), last['t']))

    def test_static_astar_respects_robot_inflation(self):
        scene = make_scene()
        grid = scene.grid()
        path = astar(grid, scene.start, scene.goal)
        self.assertIsNotNone(path)
        self.assertTrue(all(grid.is_line_collision_free(a, b, step=.02) for a, b in zip(path[:-1], path[1:])))

    def test_global_path_follower_completes_static_map(self):
        scene = replace(make_scene(), moving=[])
        a, b = run(scene, "astar"), run(scene, "voronoi_qp")
        self.assertEqual(a["status"], "success")
        self.assertEqual(b["status"], "success")

    def test_safe_qp_reduces_curvature_and_keeps_clearance(self):
        from navigation.scenarios import static_distance
        scene = make_scene()
        grid = scene.grid()
        baseline = voronoi_path(grid, scene.start, scene.goal)
        smooth = voronoi_qp_path(grid, scene.start, scene.goal)
        self.assertLess(path_metrics(smooth)["planned_peak_curvature"], path_metrics(baseline)["planned_peak_curvature"])
        baseline_gap = static_distance(resample_path(baseline, .01), scene.static).min()
        smooth_gap = static_distance(resample_path(smooth, .01), scene.static).min()
        self.assertGreater(smooth_gap, baseline_gap)
        self.assertTrue(np.array_equal(smooth[0], scene.start))
        self.assertTrue(np.allclose(smooth[-1], scene.goal))

    def test_follower_lookahead_does_not_depend_on_waypoint_density(self):
        nav = Navigator(make_scene(), "voronoi")
        start = np.array(nav.scene.start)
        nav.path = np.array([start, start + [0., 1.]])
        coarse = nav.target(start)
        nav.path = resample_path(nav.path, .02)
        self.assertTrue(np.allclose(nav.target(start), coarse))

    def test_public_courses_have_passable_turns_and_slow_moving_obstacles(self):
        from navigation.scenarios import SCENE_INFO
        self.assertEqual(set(SCENES), {"turtlebot3_stage2", "pointmaze_u", "pointmaze_medium"})
        for name in SCENES:
            scene = make_scene(name)
            grid = scene.grid()
            self.assertFalse(grid.is_line_collision_free(scene.start, scene.goal), name)
            self.assertIsNotNone(astar(grid, scene.start, scene.goal), name)
            self.assertTrue(scene.moving, name)
            for actor in scene.moving:
                points = np.asarray(actor.waypoints)
                speeds = np.linalg.norm(np.diff(points[:, 1:], axis=0), axis=1)/np.diff(points[:, 0])
                self.assertGreater(max(speeds), .05)
                self.assertLessEqual(max(speeds), .4)
            if "grid" in SCENE_INFO[name]:
                spec = SCENE_INFO[name]
                for r, row in enumerate(spec["grid"]):
                    for c, cell in enumerate(row):
                        xy = (scene.bounds[0] + (c+.5)*spec["cell_size"],
                              scene.bounds[3] - (r+.5)*spec["cell_size"])
                        self.assertEqual(grid.is_occupied(*grid.world_to_grid(*xy), use_inflation=False), cell == "1")
            else:
                self.assertEqual(sum(obs.kind == "cylinder" for obs in scene.static), 4)

    def test_rotation_can_recover_inside_comfort_margin(self):
        nav = Navigator(make_scene(), "dwa")
        v, w = nav.dwa((0., 0., 0.), (0., 0.), (-1., 0.), np.array([[.25, 0.]]), .1)
        self.assertEqual(v, 0.)
        self.assertGreater(abs(w), 0.)

    def test_observed_occupancy_is_ray_cleared_and_expires(self):
        layer = ObservationMap(Scene("observation"))
        layer.update(np.zeros(2), np.array([[.7, 0.]]), 0.)
        self.assertTrue(layer.blocks(np.array([[.7, 0.]])))
        layer.update(np.zeros(2), np.array([[1.5, 0.]]), .1)
        self.assertFalse(layer.blocks(np.array([[.7, 0.]])))
        self.assertTrue(layer.blocks(np.array([[1.5, 0.]])))
        layer.update(np.zeros(2), np.empty((0, 2)), 1.2)
        self.assertFalse(layer.dynamic.any())

    def test_local_controllers_pass_basic_obstacle_detour(self):
        scene = Scene("basic", start=(-2., 0.), goal=(2., 0.), duration=35.,
                      static=[StaticObstacle("cylinder", {"center": (0., .2), "radius": .35})])
        for method in ("apf", "dwa"):
            result = run(scene, method)
            self.assertEqual(result["status"], "success", method)
            self.assertGreater(result["min_surface_clearance"], 0.)

    def test_apf_zero_force_does_not_command_forward_speed(self):
        from scipy.optimize import brentq
        nav = Navigator(make_scene(), "apf")
        pose = np.array([0., 0., 0.])
        gap = brentq(lambda d: .008*(1/d-2)/d**2-2., .1, .3)
        hit = np.array([[gap+nav.cfg.radius, 0.]])
        v, w = apf_command(pose, (0., 0.), (4., 0.), hit, .1, nav.cfg)
        self.assertLess(abs(v)+abs(w), 1e-7)

    def test_dwa_continuous_arc_collision_distance(self):
        straight = collision_distance(np.zeros(3), np.array([.3]), np.array([0.]), np.array([[1., 0.]]), .28)
        self.assertAlmostEqual(straight[0], .72)
        turn = collision_distance(np.zeros(3), np.array([.3]), np.array([.3]), np.array([[1., 1.]]), .28)
        self.assertAlmostEqual(turn[0], math.pi/2-2*math.asin(.14))

    def test_dwa_can_move_away_inside_the_comfort_margin(self):
        nav = Navigator(make_scene(), "dwa")
        v, w = nav.dwa((0., 0., math.pi), (0., 0.), (-1., 0.), np.array([[.25, 0.]]), .1)
        self.assertGreater(v, 0.)
        self.assertLessEqual(abs(w), nav.cfg.angular_acceleration*.1+1e-9)

    def test_global_planners_detour_before_a_temporary_blockage_ends(self):
        cart = MovingDisc("cart", .28, [[0, 0, -1], [3, 0, 0], [37, 0, 0], [40, 0, 1]])
        scene = Scene("temporary_blockage", start=(-2.5, 0.), goal=(2.5, 0.), duration=40., moving=[cart])
        for method in ("astar", "rrt_star", "astar_yield", "rrt_star_yield"):
            result = run(scene, method)
            self.assertEqual(result["status"], "success", method)
            self.assertLess(result["finish_time"], 37.)
            self.assertGreater(result["replan_count"], 0)

    def test_controller_does_not_query_scripted_actor_states(self):
        scene = make_scene()
        for method in ("astar", "astar_yield", "rrt_star_yield"):
            with patch.object(scene.moving[0], "position", side_effect=AssertionError("Oracle query")):
                nav = Navigator(scene, method)
                self.assertFalse(nav.scene.moving)
                nav.step(np.array([*scene.start, 0.]), (0., 0.), np.empty((0, 2)), 0., .1)

    def test_paired_planners_yield_before_the_crossing_and_then_reach_the_goal(self):
        for name, seed, planner in (("turtlebot3_stage2", 0, "astar"), ("pointmaze_u", 1, "rrt_star")):
            scene = make_scene(name, seed)
            baseline, improved = run(scene, planner), run(scene, planner+"_yield")
            self.assertEqual(baseline["initial_path"], improved["initial_path"])
            self.assertEqual(baseline["status"], "collision")
            self.assertEqual(improved["status"], "success")
            self.assertGreater(baseline["replan_count"], 0)
            self.assertLess(improved["yield_events"][0]["start"], baseline["finish_time"])
            self.assertLess(improved["yield_events"][-1]["end"], improved["finish_time"])
            self.assertGreater(improved["min_surface_clearance"], 0.)

    def test_stopping_in_the_crossing_is_rejected_and_stale_scans_expire(self):
        guard = CrossingGuard(Config())
        position = np.array([.20, 0.])
        route = np.array([position, [2., 0.]])
        arc = np.array([[.16, 0.], [.128, .096], [.096, .128]])
        for t in np.linspace(0., .6, 7):
            guard.observe(t, position, arc+[0., .59-.2*t])
        np.testing.assert_allclose(guard.tracks[0].velocity, [0., -.2], atol=1e-8)
        guard.hold_until = 1.
        self.assertFalse(guard.should_yield(.6, .1, position, .45, route))
        guard.observe(1.3, position, np.empty((0, 2)))
        self.assertEqual(guard.tracks, [])

    def test_static_control_uses_the_same_route_without_yielding(self):
        scene = replace(make_scene(), moving=[])
        baseline, improved = run(scene, "astar"), run(scene, "astar_yield")
        self.assertEqual(improved["trajectory"], baseline["trajectory"])
        self.assertEqual(improved["yield_events"], [])

    def test_offset_robot_replans_even_when_remaining_reference_is_clear(self):
        scene = Scene("offset_reference", start=(-.75, -.14), goal=(-.85, 2.),
                      moving=[MovingDisc("disc", .48, [[0, 0, 0], [100, 0, 0]])])
        nav = Navigator(scene, "rrt_star")
        nav.path = np.array([[-.85, -.3], [-.85, 2.]])
        pose = np.array([*scene.start, math.pi/2])
        points = scan(scene, pose, 20.)
        nav.observation.update(pose[:2], points, 20.)
        self.assertFalse(nav.observation.blocks(nav.remaining_path(pose[:2])))
        nav.update_route(pose, points, 20.)
        self.assertEqual(len(nav.plan_attempts), 1)
        self.assertTrue(nav.plan_attempts[0]["success"])
        self.assertTrue(np.allclose(nav.path[0], pose[:2]))

    def test_inflation_escape_keeps_the_obstacle_occupied(self):
        scene = Scene("escape", start=(.75, 0.), goal=(2., 0.), static=[StaticObstacle(
            "box", {"center": (0., 0.), "size": (1., 1.)})])
        nav = Navigator(scene, "astar")
        self.assertTrue(nav.grid.is_occupied(*nav.grid.world_to_grid(*scene.start)))
        self.assertFalse(nav.planning_failed)
        self.assertTrue(nav.grid.is_line_collision_free(nav.path[0], nav.path[1], margin=.21))
        self.assertTrue(nav.grid.is_occupied(*nav.grid.world_to_grid(0., 0.), use_inflation=False))
        self.assertGreater(np.linalg.norm(nav.target(np.array(scene.start))-scene.start), .3)

    def test_new_blockage_is_checked_before_the_next_planning_tick(self):
        nav = Navigator(make_scene(), "astar")
        pose = np.array([*nav.scene.start, 0.])
        with patch.object(nav.observation, "blocks", side_effect=[False, True]), \
             patch.object(nav, "plan", return_value=nav.path.copy()):
            nav.update_route(pose, np.empty((0, 2)), 0.)
            nav.update_route(pose, np.empty((0, 2)), .2)
        self.assertEqual(len(nav.plan_attempts), 1)
        self.assertAlmostEqual(nav.plan_attempts[0]["t"], .2)

    def test_algorithm_override_is_validated_by_both_adapters(self):
        self.assertEqual(resolve_method({"PLANNER_ALGO": "astar"}), "astar")
        with self.assertRaises(ValueError):
            resolve_method({"PLANNER_ALGO": "informed_rrt_star_dwa"})

    def test_classical_global_planners_return_collision_free_paths(self):
        for method in ("astar", "rrt_star", "voronoi", "voronoi_qp"):
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

    def test_local_baselines_use_the_same_astar_reference(self):
        reference = Navigator(make_scene(), "astar").path
        for method in ("apf", "dwa"):
            scene = make_scene()
            nav = Navigator(scene, method)
            self.assertTrue(np.array_equal(nav.path, reference))
            self.assertEqual(nav.role, "local_controller")
            self.assertEqual(nav.guide_planner, "astar")

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
