"""Critical geometry, sensing, control and world-consistency regression tests."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.core import Navigator, astar
from navigation.scenarios import SCENES, Scene, MovingDisc, make_scene, clearance, validate_scene, GridMap, StaticObstacle
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

    def test_near_obstacle_is_not_filtered_away(self):
        scene = Scene("sensor", moving=[MovingDisc("disc", .1, [[0, .3, 0], [1, .3, 0]])])
        points = scan(scene, (0., 0., 0.), 0)
        self.assertAlmostEqual(float(np.linalg.norm(points, axis=1).min()), .2)
        self.assertLess(clearance(scene, np.array([.03, 0.]), 0), 0)

    def test_static_astar_respects_robot_inflation(self):
        scene = make_scene("static_detour")
        grid = scene.grid()
        path = astar(grid, scene.start, scene.goal)
        self.assertIsNotNone(path)
        self.assertTrue(all(grid.is_line_collision_free(a, b, step=.02) for a, b in zip(path[:-1], path[1:])))

    def test_control_and_replanning_share_baseline_on_static_scene(self):
        scene = make_scene("open_control")
        a, b = run(scene, "fixed_dwa"), run(scene, "replan_dwa")
        self.assertEqual(a["trajectory"], b["trajectory"])
        self.assertEqual(a["status"], "success")
        self.assertEqual(b["replans"], 0)

    def test_replanning_uses_observations_to_route_around_blocker(self):
        result = run(make_scene("blocked_passage"), "replan_dwa")
        self.assertEqual(result["status"], "success")
        self.assertGreater(result["replans"], 0)
        self.assertTrue(any(max(p[1] for p in h["path"]) > 2.2 for h in result["path_history"]))

    def test_safe_start_can_exit_new_obstacle_inflation(self):
        grid = GridMap(robot_radius=.18, safety_margin=.22,
                       obstacles=[StaticObstacle("cylinder", {"center": (0.,0.), "radius": .48})])
        start = (-.8, 0.)
        self.assertTrue(grid.is_occupied(*grid.world_to_grid(*start)))
        path = astar(grid, start, (4., 0.))
        self.assertIsNotNone(path)
        self.assertTrue(all(grid.is_line_collision_free(a, b, step=.01, margin=.19)
                            for a, b in zip(path[:-1], path[1:])))
        self.assertIsNone(astar(grid, (-.5,0.), (4.,0.)))


if __name__ == "__main__":
    unittest.main()
