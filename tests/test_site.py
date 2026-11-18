"""Check that the public website faithfully exports the archived experiments."""

import copy
import csv
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import statistics
import tempfile
import unittest

from navigation.scenarios import SCENES, make_scene
from navigation.settings import METHOD_LABELS
from scripts.build_site import ROOT, NATIVE_ASSETS, RESULT_ASSETS, build, create_payload


class SiteDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.payload = create_payload()
        with gzip.open(ROOT / "results/raw_runs.json.gz", "rt", encoding="utf-8") as stream:
            cls.archive = json.load(stream)
        cls.by_key = {(run["scene"], run["algorithm"], run["seed"]): run for run in cls.archive.values()}

    def test_aggregates_keep_collision_and_success_only_timing(self):
        self.assertEqual(len(self.payload["runs"]), len(SCENES) * len(METHOD_LABELS) * 5)
        for summary in self.payload["summary"]:
            selected = [run for (scene, method, _), run in self.by_key.items()
                        if scene == summary["scene"] and method == summary["method"]]
            successes = [run for run in selected if run["status"] == "success"]
            self.assertEqual(summary["success"], len(successes))
            self.assertEqual(summary["collision"], sum(run["status"] == "collision" for run in selected))
            if successes:
                self.assertAlmostEqual(summary["mean_time"],
                                       statistics.fmean(run["finish_time"] for run in successes), places=5)
            else:
                self.assertIsNone(summary["mean_time"])
            self.assertAlmostEqual(summary["mean_clearance"],
                                   statistics.fmean(run["min_surface_clearance"] for run in selected), places=5)
            self.assertEqual(sum(summary[key] for key in ("success", "collision", "timeout", "planning_failed")),
                             len(selected))
        self.assertEqual(next(row for row in self.payload["summary"]
                              if row["method"] == "apf")["mean_curvature"], None)

    def test_all_trajectory_samples_and_route_updates_are_preserved(self):
        for run in self.payload["runs"]:
            source = self.by_key[(run["scene"], run["method"], run["seed"])]
            self.assertEqual(len(run["trajectory"]), len(source["trajectory"]))
            for exported, original in zip(run["trajectory"], source["trajectory"]):
                self.assertEqual(exported, [round(original[key], 6)
                                           for key in ("t", "x", "y", "theta", "clearance")])
            self.assertEqual(len(run["route_updates"]), len(source["route_updates"]))
            self.assertEqual(run["status"], source["status"])

    def test_scene_and_dynamic_schedules_match_each_archived_seed(self):
        self.assertEqual([s["id"] for s in self.payload["scenes"]], list(SCENES))
        for item in self.payload["scenes"]:
            for seed in self.payload["seeds"]:
                scene = make_scene(item["id"], seed)
                self.assertEqual(item["bounds"], list(scene.bounds))
                self.assertEqual(len(item["static"]), len(scene.static))
                for exported, original in zip(item["static"], scene.static):
                    self.assertEqual(exported["kind"], original.kind)
                    for key, value in original.params.items():
                        if isinstance(value, tuple):
                            self.assertEqual(exported[key], [round(v, 6) for v in value])
                        else:
                            self.assertAlmostEqual(exported[key], value, places=6)
                self.assertEqual(len(item["moving"][str(seed)]), len(scene.moving))
                for exported, original in zip(item["moving"][str(seed)], scene.moving):
                    self.assertEqual(exported["name"], original.name)
                    self.assertEqual(exported["radius"], original.radius)
                    self.assertEqual(exported["waypoints"],
                                     [[round(v, 6) for v in row] for row in original.waypoints])

    def test_current_labels_keep_stable_method_ids(self):
        self.assertEqual({row["id"]: row["label"] for row in self.payload["methods"]}, METHOD_LABELS)
        self.assertEqual(next(row["label"] for row in self.payload["methods"]
                              if row["id"] == "voronoi_qp"), "G²VD Planner")
        self.assertEqual(self.payload["pairs"], [{"base": "astar", "improved": "astar_yield"},
                                                {"base": "rrt_star", "improved": "rrt_star_yield"}])
        for run in self.payload["runs"]:
            self.assertEqual(run["yield_count"], len(run["yield_events"]))
            self.assertAlmostEqual(run["yield_time_s"],
                                   sum(e["end"]-e["start"] for e in run["yield_events"]), places=5)
        if self.payload["webots"]:
            runs = self.payload["webots"]["runs"]
            self.assertEqual({(row["scene"], row["method"]) for row in runs},
                             {(s, m) for s in SCENES for m in METHOD_LABELS})
        json.dumps(self.payload, allow_nan=False)

    def fixture(self, directory):
        root = Path(directory)
        (root / "results").mkdir()
        for name in ("raw_runs.json.gz", "summary.csv", "manifest.json"):
            shutil.copy2(ROOT / "results" / name, root / "results" / name)
        if self.payload["webots"]:
            for name in NATIVE_ASSETS:
                shutil.copy2(ROOT / "results" / name, root / "results" / name)
        return root

    def test_inconsistent_csv_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(directory)
            path = root / "results/summary.csv"
            with path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                fieldnames, rows = reader.fieldnames, list(reader)
            rows[0]["finish_time"] = "0"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "CSV/archive finish_time mismatch"):
                create_payload(root)

    def test_missing_run_and_nonfinite_values_are_rejected(self):
        for mutation, message in (("missing", "Archive filenames"), ("nonfinite", "Non-finite")):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                root = self.fixture(directory)
                archive = copy.deepcopy(self.archive)
                first = next(iter(archive))
                if mutation == "missing":
                    del archive[first]
                else:
                    archive[first]["trajectory"][0]["x"] = float("nan")
                raw = root / "results/raw_runs.json.gz"
                with gzip.open(raw, "wt", encoding="utf-8") as stream:
                    json.dump(archive, stream)
                manifest_path = root / "results/manifest.json"
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                manifest["raw_runs_sha256"] = hashlib.sha256(raw.read_bytes()).hexdigest()
                manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    create_payload(root)

    def test_build_preserves_source_assets_and_unrelated_destination_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(directory)
            for name in RESULT_ASSETS:
                shutil.copy2(ROOT / "results" / name, root / "results" / name)
            source = root / "site"
            source.mkdir()
            source_html = (
                '<!doctype html><title>Fixture</title>'
                '<link rel="stylesheet" href="style.css">'
                "<script src='data.js' defer></script>"
                '<script src="./app.js" defer></script>'
                '<a href="https://example.com/app.js">External source</a>'
            )
            (source / "index.html").write_text(source_html)
            (source / "style.css").write_text("body { color: green; }")
            (source / "app.js").write_text("window.appVersion = 1;")
            output = root / "dist"
            output.mkdir()
            (output / "keep.txt").write_text("Do not remove")
            original_hash = hashlib.sha256((root / "results/raw_runs.json.gz").read_bytes()).hexdigest()
            built, payload = build(root)
            self.assertEqual(built, output)
            self.assertEqual((output / "keep.txt").read_text(), "Do not remove")
            self.assertFalse((source / "data.js").exists())
            self.assertTrue((output / ".nojekyll").is_file())
            assignment = (output / "data.js").read_text(encoding="utf-8").split("window.BENCHMARK_DATA = ", 1)[1]
            self.assertEqual(json.loads(assignment.rstrip().removesuffix(";")), payload)
            self.assertEqual(hashlib.sha256((output / "results/raw_runs.json.gz").read_bytes()).hexdigest(),
                             original_hash)
            def versions():
                html = (output / "index.html").read_text()
                return dict(re.findall(r"(style\.css|app\.js|data\.js)\?v=([0-9a-f]+)", html))

            first_versions = versions()
            self.assertEqual(set(first_versions), {"style.css", "app.js", "data.js"})
            for asset, version in first_versions.items():
                self.assertEqual(version, hashlib.sha256((output / asset).read_bytes()).hexdigest()[:12])
            self.assertEqual((source / "index.html").read_text(), source_html)
            self.assertIn('href="https://example.com/app.js"', (output / "index.html").read_text())

            # A new application deployment must bypass cached JS without forcing
            # unchanged styles or experiment data onto a different cache key.
            (source / "app.js").write_text("window.appVersion = 2;")
            build(root)
            second_versions = versions()
            self.assertNotEqual(first_versions["app.js"], second_versions["app.js"])
            self.assertEqual(first_versions["style.css"], second_versions["style.css"])
            self.assertEqual(first_versions["data.js"], second_versions["data.js"])
            self.assertEqual((source / "index.html").read_text(), source_html)
            self.assertEqual((output / "keep.txt").read_text(), "Do not remove")
            with self.assertRaisesRegex(ValueError, "must not overwrite source"):
                build(root, source)


if __name__ == "__main__":
    unittest.main()
