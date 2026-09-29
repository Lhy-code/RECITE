"""Numerical and recorded-example checks for the CPU compiler stages."""
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class ReleaseTests(unittest.TestCase):
    def test_existing_scene_rules_and_variants(self):
        self.assertTrue((ROOT / "recite/scenes.py").exists())
        scenes = importlib.import_module("recite.scenes")
        self.assertEqual(len(scenes.ALL_BASE_SCENES), 25)
        variants = scenes.generate_variants(scenes.ALL_BASE_SCENES["rm65_corridor"], 3)
        self.assertEqual(len(variants), 3)
        self.assertTrue(all(scenes.validate_scene(v)["valid"] for v in variants))

    def test_ici_end_exclusive_and_anchor_suffix(self):
        self.assertTrue((ROOT / "recite/srir.py").exists())
        srir = importlib.import_module("recite.srir")
        clearance = np.r_[np.full(10, .04), np.full(10, .12)]
        intervals = srir.find_event_windows(clearance)
        self.assertEqual([(w["start"], w["end"]) for w in intervals], [(0, 10)])
        route = np.column_stack([np.arange(20) * .01, np.zeros((20, 2))])
        anchors = srir.compute_rejoin_anchors(clearance, route, intervals)
        self.assertEqual(anchors[0]["step"], 10)

    def test_contact_depth_and_approach_speed(self):
        self.assertTrue((ROOT / "recite/metrics.py").exists())
        metrics = importlib.import_module("recite.metrics")
        result = metrics.compute_dynamic_contact_severity(
            robot_centers=np.zeros((3, 1, 3)),
            raw_robot_radii=np.full((3, 1), .05),
            robot_links=np.array(["test_link"]),
            dynamic_centers=np.array([[[.20, 0, 0]], [[.09, 0, 0]], [[.08, 0, 0]]]),
            dynamic_radii=np.array([.05]),
            dynamic_active=np.ones((3, 1), dtype=bool),
            time_s=np.array([0., 1., 2.]),
        )
        self.assertEqual(result["first_dynamic_contact_frame"], 1)
        self.assertAlmostEqual(result["maximum_zero_buffer_dynamic_overlap_m"], .02)
        self.assertAlmostEqual(result["first_dynamic_contact_normal_approach_speed_m_s"], .11)

    def test_archived_route_demo(self):
        self.assertTrue((ROOT / "examples/compile_sample.py").exists())
        with tempfile.TemporaryDirectory() as folder:
            run = subprocess.run(
                [sys.executable, str(ROOT / "examples/compile_sample.py"), "--output", folder],
                cwd=ROOT, text=True, capture_output=True,
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            summary = json.loads((Path(folder) / "summary.json").read_text())
            self.assertEqual(summary["route_samples"], 153)
            self.assertEqual(summary["robot_spheres"], 11)
            self.assertEqual(summary["ici_count"], 3)
            self.assertEqual(summary["scope"], "partial_stage_example")
            self.assertIn("proposal_first_active_robot_clearance_m", summary)
            self.assertGreaterEqual(summary["proposal_first_active_robot_clearance_m"], .005)
            self.assertIsNotNone(summary["proposal_nominal_contact_metrics"]["first_dynamic_contact_normal_approach_speed_m_s"])
            self.assertTrue((Path(folder) / "compiled_srir.json").is_file())
            self.assertTrue((Path(folder) / "dynamic_proposal.npz").is_file())


if __name__ == "__main__":
    unittest.main()
