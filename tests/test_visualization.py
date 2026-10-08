"""Regression tests for receiver energy and commanded/applied figure data."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "tools/visualization/generate_visualization_assets.py"
SPEC = importlib.util.spec_from_file_location("inoas_visuals", SCRIPT)
visuals = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(visuals)


class VisualizationTests(unittest.TestCase):
    def test_energy_uses_exported_receiver_states(self):
        data = {
            "time": np.array([0., 10., 30., 60.]),
            "receiver_time": np.array([0., 10., 30., 60.]),
            "receiver_mode": np.array([1., 2., 0., 0.]),
            "receiver_energy_wh": np.array([0., 23.4, 59.4, 60.15]) / 3600.,
        }
        energy, powered = visuals.receiver_energy_profile(data)
        np.testing.assert_allclose(energy, [0., 23.4, 59.4, 60.15])
        self.assertAlmostEqual(powered, .5)

    def test_missing_state_is_not_inferred_from_lambda(self):
        data = {
            "time": np.array([0., 60.]), "receiver_time": np.array([]),
            "receiver_mode": np.array([]), "receiver_energy_wh": np.array([]),
            "lambda": np.array([0., 0.]),
        }
        energy, powered = visuals.receiver_energy_profile(data)
        self.assertTrue(np.isnan(energy).all())
        self.assertTrue(np.isnan(powered))

    def test_applied_delta_v_is_preferred_and_labeled(self):
        data = {
            "control_time": np.array([0., 10.]), "control": np.ones((2, 3)),
            "applied_time": np.array([0., 10.]),
            "applied_control": np.array([[.2, 0., 0.], [.2, 0., 0.]]),
        }
        _, dv, label = visuals.maneuver_delta_v(data)
        self.assertEqual(label, "Applied")
        self.assertAlmostEqual(dv[-1], 2.)

    def test_commanded_fallback_is_not_labeled_applied(self):
        data = {
            "control_time": np.array([0., 10.]),
            "control": np.array([[.1, 0., 0.], [.1, 0., 0.]]),
            "applied_time": np.array([]), "applied_control": np.empty((0, 3)),
        }
        _, dv, label = visuals.maneuver_delta_v(data)
        self.assertEqual(label, "Commanded")
        self.assertAlmostEqual(dv[-1], 1.)

    def test_truth_and_estimate_grids_are_aligned(self):
        mat = {
            "time_s": np.array([0., 5., 10.]),
            "truth_eci_m": np.zeros((3, 3)),
            "estimate_time_s": np.array([0., 10.]),
            "estimated_eci_m": np.array([[1., 0., 0.], [3., 0., 0.]]),
            "reference_eci_m": np.ones((3, 3)),
            "reference_velocity_eci_mps": np.zeros((3, 3)),
            "control_time_s": np.array([0., 10.]),
            "control_eci_mps2": np.zeros((2, 3)),
        }
        with patch.object(visuals, "loadmat", return_value=mat):
            data = visuals.load_data(Path("unused.mat"))
        np.testing.assert_allclose(data["estimate"][:, 0], [1., 2., 3.])
        self.assertEqual(data["truth"].shape, data["estimate"].shape)


if __name__ == "__main__":
    unittest.main()
