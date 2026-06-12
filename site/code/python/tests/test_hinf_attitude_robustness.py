import unittest

from hgv_control.simulation.hinf_attitude_robustness import run


class HinfAttitudeRobustnessTests(unittest.TestCase):
    def test_weighted_design_reduces_worst_frequency_and_saturation_metrics(self):
        rows, summary = run()

        self.assertEqual(len(rows), 10)
        self.assertLess(
            summary["hinf_worst_mixed_sensitivity_peak"],
            summary["baseline_worst_mixed_sensitivity_peak"],
        )
        self.assertLess(
            summary["hinf_worst_saturation_fraction"],
            summary["baseline_worst_saturation_fraction"],
        )
        self.assertFalse(summary["baseline_all_pass"])
        self.assertTrue(summary["hinf_all_pass"])

    def test_rows_record_uncertainty_weights_and_time_metrics(self):
        rows, summary = run()
        first = rows[0]

        self.assertIn("plant_description", first)
        self.assertIn("mixed_sensitivity_peak", first)
        self.assertIn("robust_stability_peak", first)
        self.assertIn("attitude_rms_rad", first)
        self.assertGreater(summary["hinf_selected_kp"], 0.0)
        self.assertGreater(summary["hinf_selected_kd"], 0.0)


if __name__ == "__main__":
    unittest.main()
