import math
import unittest

from hgv_control.simulation.fault_injection import FaultConfig, run


class FaultInjectionTests(unittest.TestCase):
    def test_alpha_sensor_bias_is_detected_and_recorded(self):
        _, metrics = run(
            "guarded",
            FaultConfig(fault_type="alpha_sensor_bias", start_time=0.1, magnitude=math.radians(2.0)),
            duration=1.0,
            dt=0.05,
        )

        self.assertTrue(metrics["fault_detected"])
        self.assertEqual(metrics["missed_fault_count"], 0.0)
        self.assertGreater(float(metrics["alpha_est_rms"]), 0.0)

    def test_elevator_loss_changes_actual_command_and_keeps_metrics(self):
        trace, metrics = run(
            "baseline",
            FaultConfig(fault_type="elevator_loss", start_time=0.0, magnitude=0.5, confirm_steps=1),
            duration=1.0,
            dt=0.05,
        )

        self.assertIn("post_fault_h_rms", metrics)
        self.assertIn("q_est_margin_min", metrics)
        self.assertTrue(any(abs(row["delta_cmd"] - row["delta_actual"]) > 1e-6 for row in trace))

    def test_no_fault_has_no_detection_or_estimation_error(self):
        _, metrics = run(
            "guarded",
            FaultConfig(fault_type="none", start_time=0.0, magnitude=0.0),
            duration=0.5,
            dt=0.05,
        )

        self.assertFalse(metrics["fault_detected"])
        self.assertEqual(metrics["false_alarm_count"], 0.0)
        self.assertEqual(metrics["altitude_est_rms"], 0.0)


if __name__ == "__main__":
    unittest.main()
