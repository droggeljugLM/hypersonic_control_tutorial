import unittest

from hgv_control.simulation.indi_attitude_control import run, simulate


class IndiAttitudeControlTests(unittest.TestCase):
    def test_indi_reduces_tracking_error_and_saturation(self):
        rows, summary = run()

        self.assertEqual(len(rows), 2)
        self.assertLess(summary["indi_attitude_rms_rad"], summary["pd_attitude_rms_rad"])
        self.assertLess(summary["indi_post_disturbance_rms_rad"], summary["pd_post_disturbance_rms_rad"])
        self.assertLess(summary["indi_saturation_fraction"], summary["pd_saturation_fraction"])
        self.assertFalse(summary["pd_case_pass"])
        self.assertTrue(summary["indi_case_pass"])

    def test_indi_reports_acceleration_feedback_metrics(self):
        row = simulate("indi")

        self.assertIn("accel_estimation_error_rms_rad_s2", row)
        self.assertIn("command_delay_steps", row)
        self.assertGreater(row["command_increment_rms_nm"], 0.0)
        self.assertTrue(row["acceleration_feedback_pass"])


if __name__ == "__main__":
    unittest.main()
