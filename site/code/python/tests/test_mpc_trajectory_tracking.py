import unittest

from hgv_control.simulation.mpc_trajectory_tracking import run, simulate


class MpcTrajectoryTrackingTests(unittest.TestCase):
    def test_mpc_reduces_q_violation_and_keeps_tracking_feasible(self):
        rows, summary = run()

        self.assertEqual(len(rows), 2)
        self.assertGreater(summary["naive_q_violation_integral"], summary["mpc_q_violation_integral"])
        self.assertGreater(summary["naive_v_rms_m_s"], summary["mpc_v_rms_m_s"])
        self.assertFalse(summary["naive_case_pass"])
        self.assertTrue(summary["mpc_case_pass"])

    def test_mpc_reports_solver_and_feasibility_fields(self):
        row = simulate("finite_set_mpc")

        self.assertEqual(row["prediction_horizon_steps"], 12)
        self.assertIn("solver_time_max_ms", row)
        self.assertIn("fallback_count", row)
        self.assertIn("recursive_feasibility_failures", row)
        self.assertGreater(row["solver_success_rate"], 0.99)
        self.assertTrue(row["realtime_pass"])


if __name__ == "__main__":
    unittest.main()
