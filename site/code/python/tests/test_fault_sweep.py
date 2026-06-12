import unittest

from hgv_control.simulation.fault_sweep import paired_fault_deltas, run_sweep, summarize_pairs


class FaultSweepTests(unittest.TestCase):
    def test_sweep_runs_controller_fault_matrix(self):
        rows = run_sweep(
            ["baseline", "guarded"],
            ["none", "alpha_sensor_bias"],
            duration=0.6,
            dt=0.05,
        )

        self.assertEqual(len(rows), 4)
        self.assertIn("fault_detection_delay", rows[0])
        self.assertIn("post_fault_q_violation_integral", rows[0])

    def test_paired_fault_deltas_include_candidate_minus_baseline(self):
        rows = run_sweep(
            ["baseline", "guarded"],
            ["none", "elevator_loss"],
            duration=0.6,
            dt=0.05,
        )
        pairs = paired_fault_deltas(rows, baseline="baseline", candidate="guarded")
        summary = summarize_pairs(pairs)

        self.assertEqual(len(pairs), 2)
        self.assertIn("delta_post_fault_q_violation_integral", pairs[0])
        self.assertEqual(summary["pairs"], 2.0)


if __name__ == "__main__":
    unittest.main()
