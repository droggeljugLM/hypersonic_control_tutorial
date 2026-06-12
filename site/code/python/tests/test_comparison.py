import unittest

from hgv_control.metrics.comparison import paired_deltas, summarize_pairs


class ComparisonTests(unittest.TestCase):
    def test_paired_deltas_compute_candidate_minus_baseline(self):
        rows = [
            {
                "case_id": 0,
                "seed": 1,
                "controller": "baseline",
                "pass": False,
                "h_rms": 10.0,
                "v_rms": 20.0,
                "q_max": 100.0,
                "q_violation_integral": 30.0,
            },
            {
                "case_id": 0,
                "seed": 1,
                "controller": "guarded",
                "pass": True,
                "h_rms": 12.0,
                "v_rms": 18.0,
                "q_max": 90.0,
                "q_violation_integral": 0.0,
            },
        ]
        pairs = paired_deltas(rows)
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0]["delta_q_violation_integral"], -30.0)
        self.assertEqual(pairs[0]["delta_q_max"], -10.0)
        self.assertTrue(pairs[0]["safety_improved"])

    def test_pair_summary_counts_rates(self):
        pairs = [
            {"candidate_pass": True, "baseline_pass": False, "safety_improved": True, "delta_q_violation_integral": -1.0, "delta_q_max": -2.0, "delta_h_rms": 3.0, "delta_v_rms": -4.0},
            {"candidate_pass": False, "baseline_pass": False, "safety_improved": False, "delta_q_violation_integral": 2.0, "delta_q_max": 1.0, "delta_h_rms": 5.0, "delta_v_rms": 6.0},
        ]
        summary = summarize_pairs(pairs)
        self.assertEqual(summary["pairs"], 2)
        self.assertEqual(summary["candidate_pass_rate"], 0.5)
        self.assertEqual(summary["baseline_pass_rate"], 0.0)
        self.assertEqual(summary["safety_improved_rate"], 0.5)


if __name__ == "__main__":
    unittest.main()

