import unittest

from hgv_control.simulation.monte_carlo import run_monte_carlo, summarize_rows


class MonteCarloTests(unittest.TestCase):
    def test_row_count_matches_samples_and_controllers(self):
        rows = run_monte_carlo(samples=2, controllers=["baseline", "guarded"], duration=1.0, dt=0.05)
        self.assertEqual(len(rows), 4)

    def test_summary_has_one_row_per_controller(self):
        rows = run_monte_carlo(samples=1, controllers=["baseline", "guarded"], duration=1.0, dt=0.05)
        summary = summarize_rows(rows)
        self.assertEqual({row["controller"] for row in summary}, {"baseline", "guarded"})


if __name__ == "__main__":
    unittest.main()
