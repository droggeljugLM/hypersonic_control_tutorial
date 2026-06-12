import unittest

from hgv_control.simulation.reentry_heat_rate_control import (
    ReentryParams,
    ReentryState,
    command_heat_guard,
    dynamic_pressure,
    heat_rate,
    run,
    simulate,
)


class ReentryHeatRateControlTests(unittest.TestCase):
    def test_heat_rate_increases_with_density_and_speed(self):
        params = ReentryParams()
        high = ReentryState(0.0, 36_000.0, 2_900.0, 0.0)
        low = ReentryState(0.0, 54_000.0, 2_400.0, 0.0)

        self.assertGreater(heat_rate(high, params), heat_rate(low, params))
        self.assertGreater(dynamic_pressure(high, params), dynamic_pressure(low, params))

    def test_heat_guard_command_respects_rate_limit(self):
        params = ReentryParams()
        state = ReentryState(80_000.0, 36_000.0, 2_850.0, -0.08)
        command = command_heat_guard(state, 30.0, params)

        self.assertLessEqual(abs(command), params.gamma_rate_limit)

    def test_guarded_reentry_reduces_heat_violation_and_passes_case(self):
        rows, summary = run()

        self.assertEqual(len(rows), 2)
        self.assertGreater(
            summary["baseline_heat_rate_violation_integral"],
            summary["guarded_heat_rate_violation_integral"],
        )
        self.assertFalse(summary["baseline_case_pass"])
        self.assertTrue(summary["guarded_case_pass"])

    def test_guarded_reentry_reports_constraint_fields(self):
        _trace, metrics = simulate("heat_guard")

        self.assertIn("heat_rate_max", metrics)
        self.assertIn("heat_load", metrics)
        self.assertIn("corridor_margin_min", metrics)
        self.assertIn("load_factor_max", metrics)
        self.assertTrue(metrics["heat_pass"])
        self.assertTrue(metrics["corridor_pass"])


if __name__ == "__main__":
    unittest.main()
