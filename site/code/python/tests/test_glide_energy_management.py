import unittest

from hgv_control.simulation.glide_energy_management import (
    GlideParams,
    GlideState,
    baseline_command,
    dynamic_pressure,
    energy_guard_command,
    energy_height,
    run,
    simulate,
)


class GlideEnergyManagementTests(unittest.TestCase):
    def test_energy_height_increases_with_altitude_and_speed(self):
        params = GlideParams()
        low = GlideState(0.0, 0.0, 30_000.0, 1_500.0, 0.0, 0.0, 0.0)
        high = GlideState(0.0, 0.0, 35_000.0, 1_900.0, 0.0, 0.0, 0.0)

        self.assertGreater(energy_height(high, params), energy_height(low, params))
        self.assertGreater(dynamic_pressure(high, params), 0.0)

    def test_energy_guard_command_respects_limits(self):
        params = GlideParams()
        state = GlideState(90_000.0, 18_000.0, 33_000.0, 1_850.0, -0.05, 0.0, 0.0)
        command = energy_guard_command(state, params)

        self.assertGreaterEqual(command.alpha, params.alpha_min)
        self.assertLessEqual(command.alpha, params.alpha_max)
        self.assertLessEqual(abs(command.bank_cmd), params.bank_limit)
        self.assertLessEqual(abs(baseline_command(state, params).bank_cmd), params.bank_limit)

    def test_energy_guard_reduces_terminal_energy_error_and_passes_case(self):
        rows, summary = run()

        self.assertEqual(len(rows), 2)
        self.assertGreater(
            abs(summary["baseline_terminal_energy_error_m"]),
            abs(summary["guarded_terminal_energy_error_m"]),
        )
        self.assertFalse(summary["baseline_case_pass"])
        self.assertTrue(summary["guarded_case_pass"])

    def test_energy_guard_reports_energy_and_path_fields(self):
        _trace, metrics = simulate("energy_guard")

        self.assertIn("terminal_energy_error_m", metrics)
        self.assertIn("energy_corridor_margin_min", metrics)
        self.assertIn("crossrange_error_m", metrics)
        self.assertIn("bank_angle_peak_rad", metrics)
        self.assertTrue(metrics["energy_pass"])
        self.assertTrue(metrics["path_pass"])


if __name__ == "__main__":
    unittest.main()
