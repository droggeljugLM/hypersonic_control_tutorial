import math
import unittest

from hgv_control.models.flexible_pitch import (
    FlexiblePitchParams,
    FlexiblePitchState,
    input_effectiveness,
    measured_pitch_rate,
    modal_energy,
    rk4_step,
)
from hgv_control.simulation.flexible_pitch_demo import run, run_sweep


class FlexiblePitchTests(unittest.TestCase):
    def test_measured_rate_contains_modal_contamination(self):
        params = FlexiblePitchParams(sensor_modal_rate_gain=0.5)
        state = FlexiblePitchState(theta=0.0, q=0.1, moment=0.0, eta=0.0, eta_dot=0.2)
        self.assertAlmostEqual(measured_pitch_rate(state, params), 0.2)

    def test_input_effectiveness_is_bounded(self):
        params = FlexiblePitchParams(effectiveness_modal_gain=10.0, min_effectiveness=0.65)
        state = FlexiblePitchState(theta=0.0, q=0.0, moment=0.0, eta=0.2, eta_dot=0.0)
        self.assertEqual(input_effectiveness(state, params), 0.65)

    def test_rk4_step_excites_flexible_mode_under_moment(self):
        params = FlexiblePitchParams()
        state = FlexiblePitchState(theta=0.0, q=0.0, moment=0.0, eta=0.0, eta_dot=0.0)
        next_state = rk4_step(state, 1_000.0, 0.05, params)
        self.assertGreater(next_state.moment, 0.0)
        self.assertGreaterEqual(modal_energy(next_state, params), 0.0)

    def test_demo_reports_all_controller_metrics(self):
        _traces, metrics = run()
        names = {row["controller"] for row in metrics}
        self.assertEqual(names, {"rigid_rate_pd", "flex_sensor_pd", "band_limited_pd"})
        for row in metrics:
            self.assertIn("modal_energy_max", row)
            self.assertIn("sensor_pollution_rms_rad_s", row)
            self.assertTrue(math.isfinite(float(row["theta_rms_rad"])))

    def test_band_limited_controller_reduces_modal_and_actuator_metrics(self):
        _traces, metrics = run()
        rows = {row["controller"]: row for row in metrics}
        self.assertLess(
            rows["band_limited_pd"]["modal_energy_max"],
            rows["flex_sensor_pd"]["modal_energy_max"],
        )
        self.assertLess(
            rows["band_limited_pd"]["moment_rate_rms_nm_s"],
            rows["flex_sensor_pd"]["moment_rate_rms_nm_s"],
        )

    def test_sweep_records_parameters_and_failure_samples(self):
        rows = run_sweep()
        self.assertEqual(len(rows), 36)
        self.assertTrue(any(bool(row["case_pass"]) for row in rows))
        self.assertTrue(any(not bool(row["case_pass"]) for row in rows))
        first = rows[0]
        self.assertIn("sweep_scenario", first)
        self.assertIn("flexible_frequency_rad_s", first)
        self.assertIn("flexible_damping_ratio", first)
        self.assertIn("sensor_modal_rate_gain", first)


if __name__ == "__main__":
    unittest.main()
