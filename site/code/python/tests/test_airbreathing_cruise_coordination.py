import unittest

from hgv_control.simulation.airbreathing_cruise_coordination import (
    CruiseCoordinationConfig,
    CruiseState,
    estimate_inlet_margin,
    heat_proxy,
    make_initial_state,
    run,
    simulate,
)
from hgv_control.models.longitudinal import LongitudinalParams, dynamic_pressure


class AirbreathingCruiseCoordinationTests(unittest.TestCase):
    def test_inlet_margin_proxy_decreases_with_alpha_and_throttle(self):
        params = LongitudinalParams(thrust_max=23_000.0, throttle_max=0.92)
        config = CruiseCoordinationConfig()
        state = make_initial_state()
        qbar = dynamic_pressure(
            CruiseState(
                velocity=state.velocity,
                altitude=state.altitude,
                gamma=state.gamma,
                alpha=state.alpha,
                pitch_rate=state.pitch_rate,
                theta=state.theta,
                elevator=state.elevator,
                elevator_rate=state.elevator_rate,
                throttle_actual=state.throttle_actual,
            ),
            params,
        )
        thermal = heat_proxy(state, params)
        nominal = estimate_inlet_margin(qbar, state.alpha, 0.48, 0.02, thermal, params, config)
        stressed = estimate_inlet_margin(qbar, state.alpha + 0.08, 0.82, 0.18, thermal, params, config)

        self.assertGreater(nominal, stressed)

    def test_cruise_guard_improves_inlet_margin_and_passes_case(self):
        rows, summary = run()

        self.assertEqual(len(rows), 2)
        self.assertGreater(summary["guarded_inlet_margin_min"], summary["baseline_inlet_margin_min"])
        self.assertGreater(summary["baseline_q_violation_integral"], summary["guarded_q_violation_integral"])
        self.assertFalse(summary["baseline_case_pass"])
        self.assertTrue(summary["guarded_case_pass"])

    def test_guarded_case_reports_coordination_fields(self):
        _trace, metrics = simulate("cruise_guard")

        self.assertIn("inlet_margin_min", metrics)
        self.assertIn("inlet_margin_active_time", metrics)
        self.assertIn("propulsion_limit_active_time", metrics)
        self.assertIn("throttle_rate_peak", metrics)
        self.assertTrue(metrics["inlet_pass"])
        self.assertTrue(metrics["input_pass"])


if __name__ == "__main__":
    unittest.main()
