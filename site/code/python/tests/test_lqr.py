import unittest

from hgv_control.controllers.lqr import LqrController, inverse, linearize_continuous
from hgv_control.models.longitudinal import LongitudinalParams
from hgv_control.simulation.run_case import make_initial_state, run


class LqrTests(unittest.TestCase):
    def test_inverse_2x2(self):
        inv = inverse([[4.0, 7.0], [2.0, 6.0]])
        self.assertAlmostEqual(inv[0][0], 0.6)
        self.assertAlmostEqual(inv[0][1], -0.7)
        self.assertAlmostEqual(inv[1][0], -0.2)
        self.assertAlmostEqual(inv[1][1], 0.4)

    def test_linearization_dimensions(self):
        params = LongitudinalParams()
        state = make_initial_state()
        a, b = linearize_continuous(
            state,
            [0.0, 0.62],
            params,
            state_steps=[1.0, 10.0, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4],
            control_steps=[1e-4, 1e-4],
        )
        self.assertEqual(len(a), 8)
        self.assertEqual(len(a[0]), 8)
        self.assertEqual(len(b), 8)
        self.assertEqual(len(b[0]), 2)

    def test_lqr_command_respects_limits(self):
        params = LongitudinalParams()
        controller = LqrController()
        delta_cmd, throttle_cmd = controller.command(make_initial_state(), 30_000.0, 1_750.0, params)
        self.assertLessEqual(abs(delta_cmd), params.delta_limit)
        self.assertGreaterEqual(throttle_cmd, params.throttle_min)
        self.assertLessEqual(throttle_cmd, params.throttle_max)

    def test_lqr_short_simulation_runs(self):
        _, metrics = run("lqr", duration=1.0, dt=0.05)
        self.assertIn("q_violation_integral", metrics)


if __name__ == "__main__":
    unittest.main()

