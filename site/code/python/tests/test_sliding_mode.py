import math
import unittest

from hgv_control.controllers.sliding_mode import SlidingModeController, sat
from hgv_control.models.longitudinal import LongitudinalParams
from hgv_control.simulation.run_case import make_initial_state, run


class SlidingModeTests(unittest.TestCase):
    def test_saturation_bounds_switching_term(self):
        self.assertEqual(sat(2.0), 1.0)
        self.assertEqual(sat(-2.0), -1.0)
        self.assertEqual(sat(0.25), 0.25)

    def test_smc_command_respects_limits(self):
        params = LongitudinalParams()
        controller = SlidingModeController()
        delta_cmd, throttle_cmd = controller.command(make_initial_state(), 30_000.0, 1_750.0, params)
        self.assertLessEqual(abs(delta_cmd), params.delta_limit)
        self.assertGreaterEqual(throttle_cmd, params.throttle_min)
        self.assertLessEqual(throttle_cmd, params.throttle_max)

    def test_smc_short_simulation_runs(self):
        _, metrics = run("smc", duration=1.0, dt=0.05)
        self.assertIn("q_violation_integral", metrics)
        self.assertLess(float(metrics["alpha_max_rad"]), math.radians(18.0))


if __name__ == "__main__":
    unittest.main()
