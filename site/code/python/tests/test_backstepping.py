import math
import unittest

from hgv_control.controllers.backstepping import BacksteppingController
from hgv_control.models.longitudinal import LongitudinalParams
from hgv_control.simulation.run_case import make_initial_state, run


class BacksteppingTests(unittest.TestCase):
    def test_backstepping_command_respects_limits(self):
        params = LongitudinalParams()
        controller = BacksteppingController()
        delta_cmd, throttle_cmd = controller.command(make_initial_state(), 30_000.0, 1_750.0, params)
        self.assertLessEqual(abs(delta_cmd), params.delta_limit)
        self.assertGreaterEqual(throttle_cmd, params.throttle_min)
        self.assertLessEqual(throttle_cmd, params.throttle_max)

    def test_backstepping_short_simulation_runs(self):
        _, metrics = run("backstepping", duration=1.0, dt=0.05)
        self.assertIn("q_violation_integral", metrics)
        self.assertLess(float(metrics["alpha_max_rad"]), math.radians(18.0))


if __name__ == "__main__":
    unittest.main()
