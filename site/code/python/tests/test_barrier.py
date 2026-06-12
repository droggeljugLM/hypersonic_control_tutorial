import math
import unittest

from hgv_control.controllers.barrier import BarrierController, dynamic_pressure_rate
from hgv_control.controllers.pid import BaselineController
from hgv_control.models.longitudinal import LongitudinalParams, dynamic_pressure
from hgv_control.simulation.run_case import make_initial_state, run


class BarrierControllerTests(unittest.TestCase):
    def test_barrier_is_inactive_below_warning_when_qdot_is_safe(self):
        params = LongitudinalParams()
        state = make_initial_state()
        controller = BarrierController()
        nominal_delta, nominal_throttle = BaselineController().command(state, 30_000.0, 1_750.0, params)
        safe_delta, safe_throttle = controller.command(state, 30_000.0, 1_750.0, params)
        self.assertEqual(safe_delta, nominal_delta)
        self.assertEqual(safe_throttle, nominal_throttle)

    def test_barrier_cuts_throttle_and_unloads_elevator_near_limit(self):
        params = LongitudinalParams()
        state = make_initial_state(velocity=2_000.0, altitude=25_000.0)
        self.assertGreater(dynamic_pressure(state, params), params.q_warning)

        nominal_delta, nominal_throttle = BaselineController().command(state, 30_000.0, 1_750.0, params)
        safe_delta, safe_throttle = BarrierController().command(state, 30_000.0, 1_750.0, params)

        self.assertLessEqual(safe_throttle, nominal_throttle)
        self.assertLessEqual(safe_delta, nominal_delta)
        self.assertLessEqual(abs(safe_delta), params.delta_limit)
        self.assertGreaterEqual(safe_throttle, params.throttle_min)
        self.assertLessEqual(safe_throttle, params.throttle_max)

    def test_dynamic_pressure_rate_is_finite(self):
        params = LongitudinalParams()
        state = make_initial_state(velocity=1_900.0, altitude=25_000.0)
        qdot = dynamic_pressure_rate(state, math.radians(1.0), 0.2, params)
        self.assertTrue(math.isfinite(qdot))

    def test_barrier_short_simulation_runs(self):
        _, metrics = run("barrier", duration=1.0, dt=0.05)
        self.assertIn("q_violation_integral", metrics)
        self.assertLess(float(metrics["alpha_max_rad"]), math.radians(18.0))


if __name__ == "__main__":
    unittest.main()
