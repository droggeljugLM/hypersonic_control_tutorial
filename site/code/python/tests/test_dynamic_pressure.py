import unittest

from hgv_control.models.longitudinal import LongitudinalParams, State, dynamic_pressure
from hgv_control.metrics.summary import summarize, violation_integral


class DynamicPressureTests(unittest.TestCase):
    def test_dynamic_pressure_increases_with_velocity(self):
        params = LongitudinalParams()
        slow = State(velocity=1000.0, altitude=30000.0, gamma=0.0, alpha=0.0, pitch_rate=0.0, theta=0.0, elevator=0.0, elevator_rate=0.0)
        fast = State(velocity=1500.0, altitude=30000.0, gamma=0.0, alpha=0.0, pitch_rate=0.0, theta=0.0, elevator=0.0, elevator_rate=0.0)
        self.assertGreater(dynamic_pressure(fast, params), dynamic_pressure(slow, params))

    def test_dynamic_pressure_decreases_with_altitude(self):
        params = LongitudinalParams()
        low = State(velocity=1500.0, altitude=25000.0, gamma=0.0, alpha=0.0, pitch_rate=0.0, theta=0.0, elevator=0.0, elevator_rate=0.0)
        high = State(velocity=1500.0, altitude=35000.0, gamma=0.0, alpha=0.0, pitch_rate=0.0, theta=0.0, elevator=0.0, elevator_rate=0.0)
        self.assertGreater(dynamic_pressure(low, params), dynamic_pressure(high, params))

    def test_violation_integral_uses_excess_only(self):
        self.assertEqual(violation_integral([1.0, 3.0, 5.0], 3.0, 0.5), 1.0)

    def test_summary_pass_requires_actuator_and_alpha_limits(self):
        trace = [
            {
                "altitude": 0.0,
                "altitude_ref": 0.0,
                "velocity": 0.0,
                "velocity_ref": 0.0,
                "qbar": 1.0,
                "elevator": 0.1,
                "elevator_rate": 0.1,
                "alpha": 0.5,
                "throttle_cmd": 0.5,
            }
        ]
        metrics = summarize(trace, q_limit=10.0, dt=0.1, alpha_limit=0.2, delta_limit=0.2, delta_rate_limit=0.2)
        self.assertFalse(metrics["alpha_pass"])
        self.assertFalse(metrics["pass"])


if __name__ == "__main__":
    unittest.main()
