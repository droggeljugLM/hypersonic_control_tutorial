import unittest

from hgv_control.models.point_mass_3d import (
    PointMass3DParams,
    PointMass3DState,
    dynamic_pressure_3d,
    load_factor_3d,
)
from hgv_control.simulation.guidance_3d import GuidanceTarget3D, guidance_command, run


class Guidance3DTests(unittest.TestCase):
    def test_dynamic_pressure_increases_with_speed(self):
        params = PointMass3DParams()
        slow = PointMass3DState(0.0, 0.0, 30_000.0, 1_200.0, 0.0, 0.0)
        fast = PointMass3DState(0.0, 0.0, 30_000.0, 1_600.0, 0.0, 0.0)
        self.assertGreater(dynamic_pressure_3d(fast, params), dynamic_pressure_3d(slow, params))

    def test_guidance_command_respects_rate_limits(self):
        params = PointMass3DParams()
        state = PointMass3DState(0.0, 0.0, 30_000.0, 1_750.0, 0.0, 0.0)
        command = guidance_command(state, target=GuidanceTarget3D(), params=params)
        self.assertLessEqual(abs(command.gamma_rate), params.gamma_rate_limit)
        self.assertLessEqual(abs(command.heading_rate), params.heading_rate_limit)
        self.assertGreaterEqual(command.throttle, params.throttle_min)
        self.assertLessEqual(command.throttle, params.throttle_max)
        self.assertGreater(load_factor_3d(state, command, params), 0.0)

    def test_short_3d_guidance_run_reports_terminal_and_path_metrics(self):
        trace, metrics = run(duration=5.0, dt=0.1)
        self.assertGreater(len(trace), 10)
        self.assertIn("terminal_range_m", metrics)
        self.assertIn("load_factor_max", metrics)
        self.assertTrue(metrics["path_pass"])


if __name__ == "__main__":
    unittest.main()
