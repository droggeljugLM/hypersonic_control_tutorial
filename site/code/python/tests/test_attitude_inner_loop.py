import math
import unittest

from hgv_control.models.attitude import (
    AttitudeParams,
    AttitudeState,
    MomentCommand,
    attitude_error,
    limit_moment_command,
    rk4_step,
)
from hgv_control.simulation.attitude_inner_loop import pd_moment_command, run


class AttitudeInnerLoopTests(unittest.TestCase):
    def test_attitude_error_wraps_yaw(self):
        state = AttitudeState(0.0, 0.0, math.radians(179.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        _e_phi, _e_theta, e_psi = attitude_error(state, 0.0, 0.0, math.radians(-179.0))
        self.assertAlmostEqual(e_psi, math.radians(2.0), places=6)

    def test_moment_command_respects_limits(self):
        params = AttitudeParams(moment_limit=100.0)
        command = limit_moment_command(MomentCommand(200.0, -300.0, 50.0), params)
        self.assertEqual(command.mx, 100.0)
        self.assertEqual(command.my, -100.0)
        self.assertEqual(command.mz, 50.0)

    def test_rk4_step_moves_toward_roll_command(self):
        params = AttitudeParams()
        state = AttitudeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        command = MomentCommand(1_000.0, 0.0, 0.0)
        next_state = rk4_step(state, command, 0.05, params)
        self.assertGreater(next_state.mx, 0.0)

    def test_pd_moment_command_reacts_against_rate(self):
        params = AttitudeParams()
        state = AttitudeState(0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
        command = pd_moment_command(state, (0.0, 0.0, 0.0), params)
        self.assertLess(command.mx, 0.0)

    def test_attitude_run_reports_passing_metrics(self):
        trace, metrics = run(duration=8.0, dt=0.04)
        self.assertGreater(len(trace), 100)
        self.assertIn("attitude_rms_rad", metrics)
        self.assertTrue(metrics["moment_pass"])
        self.assertTrue(metrics["moment_rate_pass"])


if __name__ == "__main__":
    unittest.main()
