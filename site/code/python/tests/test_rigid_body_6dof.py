import math
import unittest

from hgv_control.models.rigid_body_6dof import (
    BodyForceMoment,
    RigidBody6DofParams,
    body_rate_derivative,
    euler_step,
    kinematics_health,
    make_level_initial_state,
)
from hgv_control.simulation.six_dof_rigid_body_demo import run


class RigidBody6DofTests(unittest.TestCase):
    def test_weight_balanced_body_z_force_holds_vertical_velocity(self):
        params = RigidBody6DofParams()
        state = make_level_initial_state()
        command = BodyForceMoment(0.0, 0.0, params.mass * params.gravity, 0.0, 0.0, 0.0)

        next_state = euler_step(state, command, 0.1, params)

        self.assertAlmostEqual(next_state.velocity_up, 0.0, places=9)
        self.assertAlmostEqual(next_state.velocity_east, state.velocity_east, places=9)

    def test_positive_body_x_force_accelerates_east_at_identity_attitude(self):
        params = RigidBody6DofParams()
        state = make_level_initial_state(velocity_east=0.0)
        command = BodyForceMoment(1200.0, 0.0, params.mass * params.gravity, 0.0, 0.0, 0.0)

        next_state = euler_step(state, command, 1.0, params)

        self.assertGreater(next_state.velocity_east, 0.9)
        self.assertAlmostEqual(next_state.velocity_north, 0.0, places=9)

    def test_positive_pitch_moment_increases_q_rate(self):
        params = RigidBody6DofParams()
        rate_dot = body_rate_derivative((0.0, 0.0, 0.0), (0.0, 520.0, 0.0), params)
        self.assertGreater(rate_dot[1], 0.0)

    def test_repeated_steps_keep_quaternion_and_dcm_healthy(self):
        params = RigidBody6DofParams()
        state = make_level_initial_state()
        command = BodyForceMoment(0.0, 0.0, params.mass * params.gravity, 120.0, 160.0, 140.0)
        for _ in range(80):
            state = euler_step(state, command, 0.05, params)
        quaternion_error, dcm_error = kinematics_health(state)
        self.assertLess(quaternion_error, 1e-12)
        self.assertLess(dcm_error, 1e-12)

    def test_demo_reports_teaching_metrics(self):
        trace, metrics = run(duration=3.0, dt=0.05)
        self.assertGreater(len(trace), 50)
        self.assertIn("body_rate_max_rad_s", metrics)
        self.assertIn("quaternion_norm_error_max", metrics)
        self.assertTrue(metrics["pass"])
        self.assertTrue(math.isfinite(metrics["speed_change_m_s"]))


if __name__ == "__main__":
    unittest.main()
