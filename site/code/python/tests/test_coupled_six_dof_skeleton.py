import math
import unittest

from hgv_control.models.attitude import AttitudeState
from hgv_control.models.point_mass_3d import PointMass3DParams, PointMass3DState
from hgv_control.simulation.coupled_six_dof_skeleton import achieved_point_command, run
from hgv_control.simulation.guidance_attitude_interface import InterfaceParams


class CoupledSixDofSkeletonTests(unittest.TestCase):
    def test_achieved_command_uses_attitude(self):
        point = PointMass3DState(0.0, 0.0, 30_000.0, 1_700.0, 0.0, 0.0)
        attitude = AttitudeState(
            phi=math.radians(8.0),
            theta=math.radians(4.0),
            psi=math.radians(5.0),
            p=0.0,
            q=0.0,
            r=0.0,
            mx=0.0,
            my=0.0,
            mz=0.0,
        )
        command = achieved_point_command(point, attitude, 0.4, PointMass3DParams(), InterfaceParams())
        self.assertGreater(command.heading_rate, 0.0)
        self.assertGreater(command.gamma_rate, 0.0)

    def test_coupled_run_reports_closed_chain_metrics(self):
        trace, metrics = run(duration=8.0, dt=0.1)
        self.assertGreater(len(trace), 50)
        self.assertIn("gamma_rate_error_rms_rad_s", metrics)
        self.assertIn("allocation_residual_rms_nm", metrics)
        self.assertIn("aero_moment_error_rms_nm", metrics)
        self.assertIn("aero_force_projection_rms_n", metrics)
        self.assertIn("dcm_force_delta_rms_n", metrics)
        self.assertIn("quaternion_norm_error_max", metrics)
        self.assertIn("dcm_orthogonality_error_max", metrics)
        self.assertIn("alpha_rad", trace[0])
        self.assertIn("mx_aero", trace[0])
        self.assertIn("force_normal", trace[0])
        self.assertIn("body_force_x", trace[0])
        self.assertIn("dcm_force_delta_norm", trace[0])
        self.assertIn("quaternion_norm_error", trace[0])
        self.assertIn("dcm_orthogonality_error", trace[0])
        self.assertTrue(metrics["path_pass"])
        self.assertTrue(metrics["allocation_pass"])
        self.assertTrue(metrics["aero_feedback_pass"])
        self.assertTrue(metrics["kinematics_pass"])
        self.assertTrue(metrics["force_transform_pass"])


if __name__ == "__main__":
    unittest.main()
