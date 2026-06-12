import math
import unittest

from hgv_control.models.point_mass_3d import PointMass3DParams, PointMass3DState
from hgv_control.simulation.guidance_3d import GuidanceTarget3D
from hgv_control.simulation.guidance_attitude_interface import (
    InterfaceParams,
    guidance_to_attitude_reference,
    make_attitude_state,
    run,
)


class GuidanceAttitudeInterfaceTests(unittest.TestCase):
    def test_reference_respects_roll_and_pitch_limits(self):
        point = PointMass3DState(0.0, 0.0, 30_000.0, 1_750.0, math.radians(-1.0), 0.0)
        interface_params = InterfaceParams(bank_limit=math.radians(10.0), pitch_limit=math.radians(12.0))
        command = guidance_to_attitude_reference(point, GuidanceTarget3D(), PointMass3DParams(), interface_params)
        self.assertLessEqual(abs(command.roll_ref), interface_params.bank_limit)
        self.assertLessEqual(abs(command.pitch_ref), interface_params.pitch_limit)

    def test_initial_attitude_uses_gamma_and_trim(self):
        point = PointMass3DState(0.0, 0.0, 30_000.0, 1_750.0, math.radians(-1.0), math.radians(3.0))
        state = make_attitude_state(point, InterfaceParams(alpha_trim=math.radians(2.0)))
        self.assertAlmostEqual(state.theta, math.radians(1.0), places=6)
        self.assertAlmostEqual(state.psi, math.radians(3.0), places=6)

    def test_interface_run_reports_contract_metrics(self):
        trace, metrics = run(duration=8.0, dt=0.1)
        self.assertGreater(len(trace), 50)
        self.assertIn("attitude_interface_rms_rad", metrics)
        self.assertTrue(metrics["command_pass"])
        self.assertTrue(metrics["path_pass"])


if __name__ == "__main__":
    unittest.main()
