import math
import unittest

from hgv_control.models.aero_table import (
    AeroTableParams,
    angles_from_attitude,
    default_tables,
    evaluate_aero,
    interpolate_alpha_mach,
)
from hgv_control.models.attitude import AttitudeState
from hgv_control.models.control_allocation import AllocatorState
from hgv_control.models.point_mass_3d import PointMass3DState
from hgv_control.simulation.aero_table_demo import run


class AeroTableTests(unittest.TestCase):
    def test_interpolation_returns_grid_value_at_node(self):
        params = AeroTableParams()
        clift_table, _drag_table, _cm_table = default_tables(params)
        value = interpolate_alpha_mach(clift_table, params.mach_grid[1], params.alpha_grid_rad[2], params)
        self.assertAlmostEqual(value, clift_table[1][2])

    def test_lift_increases_with_alpha(self):
        low = evaluate_aero(7.0, 50_000.0, math.radians(2.0), 0.0)
        high = evaluate_aero(7.0, 50_000.0, math.radians(10.0), 0.0)
        self.assertGreater(high.coefficients.clift, low.coefficients.clift)
        self.assertGreater(high.loads.lift, low.loads.lift)

    def test_sideslip_and_rudder_affect_lateral_channels(self):
        beta = evaluate_aero(7.0, 50_000.0, math.radians(4.0), math.radians(5.0))
        rudder = evaluate_aero(7.0, 50_000.0, math.radians(4.0), 0.0, AllocatorState(rudder=math.radians(5.0)))
        self.assertNotAlmostEqual(beta.coefficients.cy, 0.0)
        self.assertGreater(rudder.coefficients.cn_yaw, 0.0)

    def test_angles_from_attitude_uses_flight_path_and_heading(self):
        point = PointMass3DState(0.0, 0.0, 30_000.0, 1_700.0, math.radians(2.0), math.radians(10.0))
        attitude = AttitudeState(
            phi=0.0,
            theta=math.radians(7.0),
            psi=math.radians(13.0),
            p=0.0,
            q=0.0,
            r=0.0,
            mx=0.0,
            my=0.0,
            mz=0.0,
        )
        angles = angles_from_attitude(point, attitude)
        self.assertAlmostEqual(angles.alpha, math.radians(5.0))
        self.assertGreater(angles.beta, 0.0)

    def test_demo_reports_passing_metrics(self):
        trace, metrics = run(duration=6.0, dt=0.1)
        self.assertGreater(len(trace), 50)
        self.assertIn("clift_alpha_slope_per_rad", metrics)
        self.assertTrue(metrics["lift_slope_pass"])
        self.assertTrue(metrics["drag_pass"])
        self.assertTrue(metrics["pass"])


if __name__ == "__main__":
    unittest.main()
