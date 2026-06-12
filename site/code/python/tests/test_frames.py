import math
import unittest

from hgv_control.models.frames import (
    dot,
    flight_path_basis,
    force_path_rates,
    norm,
    wind_forces_to_enu,
)


class FrameUtilityTests(unittest.TestCase):
    def test_flight_path_basis_is_orthonormal(self):
        basis = flight_path_basis(math.radians(5.0), math.radians(30.0))
        for vector in basis:
            self.assertAlmostEqual(norm(vector), 1.0)
        self.assertAlmostEqual(dot(basis[0], basis[1]), 0.0)
        self.assertAlmostEqual(dot(basis[0], basis[2]), 0.0)
        self.assertAlmostEqual(dot(basis[1], basis[2]), 0.0)

    def test_lift_increases_gamma_rate_when_level(self):
        force = wind_forces_to_enu(lift=12_000.0, drag=0.0, side_force=0.0, gamma=0.0, heading=0.0, bank=0.0)
        _speed_dot, gamma_rate, heading_rate = force_path_rates(force, mass=1_200.0, velocity=1_700.0, gamma=0.0, heading=0.0, gravity=9.80665)
        self.assertGreater(gamma_rate, 0.0)
        self.assertAlmostEqual(heading_rate, 0.0)

    def test_bank_projects_lift_into_heading_rate(self):
        force = wind_forces_to_enu(lift=12_000.0, drag=0.0, side_force=0.0, gamma=0.0, heading=0.0, bank=math.radians(20.0))
        _speed_dot, _gamma_rate, heading_rate = force_path_rates(force, mass=1_200.0, velocity=1_700.0, gamma=0.0, heading=0.0, gravity=9.80665)
        self.assertGreater(heading_rate, 0.0)


if __name__ == "__main__":
    unittest.main()
