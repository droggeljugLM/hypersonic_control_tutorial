import math
import unittest

from hgv_control.models.rigid_body_kinematics import (
    dcm_matvec,
    dcm_orthogonality_error,
    euler_to_dcm_body_to_enu,
    euler_to_quaternion,
    wind_forces_to_body,
    wind_forces_to_enu_dcm,
    integrate_quaternion_euler,
    normalize_quaternion,
    quaternion_norm,
    quaternion_to_dcm_body_to_enu,
)


class RigidBodyKinematicsTests(unittest.TestCase):
    def test_identity_quaternion_gives_identity_dcm(self):
        dcm = quaternion_to_dcm_body_to_enu((1.0, 0.0, 0.0, 0.0))
        self.assertEqual(dcm[0], (1.0, 0.0, 0.0))
        self.assertEqual(dcm[1], (0.0, 1.0, 0.0))
        self.assertEqual(dcm[2], (0.0, 0.0, 1.0))

    def test_normalize_quaternion_preserves_unit_norm(self):
        quaternion = normalize_quaternion((2.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(quaternion_norm(quaternion), 1.0)

    def test_euler_dcm_has_expected_yaw_and_pitch_directions(self):
        yaw_dcm = euler_to_dcm_body_to_enu(0.0, 0.0, math.radians(10.0))
        yaw_body_x = dcm_matvec(yaw_dcm, (1.0, 0.0, 0.0))
        self.assertGreater(yaw_body_x[1], 0.0)

        pitch_dcm = euler_to_dcm_body_to_enu(0.0, math.radians(8.0), 0.0)
        pitch_body_x = dcm_matvec(pitch_dcm, (1.0, 0.0, 0.0))
        self.assertGreater(pitch_body_x[2], 0.0)

    def test_dcm_from_quaternion_is_orthonormal(self):
        quaternion = euler_to_quaternion(math.radians(4.0), math.radians(7.0), math.radians(11.0))
        dcm = quaternion_to_dcm_body_to_enu(quaternion)
        self.assertLess(dcm_orthogonality_error(dcm), 1e-12)

    def test_integrated_positive_yaw_rate_turns_body_x_toward_north(self):
        quaternion = integrate_quaternion_euler((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.2), 0.1)
        dcm = quaternion_to_dcm_body_to_enu(quaternion)
        body_x = dcm_matvec(dcm, (1.0, 0.0, 0.0))
        self.assertGreater(body_x[1], 0.0)
        self.assertAlmostEqual(quaternion_norm(quaternion), 1.0)

    def test_zero_angle_wind_loads_map_to_teaching_body_axes(self):
        body_force = wind_forces_to_body(lift=1200.0, drag=300.0, side_force=50.0, alpha=0.0, beta=0.0)
        self.assertAlmostEqual(body_force[0], -300.0)
        self.assertAlmostEqual(body_force[1], 50.0)
        self.assertAlmostEqual(body_force[2], 1200.0)

    def test_dcm_force_transform_preserves_force_norm(self):
        dcm = euler_to_dcm_body_to_enu(math.radians(4.0), math.radians(6.0), math.radians(12.0))
        body_force, enu_force = wind_forces_to_enu_dcm(
            lift=1500.0,
            drag=400.0,
            side_force=80.0,
            alpha=math.radians(3.0),
            beta=math.radians(2.0),
            body_to_enu_dcm=dcm,
        )
        body_norm = math.sqrt(sum(value * value for value in body_force))
        enu_norm = math.sqrt(sum(value * value for value in enu_force))
        self.assertAlmostEqual(body_norm, enu_norm)


if __name__ == "__main__":
    unittest.main()
