"""Teaching quaternion and DCM utilities for six-DOF extensions.

The convention here is local ENU. The quaternion is scalar-first and represents
the body-to-ENU attitude. Positive yaw rotates the body x-axis from east toward
north; positive pitch raises the body x-axis upward.
"""

from __future__ import annotations

import math


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


def vector_dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vector_cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(vector_dot(vector, vector))


def normalize_vector(vector: Vector3) -> Vector3:
    norm = vector_norm(vector)
    if norm <= 1e-12:
        raise ValueError("cannot normalize a near-zero vector")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_norm(quaternion: Quaternion) -> float:
    return math.sqrt(sum(value * value for value in quaternion))


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = quaternion_norm(quaternion)
    if norm <= 1e-12:
        raise ValueError("cannot normalize a near-zero quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Return body-to-ENU quaternion from roll, pitch, yaw.

    The pitch sign is chosen so positive pitch gives a positive ENU up
    component for the body x-axis.
    """
    half_roll = 0.5 * roll
    half_pitch = -0.5 * pitch
    half_yaw = 0.5 * yaw
    q_roll = (math.cos(half_roll), math.sin(half_roll), 0.0, 0.0)
    q_pitch = (math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0)
    q_yaw = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))
    return normalize_quaternion(quaternion_multiply(quaternion_multiply(q_yaw, q_pitch), q_roll))


def quaternion_derivative_body_rates(quaternion: Quaternion, body_rates: Vector3) -> Quaternion:
    """Return q_dot for body-frame rates p, q, r."""
    p_rate, q_rate, r_rate = body_rates
    derivative = quaternion_multiply(normalize_quaternion(quaternion), (0.0, p_rate, q_rate, r_rate))
    return tuple(0.5 * value for value in derivative)  # type: ignore[return-value]


def integrate_quaternion_euler(
    quaternion: Quaternion,
    body_rates: Vector3,
    dt: float,
    normalize_result: bool = True,
) -> Quaternion:
    derivative = quaternion_derivative_body_rates(quaternion, body_rates)
    next_quaternion = tuple(quaternion[i] + dt * derivative[i] for i in range(4))  # type: ignore[return-value]
    if normalize_result:
        return normalize_quaternion(next_quaternion)
    return next_quaternion


def quaternion_to_dcm_body_to_enu(quaternion: Quaternion) -> Matrix3:
    """Return a DCM that maps body-frame vectors into local ENU."""
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ),
        (
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ),
        (
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def euler_to_dcm_body_to_enu(roll: float, pitch: float, yaw: float) -> Matrix3:
    return quaternion_to_dcm_body_to_enu(euler_to_quaternion(roll, pitch, yaw))


def dcm_matvec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def wind_forces_to_body(lift: float, drag: float, side_force: float, alpha: float, beta: float) -> Vector3:
    """Convert teaching wind-axis aerodynamic loads into body axes.

    Body axes are x forward, y lateral, z up. Drag opposes the body-frame
    velocity direction. Lift is built from the pitch-plane normal and made
    perpendicular to velocity; side force completes the right-handed local
    wind triad. This is a teaching convention check, not a substitute for a
    validated aerodynamic-axis definition.
    """
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)
    cos_beta = math.cos(beta)
    sin_beta = math.sin(beta)
    velocity_axis = normalize_vector((cos_alpha * cos_beta, sin_beta, -sin_alpha * cos_beta))
    pitch_lift_seed = normalize_vector((sin_alpha, 0.0, cos_alpha))
    side_axis = normalize_vector(vector_cross(pitch_lift_seed, velocity_axis))
    lift_axis = normalize_vector(vector_cross(velocity_axis, side_axis))
    return (
        -drag * velocity_axis[0] + lift * lift_axis[0] + side_force * side_axis[0],
        -drag * velocity_axis[1] + lift * lift_axis[1] + side_force * side_axis[1],
        -drag * velocity_axis[2] + lift * lift_axis[2] + side_force * side_axis[2],
    )


def body_force_to_enu(body_force: Vector3, body_to_enu_dcm: Matrix3) -> Vector3:
    return dcm_matvec(body_to_enu_dcm, body_force)


def wind_forces_to_enu_dcm(
    lift: float,
    drag: float,
    side_force: float,
    alpha: float,
    beta: float,
    body_to_enu_dcm: Matrix3,
) -> tuple[Vector3, Vector3]:
    body_force = wind_forces_to_body(lift, drag, side_force, alpha, beta)
    return body_force, body_force_to_enu(body_force, body_to_enu_dcm)


def _dcm_column_dot(matrix: Matrix3, i: int, j: int) -> float:
    return sum(matrix[row][i] * matrix[row][j] for row in range(3))


def dcm_orthogonality_error(matrix: Matrix3) -> float:
    """Return max absolute entry of R^T R - I."""
    error = 0.0
    for i in range(3):
        for j in range(3):
            expected = 1.0 if i == j else 0.0
            error = max(error, abs(_dcm_column_dot(matrix, i, j) - expected))
    return error
