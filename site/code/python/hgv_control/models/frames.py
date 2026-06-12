"""Teaching coordinate-frame utilities for 3-D flight examples.

The project uses a local ENU convention: east, north, up. Heading is measured
from east toward north, and flight-path angle is positive upward.
"""

from __future__ import annotations

import math


Vector3 = tuple[float, float, float]


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(value: float, vector: Vector3) -> Vector3:
    return (value * vector[0], value * vector[1], value * vector[2])


def norm(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def flight_path_basis(gamma: float, heading: float) -> tuple[Vector3, Vector3, Vector3]:
    """Return velocity, vertical-normal, and heading-side unit vectors in ENU."""
    cos_gamma = math.cos(gamma)
    sin_gamma = math.sin(gamma)
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)
    velocity_axis = (cos_gamma * cos_heading, cos_gamma * sin_heading, sin_gamma)
    normal_axis = (-sin_gamma * cos_heading, -sin_gamma * sin_heading, cos_gamma)
    heading_axis = (-sin_heading, cos_heading, 0.0)
    return velocity_axis, normal_axis, heading_axis


def wind_forces_to_enu(
    lift: float,
    drag: float,
    side_force: float,
    gamma: float,
    heading: float,
    bank: float,
) -> Vector3:
    """Convert teaching wind-axis forces to ENU.

    Drag acts opposite velocity. Lift is rotated by bank from the vertical
    flight-path normal toward the heading-side axis. Side force is added along
    the heading-side axis.
    """
    velocity_axis, normal_axis, heading_axis = flight_path_basis(gamma, heading)
    force = scale(-drag, velocity_axis)
    force = add(force, scale(lift * math.cos(bank), normal_axis))
    force = add(force, scale(lift * math.sin(bank) + side_force, heading_axis))
    return force


def force_path_rates(
    force_enu: Vector3,
    mass: float,
    velocity: float,
    gamma: float,
    heading: float,
    gravity: float,
) -> tuple[float, float, float]:
    """Project inertial acceleration into speed, gamma-rate, and heading-rate."""
    velocity_axis, normal_axis, heading_axis = flight_path_basis(gamma, heading)
    gravity_enu = (0.0, 0.0, -gravity)
    acceleration = add(scale(1.0 / max(mass, 1e-9), force_enu), gravity_enu)
    speed_dot = dot(acceleration, velocity_axis)
    gamma_rate = dot(acceleration, normal_axis) / max(velocity, 100.0)
    heading_rate = dot(acceleration, heading_axis) / max(velocity * max(math.cos(gamma), 0.18), 100.0)
    return speed_dot, gamma_rate, heading_rate
