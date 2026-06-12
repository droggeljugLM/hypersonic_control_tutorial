"""Teaching six-DOF rigid-body propagation model.

This module is intentionally small. It propagates ENU position, ENU velocity,
body-to-ENU quaternion attitude, and body rates under body-frame forces and
moments. It is a teaching plant for coordinate and state propagation, not a
validated high-fidelity hypersonic vehicle model.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import clamp
from hgv_control.models.rigid_body_kinematics import (
    Quaternion,
    Vector3,
    body_force_to_enu,
    dcm_orthogonality_error,
    normalize_quaternion,
    quaternion_derivative_body_rates,
    quaternion_norm,
    quaternion_to_dcm_body_to_enu,
    vector_cross,
    vector_norm,
)


@dataclass(frozen=True)
class RigidBody6DofParams:
    mass: float = 1200.0
    gravity: float = 9.80665
    ix: float = 3.6e4
    iy: float = 5.2e4
    iz: float = 6.1e4
    force_limit: float = 80_000.0
    moment_limit: float = 8_000.0
    min_altitude: float = 0.0


@dataclass
class RigidBody6DofState:
    east: float
    north: float
    altitude: float
    velocity_east: float
    velocity_north: float
    velocity_up: float
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float
    p: float
    q: float
    r: float


@dataclass(frozen=True)
class BodyForceMoment:
    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float


def make_level_initial_state(
    altitude: float = 30_000.0,
    velocity_east: float = 1700.0,
) -> RigidBody6DofState:
    return RigidBody6DofState(
        east=0.0,
        north=0.0,
        altitude=altitude,
        velocity_east=velocity_east,
        velocity_north=0.0,
        velocity_up=0.0,
        quat_w=1.0,
        quat_x=0.0,
        quat_y=0.0,
        quat_z=0.0,
        p=0.0,
        q=0.0,
        r=0.0,
    )


def state_quaternion(state: RigidBody6DofState) -> Quaternion:
    return (state.quat_w, state.quat_x, state.quat_y, state.quat_z)


def state_body_rates(state: RigidBody6DofState) -> Vector3:
    return (state.p, state.q, state.r)


def speed(state: RigidBody6DofState) -> float:
    return math.sqrt(
        state.velocity_east * state.velocity_east
        + state.velocity_north * state.velocity_north
        + state.velocity_up * state.velocity_up
    )


def limit_force_moment(command: BodyForceMoment, params: RigidBody6DofParams) -> BodyForceMoment:
    return BodyForceMoment(
        fx=clamp(command.fx, -params.force_limit, params.force_limit),
        fy=clamp(command.fy, -params.force_limit, params.force_limit),
        fz=clamp(command.fz, -params.force_limit, params.force_limit),
        mx=clamp(command.mx, -params.moment_limit, params.moment_limit),
        my=clamp(command.my, -params.moment_limit, params.moment_limit),
        mz=clamp(command.mz, -params.moment_limit, params.moment_limit),
    )


def body_rate_derivative(
    body_rates: Vector3,
    moments: Vector3,
    params: RigidBody6DofParams,
) -> Vector3:
    p, q, r = body_rates
    j_omega = (params.ix * p, params.iy * q, params.iz * r)
    gyroscopic = vector_cross(body_rates, j_omega)
    return (
        (moments[0] - gyroscopic[0]) / params.ix,
        (moments[1] - gyroscopic[1]) / params.iy,
        (moments[2] - gyroscopic[2]) / params.iz,
    )


def force_acceleration_enu(
    state: RigidBody6DofState,
    command: BodyForceMoment,
    params: RigidBody6DofParams,
) -> tuple[Vector3, Vector3]:
    dcm_body_to_enu = quaternion_to_dcm_body_to_enu(state_quaternion(state))
    force_enu = body_force_to_enu((command.fx, command.fy, command.fz), dcm_body_to_enu)
    acceleration = (
        force_enu[0] / params.mass,
        force_enu[1] / params.mass,
        force_enu[2] / params.mass - params.gravity,
    )
    return force_enu, acceleration


def derivatives(
    state: RigidBody6DofState,
    command: BodyForceMoment,
    params: RigidBody6DofParams,
) -> RigidBody6DofState:
    command = limit_force_moment(command, params)
    _force_enu, acceleration = force_acceleration_enu(state, command, params)
    quat_dot = quaternion_derivative_body_rates(state_quaternion(state), state_body_rates(state))
    rate_dot = body_rate_derivative(state_body_rates(state), (command.mx, command.my, command.mz), params)
    return RigidBody6DofState(
        east=state.velocity_east,
        north=state.velocity_north,
        altitude=state.velocity_up,
        velocity_east=acceleration[0],
        velocity_north=acceleration[1],
        velocity_up=acceleration[2],
        quat_w=quat_dot[0],
        quat_x=quat_dot[1],
        quat_y=quat_dot[2],
        quat_z=quat_dot[3],
        p=rate_dot[0],
        q=rate_dot[1],
        r=rate_dot[2],
    )


def euler_step(
    state: RigidBody6DofState,
    command: BodyForceMoment,
    dt: float,
    params: RigidBody6DofParams,
) -> RigidBody6DofState:
    derivative = derivatives(state, command, params)
    quaternion = normalize_quaternion(
        (
            state.quat_w + dt * derivative.quat_w,
            state.quat_x + dt * derivative.quat_x,
            state.quat_y + dt * derivative.quat_y,
            state.quat_z + dt * derivative.quat_z,
        )
    )
    return RigidBody6DofState(
        east=state.east + dt * derivative.east,
        north=state.north + dt * derivative.north,
        altitude=max(params.min_altitude, state.altitude + dt * derivative.altitude),
        velocity_east=state.velocity_east + dt * derivative.velocity_east,
        velocity_north=state.velocity_north + dt * derivative.velocity_north,
        velocity_up=state.velocity_up + dt * derivative.velocity_up,
        quat_w=quaternion[0],
        quat_x=quaternion[1],
        quat_y=quaternion[2],
        quat_z=quaternion[3],
        p=state.p + dt * derivative.p,
        q=state.q + dt * derivative.q,
        r=state.r + dt * derivative.r,
    )


def kinematics_health(state: RigidBody6DofState) -> tuple[float, float]:
    quaternion = state_quaternion(state)
    dcm_body_to_enu = quaternion_to_dcm_body_to_enu(quaternion)
    return abs(quaternion_norm(quaternion) - 1.0), dcm_orthogonality_error(dcm_body_to_enu)


def state_is_finite(state: RigidBody6DofState) -> bool:
    return all(math.isfinite(value) for value in state.__dict__.values())


def body_rate_norm(state: RigidBody6DofState) -> float:
    return vector_norm(state_body_rates(state))
