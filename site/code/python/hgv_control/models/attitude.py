"""Teaching three-axis attitude inner-loop model.

The model keeps Euler angles, body rates, and rate-limited body moments. It is
an attitude-control teaching plant, not a complete six-DOF flight model.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import clamp


@dataclass(frozen=True)
class AttitudeParams:
    ix: float = 3.6e4
    iy: float = 5.2e4
    iz: float = 6.1e4
    damping_p: float = 1.4e3
    damping_q: float = 1.7e3
    damping_r: float = 1.9e3
    moment_limit: float = 4_800.0
    moment_rate_limit: float = 18_000.0
    actuator_time_constant: float = 0.08
    max_pitch_for_kinematics: float = math.radians(80.0)


@dataclass
class AttitudeState:
    phi: float
    theta: float
    psi: float
    p: float
    q: float
    r: float
    mx: float
    my: float
    mz: float


@dataclass(frozen=True)
class MomentCommand:
    mx: float
    my: float
    mz: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def attitude_error(state: AttitudeState, phi_ref: float, theta_ref: float, psi_ref: float) -> tuple[float, float, float]:
    return (
        wrap_angle(phi_ref - state.phi),
        wrap_angle(theta_ref - state.theta),
        wrap_angle(psi_ref - state.psi),
    )


def limit_moment_command(command: MomentCommand, params: AttitudeParams) -> MomentCommand:
    return MomentCommand(
        mx=clamp(command.mx, -params.moment_limit, params.moment_limit),
        my=clamp(command.my, -params.moment_limit, params.moment_limit),
        mz=clamp(command.mz, -params.moment_limit, params.moment_limit),
    )


def _moment_rate(actual: float, target: float, params: AttitudeParams) -> float:
    desired_rate = (target - actual) / max(params.actuator_time_constant, 1e-6)
    return clamp(desired_rate, -params.moment_rate_limit, params.moment_rate_limit)


def derivatives(state: AttitudeState, command: MomentCommand, params: AttitudeParams) -> AttitudeState:
    command = limit_moment_command(command, params)
    theta = clamp(state.theta, -params.max_pitch_for_kinematics, params.max_pitch_for_kinematics)
    cos_theta = max(0.18, abs(math.cos(theta)))
    tan_theta = math.sin(theta) / cos_theta

    phi_dot = state.p + tan_theta * (state.q * math.sin(state.phi) + state.r * math.cos(state.phi))
    theta_dot = state.q * math.cos(state.phi) - state.r * math.sin(state.phi)
    psi_dot = (state.q * math.sin(state.phi) + state.r * math.cos(state.phi)) / cos_theta

    mx_rate = _moment_rate(state.mx, command.mx, params)
    my_rate = _moment_rate(state.my, command.my, params)
    mz_rate = _moment_rate(state.mz, command.mz, params)

    p_dot = (
        state.mx
        - params.damping_p * state.p
        + (params.iy - params.iz) * state.q * state.r
    ) / params.ix
    q_dot = (
        state.my
        - params.damping_q * state.q
        + (params.iz - params.ix) * state.p * state.r
    ) / params.iy
    r_dot = (
        state.mz
        - params.damping_r * state.r
        + (params.ix - params.iy) * state.p * state.q
    ) / params.iz

    return AttitudeState(
        phi=phi_dot,
        theta=theta_dot,
        psi=psi_dot,
        p=p_dot,
        q=q_dot,
        r=r_dot,
        mx=mx_rate,
        my=my_rate,
        mz=mz_rate,
    )


def add_state(a: AttitudeState, b: AttitudeState, scale: float, params: AttitudeParams) -> AttitudeState:
    return AttitudeState(
        phi=wrap_angle(a.phi + scale * b.phi),
        theta=clamp(a.theta + scale * b.theta, -params.max_pitch_for_kinematics, params.max_pitch_for_kinematics),
        psi=wrap_angle(a.psi + scale * b.psi),
        p=a.p + scale * b.p,
        q=a.q + scale * b.q,
        r=a.r + scale * b.r,
        mx=clamp(a.mx + scale * b.mx, -params.moment_limit, params.moment_limit),
        my=clamp(a.my + scale * b.my, -params.moment_limit, params.moment_limit),
        mz=clamp(a.mz + scale * b.mz, -params.moment_limit, params.moment_limit),
    )


def rk4_step(state: AttitudeState, command: MomentCommand, dt: float, params: AttitudeParams) -> AttitudeState:
    k1 = derivatives(state, command, params)
    k2 = derivatives(add_state(state, k1, 0.5 * dt, params), command, params)
    k3 = derivatives(add_state(state, k2, 0.5 * dt, params), command, params)
    k4 = derivatives(add_state(state, k3, dt, params), command, params)
    next_state = AttitudeState(
        phi=state.phi + dt * (k1.phi + 2 * k2.phi + 2 * k3.phi + k4.phi) / 6.0,
        theta=state.theta + dt * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta) / 6.0,
        psi=state.psi + dt * (k1.psi + 2 * k2.psi + 2 * k3.psi + k4.psi) / 6.0,
        p=state.p + dt * (k1.p + 2 * k2.p + 2 * k3.p + k4.p) / 6.0,
        q=state.q + dt * (k1.q + 2 * k2.q + 2 * k3.q + k4.q) / 6.0,
        r=state.r + dt * (k1.r + 2 * k2.r + 2 * k3.r + k4.r) / 6.0,
        mx=state.mx + dt * (k1.mx + 2 * k2.mx + 2 * k3.mx + k4.mx) / 6.0,
        my=state.my + dt * (k1.my + 2 * k2.my + 2 * k3.my + k4.my) / 6.0,
        mz=state.mz + dt * (k1.mz + 2 * k2.mz + 2 * k3.mz + k4.mz) / 6.0,
    )
    next_state.phi = wrap_angle(next_state.phi)
    next_state.theta = clamp(next_state.theta, -params.max_pitch_for_kinematics, params.max_pitch_for_kinematics)
    next_state.psi = wrap_angle(next_state.psi)
    next_state.mx = clamp(next_state.mx, -params.moment_limit, params.moment_limit)
    next_state.my = clamp(next_state.my, -params.moment_limit, params.moment_limit)
    next_state.mz = clamp(next_state.mz, -params.moment_limit, params.moment_limit)
    return next_state
