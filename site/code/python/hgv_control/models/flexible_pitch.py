"""Low-order flexible pitch-axis teaching model.

The model augments a rigid pitch axis with one elastic mode, measured-rate
contamination, actuator rate limiting, and a modal effectiveness factor. It is
for control-interface training, not structural certification.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import clamp


@dataclass(frozen=True)
class FlexiblePitchParams:
    inertia_y: float = 5.2e4
    rigid_damping_q: float = 1.7e3
    actuator_tau: float = 0.08
    moment_limit: float = 4_800.0
    moment_rate_limit: float = 18_000.0
    flexible_frequency_rad_s: float = 8.0
    flexible_damping_ratio: float = 0.035
    moment_to_modal: float = 2.0e-4
    rate_to_modal: float = 0.12
    modal_to_moment: float = -800.0
    sensor_modal_rate_gain: float = 0.45
    effectiveness_modal_gain: float = 4.0
    min_effectiveness: float = 0.70


@dataclass
class FlexiblePitchState:
    theta: float
    q: float
    moment: float
    eta: float
    eta_dot: float


def make_initial_state() -> FlexiblePitchState:
    return FlexiblePitchState(theta=math.radians(-1.0), q=0.0, moment=0.0, eta=0.0, eta_dot=0.0)


def measured_pitch_rate(state: FlexiblePitchState, params: FlexiblePitchParams) -> float:
    return state.q + params.sensor_modal_rate_gain * state.eta_dot


def input_effectiveness(state: FlexiblePitchState, params: FlexiblePitchParams) -> float:
    value = 1.0 - params.effectiveness_modal_gain * abs(state.eta)
    return clamp(value, params.min_effectiveness, 1.0)


def limit_moment_command(command: float, params: FlexiblePitchParams) -> float:
    return clamp(command, -params.moment_limit, params.moment_limit)


def _moment_rate(actual: float, target: float, params: FlexiblePitchParams) -> float:
    desired_rate = (target - actual) / max(params.actuator_tau, 1e-6)
    return clamp(desired_rate, -params.moment_rate_limit, params.moment_rate_limit)


def modal_energy(state: FlexiblePitchState, params: FlexiblePitchParams) -> float:
    omega = params.flexible_frequency_rad_s
    return 0.5 * (state.eta_dot * state.eta_dot + omega * omega * state.eta * state.eta)


def derivatives(
    state: FlexiblePitchState,
    moment_command: float,
    params: FlexiblePitchParams,
    disturbance_moment: float = 0.0,
) -> FlexiblePitchState:
    command = limit_moment_command(moment_command, params)
    moment_rate = _moment_rate(state.moment, command, params)
    effectiveness = input_effectiveness(state, params)
    q_dot = (
        effectiveness * state.moment
        + params.modal_to_moment * state.eta
        + disturbance_moment
        - params.rigid_damping_q * state.q
    ) / params.inertia_y
    eta_ddot = (
        params.moment_to_modal * state.moment
        + params.rate_to_modal * state.q
        - 2.0 * params.flexible_damping_ratio * params.flexible_frequency_rad_s * state.eta_dot
        - params.flexible_frequency_rad_s * params.flexible_frequency_rad_s * state.eta
    )
    return FlexiblePitchState(theta=state.q, q=q_dot, moment=moment_rate, eta=state.eta_dot, eta_dot=eta_ddot)


def add_state(a: FlexiblePitchState, b: FlexiblePitchState, scale: float, params: FlexiblePitchParams) -> FlexiblePitchState:
    return FlexiblePitchState(
        theta=a.theta + scale * b.theta,
        q=a.q + scale * b.q,
        moment=clamp(a.moment + scale * b.moment, -params.moment_limit, params.moment_limit),
        eta=a.eta + scale * b.eta,
        eta_dot=a.eta_dot + scale * b.eta_dot,
    )


def rk4_step(
    state: FlexiblePitchState,
    moment_command: float,
    dt: float,
    params: FlexiblePitchParams,
    disturbance_moment: float = 0.0,
) -> FlexiblePitchState:
    k1 = derivatives(state, moment_command, params, disturbance_moment)
    k2 = derivatives(add_state(state, k1, 0.5 * dt, params), moment_command, params, disturbance_moment)
    k3 = derivatives(add_state(state, k2, 0.5 * dt, params), moment_command, params, disturbance_moment)
    k4 = derivatives(add_state(state, k3, dt, params), moment_command, params, disturbance_moment)
    next_state = FlexiblePitchState(
        theta=state.theta + dt * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta) / 6.0,
        q=state.q + dt * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q) / 6.0,
        moment=state.moment + dt * (k1.moment + 2.0 * k2.moment + 2.0 * k3.moment + k4.moment) / 6.0,
        eta=state.eta + dt * (k1.eta + 2.0 * k2.eta + 2.0 * k3.eta + k4.eta) / 6.0,
        eta_dot=state.eta_dot + dt * (k1.eta_dot + 2.0 * k2.eta_dot + 2.0 * k3.eta_dot + k4.eta_dot) / 6.0,
    )
    next_state.moment = clamp(next_state.moment, -params.moment_limit, params.moment_limit)
    return next_state
