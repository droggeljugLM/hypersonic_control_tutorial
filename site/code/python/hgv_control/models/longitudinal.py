"""Simplified longitudinal hypersonic vehicle model for teaching.

The model is intentionally compact. It is suitable for control-education
examples, not for vehicle design or flight qualification.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LongitudinalParams:
    mass: float = 1200.0
    iy: float = 8.5e4
    ref_area: float = 1.2
    ref_chord: float = 3.0
    gravity: float = 9.80665
    rho0: float = 1.225
    scale_height: float = 7200.0
    thrust_max: float = 55_000.0
    cl0: float = 0.04
    cl_alpha: float = 2.2
    cl_delta: float = 0.25
    cd0: float = 0.045
    cd_alpha2: float = 0.85
    cm_alpha: float = -0.42
    cm_q: float = -5.0
    cm_delta: float = -0.75
    actuator_wn: float = 22.0
    actuator_zeta: float = 0.75
    delta_limit: float = math.radians(18.0)
    delta_rate_limit: float = math.radians(80.0)
    throttle_min: float = 0.0
    throttle_max: float = 1.0
    q_limit: float = 70_000.0
    q_warning: float = 62_000.0


@dataclass
class State:
    velocity: float
    altitude: float
    gamma: float
    alpha: float
    pitch_rate: float
    theta: float
    elevator: float
    elevator_rate: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def atmosphere_density(altitude: float, params: LongitudinalParams) -> float:
    altitude = max(0.0, altitude)
    return params.rho0 * math.exp(-altitude / params.scale_height)


def dynamic_pressure(state: State, params: LongitudinalParams) -> float:
    rho = atmosphere_density(state.altitude, params)
    return 0.5 * rho * state.velocity * state.velocity


def aero_forces(state: State, throttle: float, params: LongitudinalParams) -> tuple[float, float, float, float]:
    qbar = dynamic_pressure(state, params)
    alpha = state.alpha
    delta = state.elevator
    velocity = max(100.0, state.velocity)

    cl = params.cl0 + params.cl_alpha * alpha + params.cl_delta * delta
    cd = params.cd0 + params.cd_alpha2 * alpha * alpha
    cm = (
        params.cm_alpha * alpha
        + params.cm_q * (params.ref_chord / (2.0 * velocity)) * state.pitch_rate
        + params.cm_delta * delta
    )

    lift = qbar * params.ref_area * cl
    drag = qbar * params.ref_area * cd
    moment = qbar * params.ref_area * params.ref_chord * cm
    thrust = params.thrust_max * clamp(throttle, params.throttle_min, params.throttle_max)
    return lift, drag, moment, thrust


def derivatives(state: State, delta_cmd: float, throttle_cmd: float, params: LongitudinalParams) -> State:
    delta_cmd = clamp(delta_cmd, -params.delta_limit, params.delta_limit)
    throttle_cmd = clamp(throttle_cmd, params.throttle_min, params.throttle_max)

    lift, drag, moment, thrust = aero_forces(state, throttle_cmd, params)
    velocity = max(100.0, state.velocity)

    delta_accel = (
        params.actuator_wn * params.actuator_wn * (delta_cmd - state.elevator)
        - 2.0 * params.actuator_zeta * params.actuator_wn * state.elevator_rate
    )
    if abs(state.elevator_rate) >= params.delta_rate_limit and state.elevator_rate * delta_accel > 0.0:
        delta_accel = 0.0

    v_dot = (thrust * math.cos(state.alpha) - drag) / params.mass - params.gravity * math.sin(state.gamma)
    gamma_dot = (
        (thrust * math.sin(state.alpha) + lift) / (params.mass * velocity)
        - params.gravity * math.cos(state.gamma) / velocity
    )
    h_dot = state.velocity * math.sin(state.gamma)
    q_dot = moment / params.iy
    theta_dot = state.pitch_rate
    alpha_dot = theta_dot - gamma_dot

    return State(
        velocity=v_dot,
        altitude=h_dot,
        gamma=gamma_dot,
        alpha=alpha_dot,
        pitch_rate=q_dot,
        theta=theta_dot,
        elevator=state.elevator_rate,
        elevator_rate=delta_accel,
    )


def add_state(a: State, b: State, scale: float) -> State:
    return State(
        velocity=a.velocity + scale * b.velocity,
        altitude=a.altitude + scale * b.altitude,
        gamma=a.gamma + scale * b.gamma,
        alpha=a.alpha + scale * b.alpha,
        pitch_rate=a.pitch_rate + scale * b.pitch_rate,
        theta=a.theta + scale * b.theta,
        elevator=clamp(a.elevator + scale * b.elevator, -math.radians(25.0), math.radians(25.0)),
        elevator_rate=clamp(a.elevator_rate + scale * b.elevator_rate, -math.radians(120.0), math.radians(120.0)),
    )


def rk4_step(state: State, delta_cmd: float, throttle_cmd: float, dt: float, params: LongitudinalParams) -> State:
    k1 = derivatives(state, delta_cmd, throttle_cmd, params)
    k2 = derivatives(add_state(state, k1, 0.5 * dt), delta_cmd, throttle_cmd, params)
    k3 = derivatives(add_state(state, k2, 0.5 * dt), delta_cmd, throttle_cmd, params)
    k4 = derivatives(add_state(state, k3, dt), delta_cmd, throttle_cmd, params)

    next_state = State(
        velocity=state.velocity + dt * (k1.velocity + 2 * k2.velocity + 2 * k3.velocity + k4.velocity) / 6.0,
        altitude=state.altitude + dt * (k1.altitude + 2 * k2.altitude + 2 * k3.altitude + k4.altitude) / 6.0,
        gamma=state.gamma + dt * (k1.gamma + 2 * k2.gamma + 2 * k3.gamma + k4.gamma) / 6.0,
        alpha=state.alpha + dt * (k1.alpha + 2 * k2.alpha + 2 * k3.alpha + k4.alpha) / 6.0,
        pitch_rate=state.pitch_rate + dt * (k1.pitch_rate + 2 * k2.pitch_rate + 2 * k3.pitch_rate + k4.pitch_rate) / 6.0,
        theta=state.theta + dt * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta) / 6.0,
        elevator=state.elevator + dt * (k1.elevator + 2 * k2.elevator + 2 * k3.elevator + k4.elevator) / 6.0,
        elevator_rate=state.elevator_rate
        + dt * (k1.elevator_rate + 2 * k2.elevator_rate + 2 * k3.elevator_rate + k4.elevator_rate) / 6.0,
    )
    next_state.velocity = max(100.0, next_state.velocity)
    next_state.altitude = max(0.0, next_state.altitude)
    next_state.elevator = clamp(next_state.elevator, -params.delta_limit, params.delta_limit)
    next_state.elevator_rate = clamp(next_state.elevator_rate, -params.delta_rate_limit, params.delta_rate_limit)
    next_state.alpha = clamp(next_state.alpha, math.radians(-12.0), math.radians(18.0))
    return next_state
