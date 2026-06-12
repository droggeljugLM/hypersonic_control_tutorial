"""Teaching 3-D point-mass guidance model.

This is the first executable step toward the six-DOF/IGC chapter. It keeps
only position, speed, flight-path angle, and heading. It is not a six-DOF
rigid-body model and has no attitude or control-allocation dynamics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import clamp


@dataclass(frozen=True)
class PointMass3DParams:
    mass: float = 1200.0
    ref_area: float = 1.2
    gravity: float = 9.80665
    rho0: float = 1.225
    scale_height: float = 7200.0
    thrust_max: float = 8000.0
    cd0: float = 0.075
    throttle_min: float = 0.0
    throttle_max: float = 1.0
    gamma_rate_limit: float = math.radians(0.8)
    heading_rate_limit: float = math.radians(1.2)
    q_limit: float = 70_000.0
    load_factor_limit: float = 4.0


@dataclass
class PointMass3DState:
    east: float
    north: float
    altitude: float
    velocity: float
    gamma: float
    heading: float


@dataclass(frozen=True)
class PointMass3DCommand:
    gamma_rate: float
    heading_rate: float
    throttle: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def atmosphere_density_3d(altitude: float, params: PointMass3DParams) -> float:
    altitude = max(0.0, altitude)
    return params.rho0 * math.exp(-altitude / params.scale_height)


def dynamic_pressure_3d(state: PointMass3DState, params: PointMass3DParams) -> float:
    rho = atmosphere_density_3d(state.altitude, params)
    return 0.5 * rho * state.velocity * state.velocity


def load_factor_3d(
    state: PointMass3DState,
    command: PointMass3DCommand,
    params: PointMass3DParams,
) -> float:
    gamma_rate = clamp(command.gamma_rate, -params.gamma_rate_limit, params.gamma_rate_limit)
    heading_rate = clamp(command.heading_rate, -params.heading_rate_limit, params.heading_rate_limit)
    normal_component = math.cos(state.gamma) + state.velocity * gamma_rate / params.gravity
    lateral_component = state.velocity * math.cos(state.gamma) * heading_rate / params.gravity
    return math.sqrt(normal_component * normal_component + lateral_component * lateral_component)


def derivatives(
    state: PointMass3DState,
    command: PointMass3DCommand,
    params: PointMass3DParams,
) -> PointMass3DState:
    velocity = max(100.0, state.velocity)
    gamma_rate = clamp(command.gamma_rate, -params.gamma_rate_limit, params.gamma_rate_limit)
    heading_rate = clamp(command.heading_rate, -params.heading_rate_limit, params.heading_rate_limit)
    throttle = clamp(command.throttle, params.throttle_min, params.throttle_max)
    qbar = dynamic_pressure_3d(state, params)
    drag = qbar * params.ref_area * params.cd0
    thrust = params.thrust_max * throttle

    return PointMass3DState(
        east=velocity * math.cos(state.gamma) * math.cos(state.heading),
        north=velocity * math.cos(state.gamma) * math.sin(state.heading),
        altitude=velocity * math.sin(state.gamma),
        velocity=(thrust - drag) / params.mass - params.gravity * math.sin(state.gamma),
        gamma=gamma_rate,
        heading=heading_rate,
    )


def add_state(a: PointMass3DState, b: PointMass3DState, scale: float) -> PointMass3DState:
    return PointMass3DState(
        east=a.east + scale * b.east,
        north=a.north + scale * b.north,
        altitude=max(0.0, a.altitude + scale * b.altitude),
        velocity=max(100.0, a.velocity + scale * b.velocity),
        gamma=a.gamma + scale * b.gamma,
        heading=wrap_angle(a.heading + scale * b.heading),
    )


def rk4_step(
    state: PointMass3DState,
    command: PointMass3DCommand,
    dt: float,
    params: PointMass3DParams,
) -> PointMass3DState:
    k1 = derivatives(state, command, params)
    k2 = derivatives(add_state(state, k1, 0.5 * dt), command, params)
    k3 = derivatives(add_state(state, k2, 0.5 * dt), command, params)
    k4 = derivatives(add_state(state, k3, dt), command, params)
    next_state = PointMass3DState(
        east=state.east + dt * (k1.east + 2 * k2.east + 2 * k3.east + k4.east) / 6.0,
        north=state.north + dt * (k1.north + 2 * k2.north + 2 * k3.north + k4.north) / 6.0,
        altitude=state.altitude + dt * (k1.altitude + 2 * k2.altitude + 2 * k3.altitude + k4.altitude) / 6.0,
        velocity=state.velocity + dt * (k1.velocity + 2 * k2.velocity + 2 * k3.velocity + k4.velocity) / 6.0,
        gamma=state.gamma + dt * (k1.gamma + 2 * k2.gamma + 2 * k3.gamma + k4.gamma) / 6.0,
        heading=state.heading + dt * (k1.heading + 2 * k2.heading + 2 * k3.heading + k4.heading) / 6.0,
    )
    next_state.altitude = max(0.0, next_state.altitude)
    next_state.velocity = max(100.0, next_state.velocity)
    next_state.gamma = clamp(next_state.gamma, math.radians(-20.0), math.radians(20.0))
    next_state.heading = wrap_angle(next_state.heading)
    return next_state
