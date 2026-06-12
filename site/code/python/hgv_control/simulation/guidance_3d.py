"""Run a teaching 3-D point-mass guidance scenario."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import violation_integral
from hgv_control.models.longitudinal import clamp
from hgv_control.models.point_mass_3d import (
    PointMass3DCommand,
    PointMass3DParams,
    PointMass3DState,
    dynamic_pressure_3d,
    load_factor_3d,
    rk4_step,
    wrap_angle,
)


@dataclass(frozen=True)
class GuidanceTarget3D:
    east: float = 52_000.0
    north: float = 14_000.0
    altitude: float = 29_000.0
    velocity: float = 1_680.0


def make_initial_state() -> PointMass3DState:
    return PointMass3DState(
        east=0.0,
        north=0.0,
        altitude=30_000.0,
        velocity=1_750.0,
        gamma=math.radians(-1.0),
        heading=math.radians(0.0),
    )


def range_to_target(state: PointMass3DState, target: GuidanceTarget3D) -> tuple[float, float, float]:
    east_error = target.east - state.east
    north_error = target.north - state.north
    altitude_error = target.altitude - state.altitude
    horizontal_range = math.hypot(east_error, north_error)
    total_range = math.sqrt(horizontal_range * horizontal_range + altitude_error * altitude_error)
    return horizontal_range, total_range, altitude_error


def guidance_command(
    state: PointMass3DState,
    target: GuidanceTarget3D,
    params: PointMass3DParams,
) -> PointMass3DCommand:
    east_error = target.east - state.east
    north_error = target.north - state.north
    horizontal_range, _total_range, altitude_error = range_to_target(state, target)
    desired_heading = math.atan2(north_error, east_error)
    desired_gamma = math.atan2(altitude_error, max(horizontal_range, 1.0))
    desired_gamma = clamp(desired_gamma, math.radians(-8.0), math.radians(5.0))

    heading_rate = clamp(0.055 * wrap_angle(desired_heading - state.heading), -params.heading_rate_limit, params.heading_rate_limit)
    gamma_rate = clamp(0.075 * (desired_gamma - state.gamma), -params.gamma_rate_limit, params.gamma_rate_limit)
    throttle = clamp(0.38 + 0.0025 * (target.velocity - state.velocity), params.throttle_min, params.throttle_max)
    return PointMass3DCommand(gamma_rate=gamma_rate, heading_rate=heading_rate, throttle=throttle)


def summarize_3d(
    trace: list[dict[str, float]],
    target: GuidanceTarget3D,
    params: PointMass3DParams,
    dt: float,
) -> dict[str, float | bool]:
    last = trace[-1]
    horizontal_miss = math.hypot(last["east"] - target.east, last["north"] - target.north)
    altitude_error = last["altitude"] - target.altitude
    terminal_range = math.sqrt(horizontal_miss * horizontal_miss + altitude_error * altitude_error)
    speed_error = last["velocity"] - target.velocity
    q_values = [row["qbar"] for row in trace]
    load_values = [row["load_factor"] for row in trace]
    q_violation = violation_integral(q_values, params.q_limit, dt)
    load_violation = violation_integral(load_values, params.load_factor_limit, dt)
    terminal_pass = terminal_range <= 8_000.0 and abs(speed_error) <= 180.0
    path_pass = q_violation <= 1.0 and load_violation <= 0.1
    return {
        "terminal_range_m": terminal_range,
        "horizontal_miss_m": horizontal_miss,
        "terminal_altitude_error_m": altitude_error,
        "terminal_speed_error_m_s": speed_error,
        "q_max": max(q_values) if q_values else 0.0,
        "q_margin_min": min((params.q_limit - value for value in q_values), default=0.0),
        "q_violation_integral": q_violation,
        "load_factor_max": max(load_values) if load_values else 0.0,
        "load_factor_violation_integral": load_violation,
        "terminal_pass": terminal_pass,
        "path_pass": path_pass,
        "pass": terminal_pass and path_pass,
    }


def run(
    duration: float = 34.0,
    dt: float = 0.05,
    params: PointMass3DParams | None = None,
    target: GuidanceTarget3D | None = None,
    initial_state: PointMass3DState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or PointMass3DParams()
    target = target or GuidanceTarget3D()
    state = initial_state or make_initial_state()
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        command = guidance_command(state, target, params)
        horizontal_range, total_range, altitude_error = range_to_target(state, target)
        qbar = dynamic_pressure_3d(state, params)
        load_factor = load_factor_3d(state, command, params)
        trace.append(
            {
                "time": time_s,
                "east": state.east,
                "north": state.north,
                "altitude": state.altitude,
                "velocity": state.velocity,
                "gamma": state.gamma,
                "heading": state.heading,
                "target_east": target.east,
                "target_north": target.north,
                "target_altitude": target.altitude,
                "horizontal_range": horizontal_range,
                "range_to_target": total_range,
                "altitude_error": altitude_error,
                "gamma_rate_cmd": command.gamma_rate,
                "heading_rate_cmd": command.heading_rate,
                "throttle_cmd": command.throttle,
                "qbar": qbar,
                "load_factor": load_factor,
            }
        )
        state = rk4_step(state, command, dt, params)
    return trace, summarize_3d(trace, target, params, dt)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=34.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    trace, metrics = run(duration=args.duration, dt=args.dt)
    if args.output is not None:
        write_csv(trace, args.output)
    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
