"""Teaching reentry heat-rate constrained control case.

This module supports case C14 in Volume 5. It uses a reduced point-mass
reentry model with velocity, altitude, downrange, and flight-path angle. The
guarded controller modifies the descent command when heat-rate, dynamic
pressure, load factor, or corridor margins become small.

The implementation is intentionally lightweight. It is a control-education
case for evidence fields, not a high-fidelity aerothermal or bank-angle
guidance model.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import violation_integral
from hgv_control.models.longitudinal import clamp


@dataclass(frozen=True)
class ReentryParams:
    dt: float = 0.1
    duration: float = 72.0
    mass: float = 1_150.0
    ref_area: float = 1.05
    gravity: float = 9.80665
    rho0: float = 1.225
    scale_height: float = 7_200.0
    cd0: float = 0.11
    gamma_rate_limit: float = math.radians(0.32)
    heat_rate_limit: float = 3.0e5
    heat_load_limit: float = 2.0e7
    q_limit: float = 68_000.0
    load_factor_limit: float = 3.8
    downrange_target: float = 185_000.0
    terminal_altitude_target: float = 33_000.0
    terminal_velocity_target: float = 2_120.0


@dataclass(frozen=True)
class ReentryState:
    downrange: float
    altitude: float
    velocity: float
    gamma: float


def make_initial_state() -> ReentryState:
    return ReentryState(
        downrange=0.0,
        altitude=54_000.0,
        velocity=3_150.0,
        gamma=math.radians(-5.6),
    )


def atmosphere_density(altitude: float, params: ReentryParams) -> float:
    return params.rho0 * math.exp(-max(0.0, altitude) / params.scale_height)


def dynamic_pressure(state: ReentryState, params: ReentryParams) -> float:
    return 0.5 * atmosphere_density(state.altitude, params) * state.velocity * state.velocity


def heat_rate(state: ReentryState, params: ReentryParams) -> float:
    # Sutton-Graves-like teaching proxy. The coefficient is tuned only to put
    # the default scenario near the heat-rate boundary.
    return 1.02e-4 * math.sqrt(atmosphere_density(state.altitude, params)) * state.velocity**3


def load_factor(state: ReentryState, gamma_rate_cmd: float, params: ReentryParams) -> float:
    gamma_rate = clamp(gamma_rate_cmd, -params.gamma_rate_limit, params.gamma_rate_limit)
    normal_component = math.cos(state.gamma) + state.velocity * gamma_rate / params.gravity
    return abs(normal_component)


def corridor_bounds(velocity: float) -> tuple[float, float]:
    speed_excess = max(0.0, velocity - 1_700.0)
    lower = 23_000.0 + 0.0068 * speed_excess * speed_excess
    upper = 61_000.0 - 0.0022 * max(0.0, 3_300.0 - velocity) ** 2
    return lower, max(lower + 4_000.0, upper)


def corridor_margin(state: ReentryState) -> float:
    lower, upper = corridor_bounds(state.velocity)
    return min(state.altitude - lower, upper - state.altitude)


def derivatives(state: ReentryState, gamma_rate_cmd: float, params: ReentryParams) -> ReentryState:
    gamma_rate = clamp(gamma_rate_cmd, -params.gamma_rate_limit, params.gamma_rate_limit)
    velocity = max(300.0, state.velocity)
    qbar = dynamic_pressure(state, params)
    drag = qbar * params.ref_area * params.cd0
    v_dot = -drag / params.mass - params.gravity * math.sin(state.gamma)
    return ReentryState(
        downrange=velocity * math.cos(state.gamma),
        altitude=velocity * math.sin(state.gamma),
        velocity=v_dot,
        gamma=gamma_rate,
    )


def add_state(state: ReentryState, slope: ReentryState, scale: float) -> ReentryState:
    return ReentryState(
        downrange=state.downrange + scale * slope.downrange,
        altitude=max(0.0, state.altitude + scale * slope.altitude),
        velocity=max(300.0, state.velocity + scale * slope.velocity),
        gamma=clamp(state.gamma + scale * slope.gamma, math.radians(-12.0), math.radians(2.0)),
    )


def rk4_step(state: ReentryState, gamma_rate_cmd: float, params: ReentryParams) -> ReentryState:
    dt = params.dt
    k1 = derivatives(state, gamma_rate_cmd, params)
    k2 = derivatives(add_state(state, k1, 0.5 * dt), gamma_rate_cmd, params)
    k3 = derivatives(add_state(state, k2, 0.5 * dt), gamma_rate_cmd, params)
    k4 = derivatives(add_state(state, k3, dt), gamma_rate_cmd, params)
    next_state = ReentryState(
        downrange=state.downrange + dt * (k1.downrange + 2.0 * k2.downrange + 2.0 * k3.downrange + k4.downrange) / 6.0,
        altitude=state.altitude + dt * (k1.altitude + 2.0 * k2.altitude + 2.0 * k3.altitude + k4.altitude) / 6.0,
        velocity=state.velocity + dt * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) / 6.0,
        gamma=state.gamma + dt * (k1.gamma + 2.0 * k2.gamma + 2.0 * k3.gamma + k4.gamma) / 6.0,
    )
    return ReentryState(
        downrange=next_state.downrange,
        altitude=max(0.0, next_state.altitude),
        velocity=max(300.0, next_state.velocity),
        gamma=clamp(next_state.gamma, math.radians(-12.0), math.radians(2.0)),
    )


def nominal_gamma_reference(time_s: float) -> float:
    if time_s < 25.0:
        return math.radians(-6.2)
    if time_s < 50.0:
        return math.radians(-5.2)
    return math.radians(-3.8)


def command_baseline(state: ReentryState, time_s: float, params: ReentryParams) -> float:
    gamma_ref = nominal_gamma_reference(time_s)
    return clamp(0.055 * (gamma_ref - state.gamma), -params.gamma_rate_limit, params.gamma_rate_limit)


def command_heat_guard(state: ReentryState, time_s: float, params: ReentryParams) -> float:
    command = command_baseline(state, time_s, params)
    heat_ratio = heat_rate(state, params) / params.heat_rate_limit
    q_ratio = dynamic_pressure(state, params) / params.q_limit
    load_ratio = load_factor(state, command, params) / params.load_factor_limit
    margin = corridor_margin(state)

    relief = 0.0
    relief += max(0.0, heat_ratio - 0.82) * math.radians(1.20)
    relief += max(0.0, q_ratio - 0.78) * math.radians(0.75)
    relief += max(0.0, load_ratio - 0.82) * math.radians(0.45)
    if margin < 3_000.0:
        relief += (3_000.0 - margin) / 3_000.0 * math.radians(0.85)

    return clamp(command + relief, -params.gamma_rate_limit, params.gamma_rate_limit)


def simulate(controller: str, params: ReentryParams | None = None) -> tuple[list[dict[str, float]], dict[str, float | str | bool]]:
    if controller not in {"baseline", "heat_guard"}:
        raise ValueError(f"unknown controller: {controller}")

    params = params or ReentryParams()
    state = make_initial_state()
    trace: list[dict[str, float]] = []
    heat_load = 0.0
    steps = int(params.duration / params.dt)
    for step in range(steps + 1):
        time_s = step * params.dt
        if controller == "baseline":
            gamma_rate_cmd = command_baseline(state, time_s, params)
        else:
            gamma_rate_cmd = command_heat_guard(state, time_s, params)

        qbar = dynamic_pressure(state, params)
        hdot = heat_rate(state, params)
        n_load = load_factor(state, gamma_rate_cmd, params)
        lower, upper = corridor_bounds(state.velocity)
        margin = min(state.altitude - lower, upper - state.altitude)
        heat_load += hdot * params.dt
        trace.append(
            {
                "time": time_s,
                "controller": 0.0 if controller == "baseline" else 1.0,
                "downrange": state.downrange,
                "altitude": state.altitude,
                "velocity": state.velocity,
                "gamma": state.gamma,
                "gamma_rate_cmd": gamma_rate_cmd,
                "qbar": qbar,
                "heat_rate": hdot,
                "heat_load": heat_load,
                "load_factor": n_load,
                "corridor_lower": lower,
                "corridor_upper": upper,
                "corridor_margin": margin,
            }
        )
        state = rk4_step(state, gamma_rate_cmd, params)

    metrics = summarize(trace, controller, params)
    return trace, metrics


def summarize(trace: list[dict[str, float]], controller: str, params: ReentryParams) -> dict[str, float | str | bool]:
    last = trace[-1]
    heat_values = [row["heat_rate"] for row in trace]
    q_values = [row["qbar"] for row in trace]
    load_values = [row["load_factor"] for row in trace]
    corridor_margins = [row["corridor_margin"] for row in trace]
    gamma_rate_values = [abs(row["gamma_rate_cmd"]) for row in trace]
    heat_violation = violation_integral(heat_values, params.heat_rate_limit, params.dt)
    q_violation = violation_integral(q_values, params.q_limit, params.dt)
    load_violation = violation_integral(load_values, params.load_factor_limit, params.dt)
    corridor_violation = sum(max(0.0, -value) for value in corridor_margins) * params.dt
    terminal_downrange_error = last["downrange"] - params.downrange_target
    terminal_altitude_error = last["altitude"] - params.terminal_altitude_target
    terminal_velocity_error = last["velocity"] - params.terminal_velocity_target
    terminal_pass = (
        abs(terminal_downrange_error) <= 45_000.0
        and abs(terminal_altitude_error) <= 8_000.0
        and abs(terminal_velocity_error) <= 1_000.0
    )
    heat_pass = heat_violation <= 1.0 and last["heat_load"] <= params.heat_load_limit
    q_pass = q_violation <= 1.0
    load_pass = load_violation <= 0.05
    corridor_pass = corridor_violation <= 1.0
    input_pass = max(gamma_rate_values) <= params.gamma_rate_limit + 1e-12

    return {
        "controller": controller,
        "model": "reduced_reentry_point_mass",
        "terminal_downrange_error_m": terminal_downrange_error,
        "terminal_altitude_error_m": terminal_altitude_error,
        "terminal_velocity_error_m_s": terminal_velocity_error,
        "heat_rate_max": max(heat_values) if heat_values else 0.0,
        "heat_rate_margin_min": min((params.heat_rate_limit - value for value in heat_values), default=0.0),
        "heat_rate_violation_integral": heat_violation,
        "heat_load": last["heat_load"],
        "heat_load_margin": params.heat_load_limit - last["heat_load"],
        "q_max": max(q_values) if q_values else 0.0,
        "q_margin_min": min((params.q_limit - value for value in q_values), default=0.0),
        "q_violation_integral": q_violation,
        "load_factor_max": max(load_values) if load_values else 0.0,
        "load_factor_violation_integral": load_violation,
        "corridor_margin_min": min(corridor_margins) if corridor_margins else 0.0,
        "corridor_violation_integral": corridor_violation,
        "gamma_rate_peak_rad_s": max(gamma_rate_values) if gamma_rate_values else 0.0,
        "terminal_pass": terminal_pass,
        "heat_pass": heat_pass,
        "q_pass": q_pass,
        "load_pass": load_pass,
        "corridor_pass": corridor_pass,
        "input_pass": input_pass,
        "case_pass": terminal_pass and heat_pass and q_pass and load_pass and corridor_pass and input_pass,
    }


def run(controller: str = "all") -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    if controller not in {"all", "baseline", "heat_guard"}:
        raise ValueError(f"unknown controller: {controller}")
    controllers = ["baseline", "heat_guard"] if controller == "all" else [controller]
    rows = [simulate(name)[1] for name in controllers]
    summary: dict[str, float | bool] = {}
    if len(rows) == 2:
        baseline, guarded = rows
        summary = {
            "baseline_heat_rate_violation_integral": float(baseline["heat_rate_violation_integral"]),
            "guarded_heat_rate_violation_integral": float(guarded["heat_rate_violation_integral"]),
            "baseline_corridor_margin_min": float(baseline["corridor_margin_min"]),
            "guarded_corridor_margin_min": float(guarded["corridor_margin_min"]),
            "baseline_case_pass": bool(baseline["case_pass"]),
            "guarded_case_pass": bool(guarded["case_pass"]),
        }
    return rows, summary


def write_metrics_csv(rows: list[dict[str, float | str | bool]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_trace_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["all", "baseline", "heat_guard"], default="all")
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--trace-output", type=Path, default=None)
    args = parser.parse_args()

    rows, summary = run(args.controller)
    if args.metrics_output is not None:
        write_metrics_csv(rows, args.metrics_output)
    if args.trace_output is not None:
        trace_controller = "heat_guard" if args.controller == "all" else args.controller
        trace, _metrics = simulate(trace_controller)
        write_trace_csv(trace, args.trace_output)
    for row in rows:
        print(f"controller: {row['controller']}")
        for key, value in row.items():
            if key != "controller":
                print(f"{key}: {value}")
    if summary:
        for key, value in summary.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
