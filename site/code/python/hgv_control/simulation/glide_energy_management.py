"""Teaching glide energy-management control case.

This module supports case C15 in Volume 5. It uses a reduced point-mass
hypersonic glide model with downrange, crossrange, altitude, speed, flight-path
angle, and heading. The energy-aware controller limits bank and adjusts angle
of attack when the vehicle is at risk of falling below the energy corridor.

The implementation is intentionally lightweight. It teaches energy-height,
range, crossrange, and corridor evidence fields; it is not a high-fidelity
trajectory optimizer or bank-reversal guidance law.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import violation_integral
from hgv_control.models.longitudinal import clamp
from hgv_control.models.point_mass_3d import wrap_angle


@dataclass(frozen=True)
class GlideParams:
    dt: float = 0.1
    duration: float = 95.0
    mass: float = 1_100.0
    ref_area: float = 1.35
    gravity: float = 9.80665
    rho0: float = 1.225
    scale_height: float = 7_200.0
    cl0: float = 0.05
    cl_alpha: float = 3.25
    cd0: float = 0.055
    induced_drag: float = 0.105
    alpha_min: float = math.radians(4.0)
    alpha_max: float = math.radians(14.0)
    bank_limit: float = math.radians(55.0)
    bank_rate_limit: float = math.radians(4.0)
    q_limit: float = 55_000.0
    heat_rate_limit: float = 5.5e5
    load_factor_limit: float = 3.5
    downrange_target: float = 220_000.0
    crossrange_target: float = 0.0
    terminal_altitude_target: float = 41_000.0
    terminal_velocity_target: float = 2_200.0


@dataclass(frozen=True)
class GlideState:
    downrange: float
    crossrange: float
    altitude: float
    velocity: float
    gamma: float
    heading: float
    bank: float


@dataclass(frozen=True)
class GlideCommand:
    alpha: float
    bank_cmd: float


def make_initial_state() -> GlideState:
    return GlideState(
        downrange=0.0,
        crossrange=22_000.0,
        altitude=44_000.0,
        velocity=2_450.0,
        gamma=math.radians(-3.2),
        heading=math.radians(-1.0),
        bank=0.0,
    )


def atmosphere_density(altitude: float, params: GlideParams) -> float:
    return params.rho0 * math.exp(-max(0.0, altitude) / params.scale_height)


def aero_coefficients(alpha: float, params: GlideParams) -> tuple[float, float]:
    alpha = clamp(alpha, params.alpha_min, params.alpha_max)
    cl = params.cl0 + params.cl_alpha * alpha
    cd = params.cd0 + params.induced_drag * cl * cl
    return cl, cd


def dynamic_pressure(state: GlideState, params: GlideParams) -> float:
    return 0.5 * atmosphere_density(state.altitude, params) * state.velocity * state.velocity


def heat_rate(state: GlideState, params: GlideParams) -> float:
    return 1.05e-4 * math.sqrt(atmosphere_density(state.altitude, params)) * state.velocity**3


def energy_height(state: GlideState, params: GlideParams) -> float:
    return state.altitude + state.velocity * state.velocity / (2.0 * params.gravity)


def target_energy_height(params: GlideParams) -> float:
    return params.terminal_altitude_target + params.terminal_velocity_target**2 / (2.0 * params.gravity)


def energy_reference(downrange: float, params: GlideParams) -> float:
    start = energy_height(make_initial_state(), params)
    target = target_energy_height(params)
    progress = clamp(downrange / params.downrange_target, 0.0, 1.0)
    return start + progress * (target - start)


def energy_corridor(downrange: float, params: GlideParams) -> tuple[float, float]:
    reference = energy_reference(downrange, params)
    lower = reference - 18_000.0
    upper = reference + 24_000.0
    return lower, upper


def energy_corridor_margin(state: GlideState, params: GlideParams) -> float:
    lower, upper = energy_corridor(state.downrange, params)
    energy = energy_height(state, params)
    return min(energy - lower, upper - energy)


def forces(state: GlideState, command: GlideCommand, params: GlideParams) -> tuple[float, float, float, float]:
    qbar = dynamic_pressure(state, params)
    cl, cd = aero_coefficients(command.alpha, params)
    lift = qbar * params.ref_area * cl
    drag = qbar * params.ref_area * cd
    return lift, drag, cl, cd


def load_factor(state: GlideState, command: GlideCommand, params: GlideParams) -> float:
    lift, _drag, _cl, _cd = forces(state, command, params)
    return abs(lift) / max(params.mass * params.gravity, 1e-9)


def derivatives(state: GlideState, command: GlideCommand, params: GlideParams) -> GlideState:
    velocity = max(300.0, state.velocity)
    bank_cmd = clamp(command.bank_cmd, -params.bank_limit, params.bank_limit)
    bank_rate = clamp(bank_cmd - state.bank, -params.bank_rate_limit, params.bank_rate_limit)
    lift, drag, _cl, _cd = forces(state, command, params)
    bank_mid = state.bank
    gamma_dot = lift * math.cos(bank_mid) / (params.mass * velocity) - params.gravity * math.cos(state.gamma) / velocity
    heading_dot = lift * math.sin(bank_mid) / (params.mass * velocity * max(math.cos(state.gamma), 0.2))
    return GlideState(
        downrange=velocity * math.cos(state.gamma) * math.cos(state.heading),
        crossrange=velocity * math.cos(state.gamma) * math.sin(state.heading),
        altitude=velocity * math.sin(state.gamma),
        velocity=-drag / params.mass - params.gravity * math.sin(state.gamma),
        gamma=gamma_dot,
        heading=heading_dot,
        bank=bank_rate,
    )


def add_state(state: GlideState, slope: GlideState, scale: float, params: GlideParams) -> GlideState:
    return GlideState(
        downrange=state.downrange + scale * slope.downrange,
        crossrange=state.crossrange + scale * slope.crossrange,
        altitude=max(0.0, state.altitude + scale * slope.altitude),
        velocity=max(300.0, state.velocity + scale * slope.velocity),
        gamma=clamp(state.gamma + scale * slope.gamma, math.radians(-12.0), math.radians(4.0)),
        heading=wrap_angle(state.heading + scale * slope.heading),
        bank=clamp(state.bank + scale * slope.bank, -params.bank_limit, params.bank_limit),
    )


def rk4_step(state: GlideState, command: GlideCommand, params: GlideParams) -> GlideState:
    dt = params.dt
    k1 = derivatives(state, command, params)
    k2 = derivatives(add_state(state, k1, 0.5 * dt, params), command, params)
    k3 = derivatives(add_state(state, k2, 0.5 * dt, params), command, params)
    k4 = derivatives(add_state(state, k3, dt, params), command, params)
    next_state = GlideState(
        downrange=state.downrange + dt * (k1.downrange + 2.0 * k2.downrange + 2.0 * k3.downrange + k4.downrange) / 6.0,
        crossrange=state.crossrange + dt * (k1.crossrange + 2.0 * k2.crossrange + 2.0 * k3.crossrange + k4.crossrange) / 6.0,
        altitude=state.altitude + dt * (k1.altitude + 2.0 * k2.altitude + 2.0 * k3.altitude + k4.altitude) / 6.0,
        velocity=state.velocity + dt * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) / 6.0,
        gamma=state.gamma + dt * (k1.gamma + 2.0 * k2.gamma + 2.0 * k3.gamma + k4.gamma) / 6.0,
        heading=state.heading + dt * (k1.heading + 2.0 * k2.heading + 2.0 * k3.heading + k4.heading) / 6.0,
        bank=state.bank + dt * (k1.bank + 2.0 * k2.bank + 2.0 * k3.bank + k4.bank) / 6.0,
    )
    return GlideState(
        downrange=next_state.downrange,
        crossrange=next_state.crossrange,
        altitude=max(0.0, next_state.altitude),
        velocity=max(300.0, next_state.velocity),
        gamma=clamp(next_state.gamma, math.radians(-12.0), math.radians(4.0)),
        heading=wrap_angle(next_state.heading),
        bank=clamp(next_state.bank, -params.bank_limit, params.bank_limit),
    )


def baseline_command(state: GlideState, params: GlideParams) -> GlideCommand:
    bank_cmd = -4.2e-5 * state.crossrange - 0.9 * state.heading
    return GlideCommand(
        alpha=math.radians(11.5),
        bank_cmd=clamp(bank_cmd, -params.bank_limit, params.bank_limit),
    )


def energy_guard_command(state: GlideState, params: GlideParams) -> GlideCommand:
    lower, upper = energy_corridor(state.downrange, params)
    energy = energy_height(state, params)
    error_to_reference = energy - energy_reference(state.downrange, params)
    energy_low = max(0.0, (lower + 7_000.0 - energy) / 9_000.0)
    energy_high = max(0.0, (energy - (upper - 8_000.0)) / 12_000.0)
    alpha = math.radians(8.0) + math.radians(4.0) * clamp(energy_high, 0.0, 1.0)
    alpha -= math.radians(2.4) * clamp(energy_low, 0.0, 1.0)
    alpha += math.radians(0.000012) * clamp(error_to_reference, -8_000.0, 8_000.0)
    alpha = clamp(alpha, params.alpha_min, params.alpha_max)

    base_bank = -3.6e-5 * state.crossrange - 0.8 * state.heading
    bank_limit = params.bank_limit * (1.0 - 0.55 * clamp(energy_low, 0.0, 1.0))
    if abs(state.crossrange) < 4_000.0 and energy_low > 0.15:
        bank_limit *= 0.55
    return GlideCommand(alpha=alpha, bank_cmd=clamp(base_bank, -bank_limit, bank_limit))


def simulate(controller: str, params: GlideParams | None = None) -> tuple[list[dict[str, float]], dict[str, float | str | bool]]:
    if controller not in {"baseline", "energy_guard"}:
        raise ValueError(f"unknown controller: {controller}")

    params = params or GlideParams()
    state = make_initial_state()
    trace: list[dict[str, float]] = []
    previous_bank = state.bank
    steps = int(params.duration / params.dt)
    for step in range(steps + 1):
        time_s = step * params.dt
        command = baseline_command(state, params) if controller == "baseline" else energy_guard_command(state, params)
        lift, drag, cl, cd = forces(state, command, params)
        lower, upper = energy_corridor(state.downrange, params)
        energy = energy_height(state, params)
        bank_rate = (state.bank - previous_bank) / params.dt if step > 0 else 0.0
        previous_bank = state.bank
        trace.append(
            {
                "time": time_s,
                "controller": 0.0 if controller == "baseline" else 1.0,
                "downrange": state.downrange,
                "crossrange": state.crossrange,
                "altitude": state.altitude,
                "velocity": state.velocity,
                "gamma": state.gamma,
                "heading": state.heading,
                "bank": state.bank,
                "bank_cmd": command.bank_cmd,
                "bank_rate": bank_rate,
                "alpha_cmd": command.alpha,
                "qbar": dynamic_pressure(state, params),
                "heat_rate": heat_rate(state, params),
                "load_factor": load_factor(state, command, params),
                "energy_height": energy,
                "energy_ref": energy_reference(state.downrange, params),
                "energy_lower": lower,
                "energy_upper": upper,
                "energy_corridor_margin": min(energy - lower, upper - energy),
                "lift_to_drag": cl / max(cd, 1e-9),
                "lift": lift,
                "drag": drag,
            }
        )
        state = rk4_step(state, command, params)
    return trace, summarize(trace, controller, params)


def summarize(trace: list[dict[str, float]], controller: str, params: GlideParams) -> dict[str, float | str | bool]:
    last = trace[-1]
    q_values = [row["qbar"] for row in trace]
    heat_values = [row["heat_rate"] for row in trace]
    load_values = [row["load_factor"] for row in trace]
    energy_margins = [row["energy_corridor_margin"] for row in trace]
    bank_values = [abs(row["bank"]) for row in trace]
    bank_rate_values = [abs(row["bank_rate"]) for row in trace]
    lift_to_drag_values = [row["lift_to_drag"] for row in trace]
    terminal_range_error = last["downrange"] - params.downrange_target
    crossrange_error = last["crossrange"] - params.crossrange_target
    terminal_energy_error = last["energy_height"] - target_energy_height(params)
    q_violation = violation_integral(q_values, params.q_limit, params.dt)
    heat_violation = violation_integral(heat_values, params.heat_rate_limit, params.dt)
    load_violation = violation_integral(load_values, params.load_factor_limit, params.dt)
    energy_violation = sum(max(0.0, -value) for value in energy_margins) * params.dt
    terminal_pass = (
        abs(terminal_range_error) <= 28_000.0
        and abs(crossrange_error) <= 6_000.0
        and abs(terminal_energy_error) <= 8_000.0
    )
    energy_pass = energy_violation <= 1.0 and min(energy_margins) >= -1e-9
    path_pass = q_violation <= 1.0 and heat_violation <= 1.0 and load_violation <= 0.05
    input_pass = max(bank_values) <= params.bank_limit + 1e-12 and max(bank_rate_values) <= params.bank_rate_limit + 1e-9

    return {
        "controller": controller,
        "model": "reduced_glide_energy_point_mass",
        "terminal_range_error_m": terminal_range_error,
        "crossrange_error_m": crossrange_error,
        "terminal_energy_error_m": terminal_energy_error,
        "energy_corridor_margin_min": min(energy_margins) if energy_margins else 0.0,
        "energy_corridor_violation_integral": energy_violation,
        "q_max": max(q_values) if q_values else 0.0,
        "q_violation_integral": q_violation,
        "heat_rate_max": max(heat_values) if heat_values else 0.0,
        "heat_violation_integral": heat_violation,
        "load_factor_max": max(load_values) if load_values else 0.0,
        "load_violation_integral": load_violation,
        "bank_angle_peak_rad": max(bank_values) if bank_values else 0.0,
        "bank_rate_peak_rad_s": max(bank_rate_values) if bank_rate_values else 0.0,
        "lift_to_drag_mean": sum(lift_to_drag_values) / len(lift_to_drag_values) if lift_to_drag_values else 0.0,
        "terminal_pass": terminal_pass,
        "energy_pass": energy_pass,
        "path_pass": path_pass,
        "input_pass": input_pass,
        "case_pass": terminal_pass and energy_pass and path_pass and input_pass,
    }


def run(controller: str = "all") -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    if controller not in {"all", "baseline", "energy_guard"}:
        raise ValueError(f"unknown controller: {controller}")
    controllers = ["baseline", "energy_guard"] if controller == "all" else [controller]
    rows = [simulate(name)[1] for name in controllers]
    summary: dict[str, float | bool] = {}
    if len(rows) == 2:
        baseline, guarded = rows
        summary = {
            "baseline_energy_corridor_violation_integral": float(baseline["energy_corridor_violation_integral"]),
            "guarded_energy_corridor_violation_integral": float(guarded["energy_corridor_violation_integral"]),
            "baseline_terminal_energy_error_m": float(baseline["terminal_energy_error_m"]),
            "guarded_terminal_energy_error_m": float(guarded["terminal_energy_error_m"]),
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
    parser.add_argument("--controller", choices=["all", "baseline", "energy_guard"], default="all")
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--trace-output", type=Path, default=None)
    args = parser.parse_args()

    rows, summary = run(args.controller)
    if args.metrics_output is not None:
        write_metrics_csv(rows, args.metrics_output)
    if args.trace_output is not None:
        trace_controller = "energy_guard" if args.controller == "all" else args.controller
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
