"""Finite-set MPC trajectory-tracking teaching case.

This module supports case C18 in Volume 5. It uses a reduced longitudinal
velocity-altitude-gamma model and an enumerated finite control set to teach
MPC evidence fields: prediction horizon, constraints, solver time, empirical
feasibility, fallback count, and tracking/safety tradeoffs.

The implementation is intentionally lightweight. It is not a QP/NMPC solver
and does not provide strict recursive-feasibility or stability guarantees.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from time import perf_counter

from hgv_control.metrics.summary import rms, violation_integral
from hgv_control.models.longitudinal import LongitudinalParams, atmosphere_density, clamp


TRACKING_V_RMS_TARGET = 180.0
TRACKING_H_RMS_TARGET = 800.0
SOLVER_TIME_TARGET_MS = 50.0


@dataclass(frozen=True)
class ReducedState:
    velocity: float
    altitude: float
    gamma: float


@dataclass(frozen=True)
class MpcConfig:
    dt: float = 0.25
    duration: float = 32.0
    horizon_steps: int = 12
    gamma_time_constant: float = 1.4
    alpha_limit: float = math.radians(6.0)
    q_limit: float = 45_000.0
    gamma_candidates_rad: tuple[float, ...] = tuple(math.radians(value) for value in (-5.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0, 7.0))
    throttle_candidates: tuple[float, ...] = (0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75)


def make_initial_state() -> ReducedState:
    return ReducedState(velocity=1_750.0, altitude=30_000.0, gamma=math.radians(-0.5))


def trajectory_reference(time_s: float) -> tuple[float, float]:
    """Return speed and altitude references for an intentionally demanding path."""

    velocity_ref = 1_830.0 if time_s < 16.0 else 1_760.0
    altitude_ref = 28_500.0 if time_s < 18.0 else 30_000.0
    return velocity_ref, altitude_ref


def reduced_qbar(state: ReducedState, params: LongitudinalParams) -> float:
    return 0.5 * atmosphere_density(state.altitude, params) * state.velocity * state.velocity


def reduced_step(
    state: ReducedState,
    gamma_cmd: float,
    throttle_cmd: float,
    config: MpcConfig,
    params: LongitudinalParams,
) -> ReducedState:
    alpha_eff = clamp(gamma_cmd - state.gamma, -config.alpha_limit, config.alpha_limit)
    qbar = reduced_qbar(state, params)
    cd = params.cd0 + params.cd_alpha2 * alpha_eff * alpha_eff
    drag = qbar * params.ref_area * cd
    thrust = params.thrust_max * clamp(throttle_cmd, params.throttle_min, params.throttle_max)
    v_dot = (thrust - drag) / params.mass - params.gravity * math.sin(state.gamma)
    h_dot = state.velocity * math.sin(state.gamma)
    gamma_dot = (gamma_cmd - state.gamma) / max(config.gamma_time_constant, 1e-6)

    return ReducedState(
        velocity=max(100.0, state.velocity + config.dt * v_dot),
        altitude=max(0.0, state.altitude + config.dt * h_dot),
        gamma=clamp(state.gamma + config.dt * gamma_dot, math.radians(-8.0), math.radians(8.0)),
    )


def naive_command(state: ReducedState, time_s: float) -> tuple[float, float]:
    velocity_ref, altitude_ref = trajectory_reference(time_s)
    gamma_cmd = clamp(1.4e-4 * (altitude_ref - state.altitude), math.radians(-5.0), math.radians(5.0))
    throttle_cmd = clamp(0.42 + 1.2e-3 * (velocity_ref - state.velocity), 0.15, 0.85)
    return gamma_cmd, throttle_cmd


def predict_cost(
    state: ReducedState,
    gamma_cmd: float,
    throttle_cmd: float,
    time_s: float,
    config: MpcConfig,
    params: LongitudinalParams,
) -> tuple[float, float, bool]:
    predicted = state
    violation_sum = 0.0
    feasible = True
    cost = 0.0
    for step in range(config.horizon_steps):
        predicted = reduced_step(predicted, gamma_cmd, throttle_cmd, config, params)
        velocity_ref, altitude_ref = trajectory_reference(time_s + step * config.dt)
        qbar = reduced_qbar(predicted, params)
        q_violation = max(0.0, qbar - config.q_limit) / config.q_limit
        violation_sum += q_violation
        feasible = feasible and q_violation <= 1e-12
        cost += (
            ((predicted.velocity - velocity_ref) / 100.0) ** 2
            + ((predicted.altitude - altitude_ref) / 600.0) ** 2
            + 200.0 * q_violation * q_violation
            + 0.04 * ((throttle_cmd - 0.45) / 0.30) ** 2
            + 0.08 * (gamma_cmd / math.radians(5.0)) ** 2
        )
    return cost, violation_sum, feasible


def mpc_command(
    state: ReducedState,
    time_s: float,
    config: MpcConfig,
    params: LongitudinalParams,
) -> tuple[float, float, bool, float]:
    start = perf_counter()
    best: tuple[bool, float, float, float, float, bool] | None = None
    for gamma_cmd in config.gamma_candidates_rad:
        for throttle_cmd in config.throttle_candidates:
            cost, violation_sum, feasible = predict_cost(state, gamma_cmd, throttle_cmd, time_s, config, params)
            score = cost + 5_000.0 * violation_sum
            record = (not feasible, score, violation_sum, gamma_cmd, throttle_cmd, feasible)
            if best is None or record < best:
                best = record
    if best is None:
        raise RuntimeError("empty MPC candidate set")
    solve_time_ms = 1_000.0 * (perf_counter() - start)
    return best[3], best[4], best[5], solve_time_ms


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered)) - 1)))
    return ordered[index]


def simulate(
    controller: str,
    config: MpcConfig | None = None,
    params: LongitudinalParams | None = None,
) -> dict[str, float | str | bool]:
    if controller not in {"naive", "finite_set_mpc"}:
        raise ValueError(f"unknown controller: {controller}")

    config = config or MpcConfig()
    params = params or LongitudinalParams(q_limit=config.q_limit)
    state = make_initial_state()
    velocity_errors: list[float] = []
    altitude_errors: list[float] = []
    q_values: list[float] = []
    solve_times: list[float] = []
    throttle_values: list[float] = []
    gamma_commands: list[float] = []
    fallback_count = 0
    feasibility_failures = 0

    steps = int(config.duration / config.dt)
    for step in range(steps + 1):
        time_s = step * config.dt
        velocity_ref, altitude_ref = trajectory_reference(time_s)
        velocity_errors.append(state.velocity - velocity_ref)
        altitude_errors.append(state.altitude - altitude_ref)
        q_values.append(reduced_qbar(state, params))

        if controller == "naive":
            gamma_cmd, throttle_cmd = naive_command(state, time_s)
        else:
            gamma_cmd, throttle_cmd, feasible, solve_time_ms = mpc_command(state, time_s, config, params)
            solve_times.append(solve_time_ms)
            if not feasible:
                fallback_count += 1
                feasibility_failures += 1

        gamma_commands.append(gamma_cmd)
        throttle_values.append(throttle_cmd)
        state = reduced_step(state, gamma_cmd, throttle_cmd, config, params)

    throttle_rate_peak = 0.0
    gamma_command_rate_peak = 0.0
    for previous, current in zip(throttle_values, throttle_values[1:]):
        throttle_rate_peak = max(throttle_rate_peak, abs(current - previous) / config.dt)
    for previous, current in zip(gamma_commands, gamma_commands[1:]):
        gamma_command_rate_peak = max(gamma_command_rate_peak, abs(current - previous) / config.dt)

    v_rms = rms(velocity_errors)
    h_rms = rms(altitude_errors)
    q_violation = violation_integral(q_values, config.q_limit, config.dt)
    q_margin_min = min((config.q_limit - value for value in q_values), default=0.0)
    solve_time_max = max(solve_times) if solve_times else 0.0
    solve_time_p95 = percentile(solve_times, 0.95)
    solver_success_rate = 1.0 if controller == "naive" else 1.0 - fallback_count / max(1, len(solve_times))
    tracking_pass = v_rms <= TRACKING_V_RMS_TARGET and h_rms <= TRACKING_H_RMS_TARGET
    q_pass = q_violation <= 1.0
    realtime_pass = controller == "naive" or solve_time_max <= SOLVER_TIME_TARGET_MS
    feasibility_pass = fallback_count == 0

    return {
        "controller": controller,
        "model": "reduced_velocity_altitude_gamma",
        "prediction_horizon_steps": 0 if controller == "naive" else config.horizon_steps,
        "dt_s": config.dt,
        "q_limit_pa": config.q_limit,
        "v_rms_m_s": v_rms,
        "h_rms_m": h_rms,
        "q_max_pa": max(q_values) if q_values else 0.0,
        "q_margin_min_pa": q_margin_min,
        "q_violation_integral": q_violation,
        "throttle_rate_peak_per_s": throttle_rate_peak,
        "gamma_command_rate_peak_rad_s": gamma_command_rate_peak,
        "solver_success_rate": solver_success_rate,
        "solver_time_max_ms": solve_time_max,
        "solver_time_p95_ms": solve_time_p95,
        "fallback_count": fallback_count,
        "recursive_feasibility_failures": feasibility_failures,
        "tracking_pass": tracking_pass,
        "q_pass": q_pass,
        "realtime_pass": realtime_pass,
        "feasibility_pass": feasibility_pass,
        "case_pass": tracking_pass and q_pass and realtime_pass and feasibility_pass,
    }


def run(controller: str = "all") -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    if controller not in {"all", "naive", "finite_set_mpc"}:
        raise ValueError(f"unknown controller: {controller}")
    controllers = ["naive", "finite_set_mpc"] if controller == "all" else [controller]
    rows = [simulate(name) for name in controllers]
    summary: dict[str, float | bool] = {}
    if len(rows) == 2:
        naive, mpc = rows
        summary = {
            "naive_q_violation_integral": float(naive["q_violation_integral"]),
            "mpc_q_violation_integral": float(mpc["q_violation_integral"]),
            "naive_v_rms_m_s": float(naive["v_rms_m_s"]),
            "mpc_v_rms_m_s": float(mpc["v_rms_m_s"]),
            "mpc_solver_time_max_ms": float(mpc["solver_time_max_ms"]),
            "mpc_fallback_count": float(mpc["fallback_count"]),
            "naive_case_pass": bool(naive["case_pass"]),
            "mpc_case_pass": bool(mpc["case_pass"]),
        }
    return rows, summary


def write_csv(rows: list[dict[str, float | str | bool]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["all", "naive", "finite_set_mpc"], default="all")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rows, summary = run(args.controller)
    if args.output is not None:
        write_csv(rows, args.output)
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
