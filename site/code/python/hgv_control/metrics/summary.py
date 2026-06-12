"""Metric computation for teaching simulations."""

from __future__ import annotations

import math


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def violation_integral(values: list[float], limit: float, dt: float) -> float:
    return sum(max(0.0, value - limit) for value in values) * dt


def summarize(
    trace: list[dict[str, float]],
    q_limit: float,
    dt: float,
    alpha_limit: float = math.radians(12.0),
    delta_limit: float = math.inf,
    delta_rate_limit: float = math.inf,
) -> dict[str, float | bool]:
    h_errors = [row["altitude"] - row["altitude_ref"] for row in trace]
    v_errors = [row["velocity"] - row["velocity_ref"] for row in trace]
    q_values = [row["qbar"] for row in trace]
    delta_values = [abs(row["elevator"]) for row in trace]
    delta_rate_values = [abs(row["elevator_rate"]) for row in trace]
    alpha_values = [abs(row["alpha"]) for row in trace]
    throttle_values = [row["throttle_cmd"] for row in trace]

    h_rms = rms(h_errors)
    v_rms = rms(v_errors)
    q_max = max(q_values) if q_values else 0.0
    q_margin_min = min((q_limit - value for value in q_values), default=0.0)
    q_violation = violation_integral(q_values, q_limit, dt)
    delta_max = max(delta_values) if delta_values else 0.0
    delta_rate_max = max(delta_rate_values) if delta_rate_values else 0.0
    alpha_max = max(alpha_values) if alpha_values else 0.0
    throttle_span = (max(throttle_values) - min(throttle_values)) if throttle_values else 0.0
    tracking_pass = h_rms <= 900.0 and v_rms <= 450.0
    q_pass = q_violation <= 1.0
    alpha_pass = alpha_max <= alpha_limit
    delta_pass = delta_max <= delta_limit + 1e-9
    delta_rate_pass = delta_rate_max <= delta_rate_limit + 1e-9

    return {
        "h_rms": h_rms,
        "v_rms": v_rms,
        "q_max": q_max,
        "q_margin_min": q_margin_min,
        "q_violation_integral": q_violation,
        "alpha_max_rad": alpha_max,
        "delta_max_rad": delta_max,
        "delta_rate_max_rad_s": delta_rate_max,
        "throttle_span": throttle_span,
        "tracking_pass": tracking_pass,
        "q_pass": q_pass,
        "alpha_pass": alpha_pass,
        "delta_pass": delta_pass,
        "delta_rate_pass": delta_rate_pass,
        "pass": tracking_pass and q_pass and alpha_pass and delta_pass and delta_rate_pass,
    }
