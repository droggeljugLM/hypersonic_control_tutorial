"""Input saturation and anti-windup teaching case.

This module supports case C12 in Volume 5. It intentionally uses the
compact longitudinal model from the tutorial, so the conclusions are
limited to teaching-level evidence.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import rms, violation_integral
from hgv_control.models.longitudinal import LongitudinalParams, State, clamp, dynamic_pressure, rk4_step
from hgv_control.simulation.run_case import make_initial_state


@dataclass
class SaturatingSpeedController:
    anti_windup: bool
    altitude_gain: float = 1.2e-4
    gamma_gain: float = 1.05
    pitch_gain: float = 0.9
    q_gain: float = 0.5
    speed_gain: float = 1.3e-3
    integral_gain: float = 8.5e-5
    trim_throttle: float = 0.42
    alpha_trim: float = math.radians(2.0)
    speed_integral: float = 0.0

    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
        dt: float,
    ) -> tuple[float, float, float, bool]:
        gamma_cmd = clamp(self.altitude_gain * (altitude_ref - state.altitude), math.radians(-7.0), math.radians(7.0))
        alpha_cmd = self.alpha_trim + clamp(self.gamma_gain * (gamma_cmd - state.gamma), math.radians(-5.0), math.radians(5.0))
        theta_cmd = alpha_cmd + state.gamma
        delta_cmd = -self.pitch_gain * (theta_cmd - state.theta) + self.q_gain * state.pitch_rate
        delta_cmd = clamp(delta_cmd, -params.delta_limit, params.delta_limit)

        speed_error = velocity_ref - state.velocity
        throttle_unsat = self.trim_throttle + self.speed_gain * speed_error + self.integral_gain * self.speed_integral
        throttle_cmd = clamp(throttle_unsat, params.throttle_min, params.throttle_max)
        saturated_high = throttle_unsat > params.throttle_max
        saturated_low = throttle_unsat < params.throttle_min
        saturation_active = saturated_high or saturated_low

        if self.anti_windup:
            drives_deeper = (saturated_high and speed_error > 0.0) or (saturated_low and speed_error < 0.0)
            if not drives_deeper:
                self.speed_integral += speed_error * dt
        else:
            self.speed_integral += speed_error * dt

        return delta_cmd, throttle_cmd, throttle_unsat, saturation_active


def reference(time_s: float) -> tuple[float, float]:
    altitude_ref = 30_000.0
    if time_s < 16.0:
        velocity_ref = 2_250.0
    elif time_s < 24.0:
        velocity_ref = 1_720.0
    else:
        velocity_ref = 1_780.0
    return altitude_ref, velocity_ref


def run(
    controller_name: str = "anti_windup",
    duration: float = 36.0,
    dt: float = 0.04,
    params: LongitudinalParams | None = None,
    initial_state: State | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    if controller_name not in {"no_anti_windup", "anti_windup"}:
        raise ValueError(f"unknown controller: {controller_name}")

    params = params or LongitudinalParams(thrust_max=18_000.0, throttle_max=0.62)
    state = initial_state or make_initial_state()
    controller = SaturatingSpeedController(anti_windup=controller_name == "anti_windup")

    trace: list[dict[str, float]] = []
    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        altitude_ref, velocity_ref = reference(time_s)
        delta_cmd, throttle_cmd, throttle_unsat, saturation_active = controller.command(
            state, altitude_ref, velocity_ref, params, dt
        )
        qbar = dynamic_pressure(state, params)
        trace.append(
            {
                "time": time_s,
                "velocity": state.velocity,
                "velocity_ref": velocity_ref,
                "altitude": state.altitude,
                "altitude_ref": altitude_ref,
                "gamma": state.gamma,
                "alpha": state.alpha,
                "theta": state.theta,
                "pitch_rate": state.pitch_rate,
                "elevator": state.elevator,
                "elevator_rate": state.elevator_rate,
                "delta_cmd": delta_cmd,
                "throttle_cmd": throttle_cmd,
                "throttle_unsat": throttle_unsat,
                "speed_error": velocity_ref - state.velocity,
                "speed_integral": controller.speed_integral,
                "saturation_active": 1.0 if saturation_active else 0.0,
                "qbar": qbar,
            }
        )
        state = rk4_step(state, delta_cmd, throttle_cmd, dt, params)

    metrics = summarize(trace, params, dt, release_time=16.0)
    metrics["anti_windup"] = controller_name == "anti_windup"
    return trace, metrics


def summarize(
    trace: list[dict[str, float]],
    params: LongitudinalParams,
    dt: float,
    release_time: float,
) -> dict[str, float | bool]:
    velocity_errors = [row["velocity"] - row["velocity_ref"] for row in trace]
    altitude_errors = [row["altitude"] - row["altitude_ref"] for row in trace]
    post_rows = [row for row in trace if row["time"] >= release_time]
    post_velocity_errors = [row["velocity"] - row["velocity_ref"] for row in post_rows]
    post_overshoot = max((row["velocity"] - row["velocity_ref"] for row in post_rows), default=0.0)
    q_values = [row["qbar"] for row in trace]
    delta_values = [abs(row["elevator"]) for row in trace]
    delta_rate_values = [abs(row["elevator_rate"]) for row in trace]
    throttle_unsat_values = [row["throttle_unsat"] for row in trace]
    saturation_fraction = sum(row["saturation_active"] for row in trace) / len(trace)
    saturation_time = sum(row["saturation_active"] for row in trace) * dt
    integrator_abs_max = max((abs(row["speed_integral"]) for row in trace), default=0.0)
    throttle_excess_integral = sum(
        max(0.0, row["throttle_unsat"] - params.throttle_max, params.throttle_min - row["throttle_unsat"])
        for row in trace
    ) * dt

    v_rms = rms(velocity_errors)
    post_v_rms = rms(post_velocity_errors)
    h_rms = rms(altitude_errors)
    q_violation = violation_integral(q_values, params.q_limit, dt)
    delta_max = max(delta_values) if delta_values else 0.0
    delta_rate_max = max(delta_rate_values) if delta_rate_values else 0.0

    tracking_pass = post_v_rms <= 150.0 and h_rms <= 1_500.0
    input_pass = delta_max <= params.delta_limit + 1e-9 and delta_rate_max <= params.delta_rate_limit + 1e-9
    q_pass = q_violation <= 1.0

    return {
        "v_rms": v_rms,
        "post_release_v_rms": post_v_rms,
        "post_release_speed_overshoot": post_overshoot,
        "h_rms": h_rms,
        "q_max": max(q_values) if q_values else 0.0,
        "q_violation_integral": q_violation,
        "delta_max_rad": delta_max,
        "delta_rate_max_rad_s": delta_rate_max,
        "throttle_cmd_max": max((row["throttle_cmd"] for row in trace), default=0.0),
        "throttle_cmd_min": min((row["throttle_cmd"] for row in trace), default=0.0),
        "throttle_unsat_max": max(throttle_unsat_values) if throttle_unsat_values else 0.0,
        "throttle_unsat_min": min(throttle_unsat_values) if throttle_unsat_values else 0.0,
        "throttle_excess_integral": throttle_excess_integral,
        "saturation_fraction": saturation_fraction,
        "saturation_time_s": saturation_time,
        "speed_integral_abs_max": integrator_abs_max,
        "tracking_pass": tracking_pass,
        "input_pass": input_pass,
        "q_pass": q_pass,
        "pass": tracking_pass and input_pass and q_pass,
    }


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["no_anti_windup", "anti_windup"], default="anti_windup")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    trace, metrics = run(args.controller)
    if args.output is not None:
        write_csv(trace, args.output)

    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
