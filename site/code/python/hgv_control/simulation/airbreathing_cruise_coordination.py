"""Teaching airbreathing cruise coordination case.

This module supports case C04 in Volume 5. It extends the reduced
longitudinal teaching model with throttle dynamics, throttle-rate limits,
an inlet-margin proxy, and guard logic that coordinates speed, altitude,
angle-of-attack, and propulsion protection.

The implementation is intentionally lightweight. It is a teaching case for
control-evidence fields, not a high-fidelity scramjet or inlet model.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import rms, violation_integral
from hgv_control.models.longitudinal import LongitudinalParams, State, aero_forces, clamp, dynamic_pressure


@dataclass(frozen=True)
class CruiseCoordinationConfig:
    dt: float = 0.04
    duration: float = 32.0
    throttle_time_constant: float = 0.75
    throttle_rate_limit: float = 0.18
    alpha_limit: float = math.radians(10.5)
    alpha_guard_soft: float = math.radians(6.5)
    q_limit: float = 41_000.0
    q_guard_soft: float = 40_000.0
    heat_proxy_limit: float = 2.6e5
    heat_guard_soft: float = 2.1e5
    inlet_guard_threshold: float = 0.28
    inlet_margin_limit: float = 0.08
    throttle_guard_soft: float = 0.68
    throttle_guard_cap: float = 0.80
    velocity_relief_max: float = 135.0
    altitude_relief_max: float = 350.0
    alpha_guard_floor: float = math.radians(7.0)
    throttle_guard_floor: float = 0.72


@dataclass
class CruiseState:
    velocity: float
    altitude: float
    gamma: float
    alpha: float
    pitch_rate: float
    theta: float
    elevator: float
    elevator_rate: float
    throttle_actual: float


@dataclass(frozen=True)
class CruiseCommand:
    delta_cmd: float
    throttle_cmd: float
    alpha_cmd: float
    velocity_ref_modified: float
    altitude_ref_modified: float
    inlet_margin_pred: float
    inlet_margin_active: bool
    propulsion_limit_active: bool


def make_initial_state() -> CruiseState:
    return CruiseState(
        velocity=1_690.0,
        altitude=29_200.0,
        gamma=math.radians(-0.8),
        alpha=math.radians(3.5),
        pitch_rate=0.0,
        theta=math.radians(2.7),
        elevator=0.0,
        elevator_rate=0.0,
        throttle_actual=0.48,
    )


def reference(time_s: float) -> tuple[float, float]:
    if time_s < 10.0:
        return 30_000.0, 1_760.0
    if time_s < 26.0:
        return 31_100.0, 1_875.0
    return 30_450.0, 1_810.0


def to_longitudinal_state(state: CruiseState) -> State:
    return State(
        velocity=state.velocity,
        altitude=state.altitude,
        gamma=state.gamma,
        alpha=state.alpha,
        pitch_rate=state.pitch_rate,
        theta=state.theta,
        elevator=state.elevator,
        elevator_rate=state.elevator_rate,
    )


def heat_proxy(state: CruiseState, params: LongitudinalParams) -> float:
    rho = 2.0 * dynamic_pressure(to_longitudinal_state(state), params) / max(state.velocity * state.velocity, 1.0)
    return 7.2e-5 * math.sqrt(max(rho, 0.0)) * state.velocity**3


def estimate_inlet_margin(
    qbar: float,
    alpha_eval: float,
    throttle_eval: float,
    throttle_rate_eval: float,
    heat_eval: float,
    params: LongitudinalParams,
    config: CruiseCoordinationConfig,
) -> float:
    alpha_ratio = max(0.0, (abs(alpha_eval) - config.alpha_guard_soft) / max(config.alpha_limit - config.alpha_guard_soft, 1e-9))
    q_ratio = max(0.0, (qbar - config.q_guard_soft) / max(config.q_limit - config.q_guard_soft, 1e-9))
    throttle_ratio = max(
        0.0,
        (throttle_eval - config.throttle_guard_soft) / max(params.throttle_max - config.throttle_guard_soft, 1e-9),
    )
    throttle_rate_ratio = abs(throttle_rate_eval) / max(config.throttle_rate_limit, 1e-9)
    heat_ratio = max(
        0.0,
        (heat_eval - config.heat_guard_soft) / max(config.heat_proxy_limit - config.heat_guard_soft, 1e-9),
    )
    return 0.34 - 0.40 * alpha_ratio - 0.30 * throttle_ratio - 0.55 * q_ratio - 0.08 * throttle_rate_ratio - 0.05 * heat_ratio


@dataclass
class CruiseCoordinationController:
    guarded: bool
    altitude_gain: float = 1.5e-4
    gamma_gain: float = 1.25
    pitch_gain: float = 0.92
    q_gain: float = 0.52
    speed_gain: float = 1.45e-3
    integral_gain: float = 7.0e-5
    trim_throttle: float = 0.50
    alpha_trim: float = math.radians(3.4)
    speed_integral: float = 0.0

    def _baseline_channel(
        self,
        state: CruiseState,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float, float, float]:
        gamma_cmd = clamp(self.altitude_gain * (altitude_ref - state.altitude), math.radians(-6.5), math.radians(6.5))
        alpha_cmd = self.alpha_trim + clamp(
            self.gamma_gain * (gamma_cmd - state.gamma),
            math.radians(-3.0),
            math.radians(7.0),
        )
        speed_error = velocity_ref - state.velocity
        throttle_unsat = self.trim_throttle + self.speed_gain * speed_error + self.integral_gain * self.speed_integral
        return gamma_cmd, alpha_cmd, throttle_unsat, speed_error

    def _elevator_command(self, state: CruiseState, alpha_cmd: float, params: LongitudinalParams) -> float:
        theta_cmd = alpha_cmd + state.gamma
        delta_cmd = -self.pitch_gain * (theta_cmd - state.theta) + self.q_gain * state.pitch_rate
        return clamp(delta_cmd, -params.delta_limit, params.delta_limit)

    def command(
        self,
        state: CruiseState,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
        config: CruiseCoordinationConfig,
    ) -> CruiseCommand:
        qbar = dynamic_pressure(to_longitudinal_state(state), params)
        thermal = heat_proxy(state, params)
        velocity_ref_modified = velocity_ref
        altitude_ref_modified = altitude_ref

        _gamma_cmd, alpha_cmd, throttle_unsat, speed_error = self._baseline_channel(
            state,
            altitude_ref_modified,
            velocity_ref_modified,
            params,
        )
        throttle_rate_request = (throttle_unsat - state.throttle_actual) / config.dt
        inlet_margin_pred = estimate_inlet_margin(
            qbar,
            abs(state.alpha),
            max(state.throttle_actual, throttle_unsat),
            throttle_rate_request,
            thermal,
            params,
            config,
        )

        severity = 0.0
        inlet_margin_active = False
        propulsion_limit_active = False
        throttle_cap = params.throttle_max
        rate_cap = config.throttle_rate_limit
        if self.guarded and inlet_margin_pred < config.inlet_guard_threshold:
            severity = clamp(
                (config.inlet_guard_threshold - inlet_margin_pred) / max(config.inlet_guard_threshold, 1e-9),
                0.0,
                1.0,
            )
            inlet_margin_active = severity > 1e-9
            velocity_ref_modified = velocity_ref - config.velocity_relief_max * severity
            altitude_ref_modified = altitude_ref + config.altitude_relief_max * severity
            _gamma_cmd, alpha_cmd, throttle_unsat, speed_error = self._baseline_channel(
                state,
                altitude_ref_modified,
                velocity_ref_modified,
                params,
            )
            alpha_bias = math.radians(1.2) * severity
            alpha_cmd = clamp(alpha_cmd + alpha_bias, config.alpha_guard_floor, config.alpha_limit)
            throttle_cap = config.throttle_guard_cap - (config.throttle_guard_cap - config.throttle_guard_floor) * severity
            rate_cap = config.throttle_rate_limit * (1.0 - 0.15 * severity)

        delta_cmd = self._elevator_command(state, alpha_cmd, params)
        throttle_cmd = clamp(throttle_unsat, params.throttle_min, throttle_cap)
        throttle_rate_limited = clamp(
            (throttle_cmd - state.throttle_actual) / config.dt,
            -rate_cap,
            rate_cap,
        )
        throttle_cmd = clamp(
            state.throttle_actual + throttle_rate_limited * config.dt,
            params.throttle_min,
            throttle_cap,
        )
        propulsion_limit_active = propulsion_limit_active or abs(throttle_cmd - throttle_unsat) > 1e-9 or throttle_cap < params.throttle_max - 1e-9

        saturated_high = throttle_unsat > throttle_cap
        saturated_low = throttle_unsat < params.throttle_min
        if self.guarded:
            drives_deeper = (saturated_high and speed_error > 0.0) or (saturated_low and speed_error < 0.0)
            if not drives_deeper:
                self.speed_integral += speed_error * config.dt
        else:
            self.speed_integral += speed_error * config.dt

        inlet_margin_pred = estimate_inlet_margin(
            qbar,
            abs(state.alpha),
            max(state.throttle_actual, throttle_cmd),
            throttle_rate_limited,
            thermal,
            params,
            config,
        )
        return CruiseCommand(
            delta_cmd=delta_cmd,
            throttle_cmd=throttle_cmd,
            alpha_cmd=alpha_cmd,
            velocity_ref_modified=velocity_ref_modified,
            altitude_ref_modified=altitude_ref_modified,
            inlet_margin_pred=inlet_margin_pred,
            inlet_margin_active=inlet_margin_active,
            propulsion_limit_active=propulsion_limit_active,
        )


def derivatives(
    state: CruiseState,
    command: CruiseCommand,
    params: LongitudinalParams,
    config: CruiseCoordinationConfig,
) -> CruiseState:
    base_state = to_longitudinal_state(state)
    lift, drag, moment, thrust = aero_forces(base_state, state.throttle_actual, params)
    velocity = max(100.0, state.velocity)

    delta_accel = (
        params.actuator_wn * params.actuator_wn * (command.delta_cmd - state.elevator)
        - 2.0 * params.actuator_zeta * params.actuator_wn * state.elevator_rate
    )
    if abs(state.elevator_rate) >= params.delta_rate_limit and state.elevator_rate * delta_accel > 0.0:
        delta_accel = 0.0

    throttle_dot = (command.throttle_cmd - state.throttle_actual) / max(config.throttle_time_constant, 1e-6)
    throttle_dot = clamp(throttle_dot, -config.throttle_rate_limit, config.throttle_rate_limit)

    v_dot = (thrust * math.cos(state.alpha) - drag) / params.mass - params.gravity * math.sin(state.gamma)
    gamma_dot = (
        (thrust * math.sin(state.alpha) + lift) / (params.mass * velocity)
        - params.gravity * math.cos(state.gamma) / velocity
    )
    h_dot = state.velocity * math.sin(state.gamma)
    q_dot = moment / params.iy
    theta_dot = state.pitch_rate
    alpha_dot = theta_dot - gamma_dot

    return CruiseState(
        velocity=v_dot,
        altitude=h_dot,
        gamma=gamma_dot,
        alpha=alpha_dot,
        pitch_rate=q_dot,
        theta=theta_dot,
        elevator=state.elevator_rate,
        elevator_rate=delta_accel,
        throttle_actual=throttle_dot,
    )


def add_state(state: CruiseState, slope: CruiseState, scale: float, params: LongitudinalParams) -> CruiseState:
    return CruiseState(
        velocity=max(100.0, state.velocity + scale * slope.velocity),
        altitude=max(0.0, state.altitude + scale * slope.altitude),
        gamma=state.gamma + scale * slope.gamma,
        alpha=clamp(state.alpha + scale * slope.alpha, math.radians(-8.0), math.radians(14.0)),
        pitch_rate=state.pitch_rate + scale * slope.pitch_rate,
        theta=state.theta + scale * slope.theta,
        elevator=clamp(state.elevator + scale * slope.elevator, -params.delta_limit, params.delta_limit),
        elevator_rate=clamp(
            state.elevator_rate + scale * slope.elevator_rate,
            -params.delta_rate_limit,
            params.delta_rate_limit,
        ),
        throttle_actual=clamp(state.throttle_actual + scale * slope.throttle_actual, params.throttle_min, params.throttle_max),
    )


def rk4_step(
    state: CruiseState,
    command: CruiseCommand,
    params: LongitudinalParams,
    config: CruiseCoordinationConfig,
) -> CruiseState:
    dt = config.dt
    k1 = derivatives(state, command, params, config)
    k2 = derivatives(add_state(state, k1, 0.5 * dt, params), command, params, config)
    k3 = derivatives(add_state(state, k2, 0.5 * dt, params), command, params, config)
    k4 = derivatives(add_state(state, k3, dt, params), command, params, config)
    next_state = CruiseState(
        velocity=state.velocity + dt * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) / 6.0,
        altitude=state.altitude + dt * (k1.altitude + 2.0 * k2.altitude + 2.0 * k3.altitude + k4.altitude) / 6.0,
        gamma=state.gamma + dt * (k1.gamma + 2.0 * k2.gamma + 2.0 * k3.gamma + k4.gamma) / 6.0,
        alpha=state.alpha + dt * (k1.alpha + 2.0 * k2.alpha + 2.0 * k3.alpha + k4.alpha) / 6.0,
        pitch_rate=state.pitch_rate + dt * (k1.pitch_rate + 2.0 * k2.pitch_rate + 2.0 * k3.pitch_rate + k4.pitch_rate) / 6.0,
        theta=state.theta + dt * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta) / 6.0,
        elevator=state.elevator + dt * (k1.elevator + 2.0 * k2.elevator + 2.0 * k3.elevator + k4.elevator) / 6.0,
        elevator_rate=state.elevator_rate
        + dt * (k1.elevator_rate + 2.0 * k2.elevator_rate + 2.0 * k3.elevator_rate + k4.elevator_rate) / 6.0,
        throttle_actual=state.throttle_actual
        + dt * (k1.throttle_actual + 2.0 * k2.throttle_actual + 2.0 * k3.throttle_actual + k4.throttle_actual) / 6.0,
    )
    return CruiseState(
        velocity=max(100.0, next_state.velocity),
        altitude=max(0.0, next_state.altitude),
        gamma=clamp(next_state.gamma, math.radians(-8.0), math.radians(6.0)),
        alpha=clamp(next_state.alpha, math.radians(-8.0), math.radians(14.0)),
        pitch_rate=next_state.pitch_rate,
        theta=next_state.theta,
        elevator=clamp(next_state.elevator, -params.delta_limit, params.delta_limit),
        elevator_rate=clamp(next_state.elevator_rate, -params.delta_rate_limit, params.delta_rate_limit),
        throttle_actual=clamp(next_state.throttle_actual, params.throttle_min, params.throttle_max),
    )


def simulate(
    controller: str,
    params: LongitudinalParams | None = None,
    config: CruiseCoordinationConfig | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | str | bool]]:
    if controller not in {"baseline", "cruise_guard"}:
        raise ValueError(f"unknown controller: {controller}")

    config = config or CruiseCoordinationConfig()
    params = params or LongitudinalParams(thrust_max=23_000.0, throttle_max=0.92, q_limit=config.q_limit)
    state = make_initial_state()
    coordinator = CruiseCoordinationController(guarded=controller == "cruise_guard")

    trace: list[dict[str, float]] = []
    previous_throttle_actual = state.throttle_actual
    steps = int(config.duration / config.dt)
    for step in range(steps + 1):
        time_s = step * config.dt
        altitude_ref, velocity_ref = reference(time_s)
        command = coordinator.command(state, altitude_ref, velocity_ref, params, config)
        qbar = dynamic_pressure(to_longitudinal_state(state), params)
        thermal = heat_proxy(state, params)
        throttle_rate = (state.throttle_actual - previous_throttle_actual) / config.dt if step > 0 else 0.0
        previous_throttle_actual = state.throttle_actual
        trace.append(
            {
                "time": time_s,
                "controller": 0.0 if controller == "baseline" else 1.0,
                "velocity": state.velocity,
                "velocity_ref": velocity_ref,
                "velocity_ref_modified": command.velocity_ref_modified,
                "altitude": state.altitude,
                "altitude_ref": altitude_ref,
                "altitude_ref_modified": command.altitude_ref_modified,
                "gamma": state.gamma,
                "alpha": state.alpha,
                "theta": state.theta,
                "pitch_rate": state.pitch_rate,
                "elevator": state.elevator,
                "elevator_rate": state.elevator_rate,
                "delta_cmd": command.delta_cmd,
                "alpha_cmd": command.alpha_cmd,
                "throttle_cmd": command.throttle_cmd,
                "throttle_actual": state.throttle_actual,
                "throttle_rate": throttle_rate,
                "qbar": qbar,
                "heat_proxy": thermal,
                "inlet_margin": command.inlet_margin_pred,
                "inlet_margin_active": 1.0 if command.inlet_margin_active else 0.0,
                "propulsion_limit_active": 1.0 if command.propulsion_limit_active else 0.0,
                "alpha_limit_margin": config.alpha_limit - abs(state.alpha),
            }
        )
        state = rk4_step(state, command, params, config)

    return trace, summarize(trace, controller, params, config)


def summarize(
    trace: list[dict[str, float]],
    controller: str,
    params: LongitudinalParams,
    config: CruiseCoordinationConfig,
) -> dict[str, float | str | bool]:
    velocity_errors = [row["velocity"] - row["velocity_ref_modified"] for row in trace]
    altitude_errors = [row["altitude"] - row["altitude_ref_modified"] for row in trace]
    mission_velocity_errors = [row["velocity"] - row["velocity_ref"] for row in trace]
    mission_altitude_errors = [row["altitude"] - row["altitude_ref"] for row in trace]
    q_values = [row["qbar"] for row in trace]
    heat_values = [row["heat_proxy"] for row in trace]
    inlet_margins = [row["inlet_margin"] for row in trace]
    alpha_values = [abs(row["alpha"]) for row in trace]
    throttle_actual_values = [row["throttle_actual"] for row in trace]
    throttle_rate_values = [abs(row["throttle_rate"]) for row in trace]
    delta_values = [abs(row["elevator"]) for row in trace]
    delta_rate_values = [abs(row["elevator_rate"]) for row in trace]
    alpha_limit_margins = [row["alpha_limit_margin"] for row in trace]
    inlet_margin_active_time = sum(row["inlet_margin_active"] for row in trace) * config.dt
    propulsion_limit_active_time = sum(row["propulsion_limit_active"] for row in trace) * config.dt

    h_rms = rms(altitude_errors)
    v_rms = rms(velocity_errors)
    mission_h_rms = rms(mission_altitude_errors)
    mission_v_rms = rms(mission_velocity_errors)
    q_violation = violation_integral(q_values, config.q_limit, config.dt)
    q_max = max(q_values) if q_values else 0.0
    inlet_margin_min = min(inlet_margins) if inlet_margins else 0.0
    throttle_peak = max(throttle_actual_values) if throttle_actual_values else 0.0
    throttle_rate_peak = max(throttle_rate_values) if throttle_rate_values else 0.0
    alpha_max = max(alpha_values) if alpha_values else 0.0
    heat_proxy_max = max(heat_values) if heat_values else 0.0
    tracking_pass = h_rms <= 2_300.0 and v_rms <= 120.0
    inlet_pass = inlet_margin_min >= config.inlet_margin_limit
    input_pass = (
        alpha_max <= config.alpha_limit + 1e-12
        and throttle_rate_peak <= config.throttle_rate_limit + 1e-12
        and max(delta_values, default=0.0) <= params.delta_limit + 1e-12
        and max(delta_rate_values, default=0.0) <= params.delta_rate_limit + 1e-12
    )
    q_pass = q_violation <= 1.0
    thermal_pass = heat_proxy_max <= config.heat_proxy_limit + 1e-9

    return {
        "controller": controller,
        "model": "teaching_airbreathing_longitudinal",
        "h_rms": h_rms,
        "v_rms": v_rms,
        "mission_h_rms": mission_h_rms,
        "mission_v_rms": mission_v_rms,
        "inlet_margin_min": inlet_margin_min,
        "inlet_margin_active_time": inlet_margin_active_time,
        "propulsion_limit_active_time": propulsion_limit_active_time,
        "throttle_peak": throttle_peak,
        "throttle_rate_peak": throttle_rate_peak,
        "alpha_max": alpha_max,
        "alpha_limit_margin_min": min(alpha_limit_margins) if alpha_limit_margins else 0.0,
        "q_max": q_max,
        "q_violation_integral": q_violation,
        "heat_proxy_max": heat_proxy_max,
        "tracking_pass": tracking_pass,
        "inlet_pass": inlet_pass,
        "input_pass": input_pass,
        "q_pass": q_pass,
        "thermal_pass": thermal_pass,
        "case_pass": tracking_pass and inlet_pass and input_pass and q_pass and thermal_pass,
    }


def run(controller: str = "all") -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    if controller not in {"all", "baseline", "cruise_guard"}:
        raise ValueError(f"unknown controller: {controller}")
    controllers = ["baseline", "cruise_guard"] if controller == "all" else [controller]
    rows = [simulate(name)[1] for name in controllers]
    summary: dict[str, float | bool] = {}
    if len(rows) == 2:
        baseline, guarded = rows
        summary = {
            "baseline_inlet_margin_min": float(baseline["inlet_margin_min"]),
            "guarded_inlet_margin_min": float(guarded["inlet_margin_min"]),
            "baseline_q_violation_integral": float(baseline["q_violation_integral"]),
            "guarded_q_violation_integral": float(guarded["q_violation_integral"]),
            "baseline_v_rms": float(baseline["v_rms"]),
            "guarded_v_rms": float(guarded["v_rms"]),
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
    parser.add_argument("--controller", choices=["all", "baseline", "cruise_guard"], default="all")
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--trace-output", type=Path, default=None)
    args = parser.parse_args()

    rows, summary = run(args.controller)
    if args.metrics_output is not None:
        write_metrics_csv(rows, args.metrics_output)
    if args.trace_output is not None:
        trace_controller = "cruise_guard" if args.controller == "all" else args.controller
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
