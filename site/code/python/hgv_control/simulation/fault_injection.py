"""Teaching fault-injection simulations for implementation studies.

The scenarios are deliberately small. They expose sensor bias, actuator
loss, stuck elevator, thrust loss, and input delay effects so the tutorial can
connect estimation/fault chapters with executable metrics.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.controllers.backstepping import BacksteppingController
from hgv_control.controllers.barrier import BarrierController
from hgv_control.controllers.lqr import LqrController
from hgv_control.controllers.ndi import NdiController
from hgv_control.controllers.pid import BaselineController
from hgv_control.controllers.safety_filter import guarded_reference
from hgv_control.controllers.sliding_mode import SlidingModeController
from hgv_control.metrics.summary import rms, summarize, violation_integral
from hgv_control.models.longitudinal import (
    LongitudinalParams,
    State,
    clamp,
    dynamic_pressure,
    rk4_step,
)
from hgv_control.simulation.run_case import make_initial_state, reference


CONTROLLER_NAMES = {
    "baseline",
    "guarded",
    "lqr",
    "guarded_lqr",
    "ndi",
    "guarded_ndi",
    "backstepping",
    "guarded_backstepping",
    "smc",
    "guarded_smc",
    "barrier",
    "guarded_barrier",
}

FAULT_TYPES = {
    "none",
    "altitude_sensor_bias",
    "velocity_sensor_bias",
    "alpha_sensor_bias",
    "elevator_loss",
    "elevator_stuck",
    "throttle_loss",
    "input_delay",
}

GUARDED_CONTROLLERS = {
    "guarded",
    "guarded_lqr",
    "guarded_ndi",
    "guarded_backstepping",
    "guarded_smc",
    "guarded_barrier",
}


@dataclass(frozen=True)
class FaultConfig:
    fault_type: str = "elevator_loss"
    start_time: float = 18.0
    magnitude: float = 0.35
    confirm_steps: int = 3

    def active(self, time_s: float) -> bool:
        return self.fault_type != "none" and time_s >= self.start_time


def default_magnitude(fault_type: str) -> float:
    if fault_type == "altitude_sensor_bias":
        return 350.0
    if fault_type == "velocity_sensor_bias":
        return 45.0
    if fault_type == "alpha_sensor_bias":
        return math.radians(2.0)
    if fault_type == "elevator_stuck":
        return math.radians(4.0)
    if fault_type == "input_delay":
        return 0.12
    if fault_type in {"elevator_loss", "throttle_loss"}:
        return 0.35
    return 0.0


def make_controller(controller_name: str):
    if controller_name in {"lqr", "guarded_lqr"}:
        return LqrController()
    if controller_name in {"ndi", "guarded_ndi"}:
        return NdiController()
    if controller_name in {"backstepping", "guarded_backstepping"}:
        return BacksteppingController()
    if controller_name in {"smc", "guarded_smc"}:
        return SlidingModeController()
    if controller_name in {"barrier", "guarded_barrier"}:
        return BarrierController()
    return BaselineController()


def estimated_state(true_state: State, config: FaultConfig, time_s: float) -> State:
    if not config.active(time_s):
        return State(**true_state.__dict__)

    estimate = State(**true_state.__dict__)
    if config.fault_type == "altitude_sensor_bias":
        estimate.altitude += config.magnitude
    elif config.fault_type == "velocity_sensor_bias":
        estimate.velocity = max(100.0, estimate.velocity + config.magnitude)
    elif config.fault_type == "alpha_sensor_bias":
        estimate.alpha += config.magnitude
        estimate.theta = estimate.alpha + estimate.gamma
    return estimate


def apply_actuator_fault(
    delta_cmd: float,
    throttle_cmd: float,
    config: FaultConfig,
    time_s: float,
    params: LongitudinalParams,
    command_history: list[tuple[float, float]],
    dt: float,
) -> tuple[float, float]:
    if not config.active(time_s):
        return delta_cmd, throttle_cmd

    delta_actual = delta_cmd
    throttle_actual = throttle_cmd

    if config.fault_type == "elevator_loss":
        efficiency = clamp(1.0 - config.magnitude, 0.0, 1.0)
        delta_actual = efficiency * delta_cmd
    elif config.fault_type == "elevator_stuck":
        delta_actual = clamp(config.magnitude, -params.delta_limit, params.delta_limit)
    elif config.fault_type == "throttle_loss":
        efficiency = clamp(1.0 - config.magnitude, 0.0, 1.0)
        throttle_actual = params.throttle_min + efficiency * (throttle_cmd - params.throttle_min)
    elif config.fault_type == "input_delay":
        delay_steps = max(1, int(round(config.magnitude / dt)))
        if len(command_history) > delay_steps:
            delta_actual, throttle_actual = command_history[-delay_steps - 1]

    return (
        clamp(delta_actual, -params.delta_limit, params.delta_limit),
        clamp(throttle_actual, params.throttle_min, params.throttle_max),
    )


def normalized_fault_residual(
    true_state: State,
    estimate: State,
    delta_cmd: float,
    delta_actual: float,
    throttle_cmd: float,
    throttle_actual: float,
    params: LongitudinalParams,
) -> float:
    qbar = dynamic_pressure(true_state, params)
    qbar_est = dynamic_pressure(estimate, params)
    return max(
        abs(estimate.altitude - true_state.altitude) / 200.0,
        abs(estimate.velocity - true_state.velocity) / 30.0,
        abs(estimate.alpha - true_state.alpha) / math.radians(0.5),
        abs(qbar_est - qbar) / 1200.0,
        abs(delta_cmd - delta_actual) / math.radians(0.75),
        abs(throttle_cmd - throttle_actual) / 0.05,
    )


def fault_metrics(
    trace: list[dict[str, float]],
    config: FaultConfig,
    q_limit: float,
    dt: float,
) -> dict[str, float | bool]:
    post_fault = [row for row in trace if row["time"] >= config.start_time]
    detections = [row["time"] for row in trace if row["fault_detected"] >= 0.5]
    false_alarms = [row for row in trace if row["time"] < config.start_time and row["fault_detected"] >= 0.5]
    first_detection = detections[0] if detections else math.nan
    active_fault = config.fault_type != "none"
    missed_fault = bool(active_fault and not detections)

    altitude_est_errors = [row["altitude_est"] - row["altitude"] for row in trace]
    velocity_est_errors = [row["velocity_est"] - row["velocity"] for row in trace]
    alpha_est_errors = [row["alpha_est"] - row["alpha"] for row in trace]
    q_est_errors = [row["qbar_est"] - row["qbar"] for row in trace]

    post_h_errors = [row["altitude"] - row["altitude_ref"] for row in post_fault]
    post_v_errors = [row["velocity"] - row["velocity_ref"] for row in post_fault]
    post_q_values = [row["qbar"] for row in post_fault]
    delay = first_detection - config.start_time if detections else math.inf

    return {
        "fault_active_time": sum(row["fault_active"] for row in trace) * dt,
        "fault_detection_delay": delay,
        "fault_detected": bool(detections),
        "false_alarm_count": float(len(false_alarms)),
        "missed_fault_count": float(1 if missed_fault else 0),
        "fault_residual_max": max((row["fault_residual"] for row in trace), default=0.0),
        "altitude_est_rms": rms(altitude_est_errors),
        "velocity_est_rms": rms(velocity_est_errors),
        "alpha_est_rms": rms(alpha_est_errors),
        "q_est_error_max": max((abs(value) for value in q_est_errors), default=0.0),
        "q_est_margin_min": min((q_limit - row["qbar_est"] for row in trace), default=0.0),
        "post_fault_h_rms": rms(post_h_errors),
        "post_fault_v_rms": rms(post_v_errors),
        "post_fault_q_violation_integral": violation_integral(post_q_values, q_limit, dt),
    }


def run(
    controller_name: str = "guarded",
    config: FaultConfig | None = None,
    duration: float = 45.0,
    dt: float = 0.02,
    params: LongitudinalParams | None = None,
    initial_state: State | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or LongitudinalParams()
    config = config or FaultConfig()
    controller = make_controller(controller_name)
    state = initial_state or make_initial_state()
    command_history: list[tuple[float, float]] = []
    residual_streak = 0
    detected_latched = False
    trace: list[dict[str, float]] = []

    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        state_est = estimated_state(state, config, time_s)
        altitude_ref, velocity_ref = reference(time_s)
        if controller_name in GUARDED_CONTROLLERS:
            altitude_cmd, velocity_cmd = guarded_reference(state_est, altitude_ref, velocity_ref, params)
        else:
            altitude_cmd, velocity_cmd = altitude_ref, velocity_ref

        delta_cmd, throttle_cmd = controller.command(state_est, altitude_cmd, velocity_cmd, params)
        delta_actual, throttle_actual = apply_actuator_fault(
            delta_cmd,
            throttle_cmd,
            config,
            time_s,
            params,
            command_history,
            dt,
        )
        command_history.append((delta_cmd, throttle_cmd))

        residual = normalized_fault_residual(
            state,
            state_est,
            delta_cmd,
            delta_actual,
            throttle_cmd,
            throttle_actual,
            params,
        )
        residual_streak = residual_streak + 1 if residual >= 1.0 else 0
        if residual_streak >= config.confirm_steps:
            detected_latched = True

        qbar = dynamic_pressure(state, params)
        qbar_est = dynamic_pressure(state_est, params)
        trace.append(
            {
                "time": time_s,
                "velocity": state.velocity,
                "velocity_est": state_est.velocity,
                "velocity_ref": velocity_ref,
                "altitude": state.altitude,
                "altitude_est": state_est.altitude,
                "altitude_ref": altitude_ref,
                "gamma": state.gamma,
                "alpha": state.alpha,
                "alpha_est": state_est.alpha,
                "theta": state.theta,
                "pitch_rate": state.pitch_rate,
                "elevator": state.elevator,
                "elevator_rate": state.elevator_rate,
                "delta_cmd": delta_cmd,
                "delta_actual": delta_actual,
                "throttle_cmd": throttle_cmd,
                "throttle_actual": throttle_actual,
                "qbar": qbar,
                "qbar_est": qbar_est,
                "fault_active": 1.0 if config.active(time_s) else 0.0,
                "fault_detected": 1.0 if detected_latched else 0.0,
                "fault_residual": residual,
            }
        )
        state = rk4_step(state, delta_actual, throttle_actual, dt, params)

    metrics = summarize(
        trace,
        params.q_limit,
        dt,
        alpha_limit=math.radians(12.0),
        delta_limit=params.delta_limit,
        delta_rate_limit=params.delta_rate_limit,
    )
    metrics.update(fault_metrics(trace, config, params.q_limit, dt))
    return trace, metrics


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=sorted(CONTROLLER_NAMES), default="guarded")
    parser.add_argument("--fault", choices=sorted(FAULT_TYPES), default="elevator_loss")
    parser.add_argument("--start-time", type=float, default=18.0)
    parser.add_argument("--magnitude", type=float, default=None)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    magnitude = default_magnitude(args.fault) if args.magnitude is None else args.magnitude
    trace, metrics = run(
        args.controller,
        FaultConfig(fault_type=args.fault, start_time=args.start_time, magnitude=magnitude),
        duration=args.duration,
        dt=args.dt,
    )
    if args.output is not None:
        write_csv(trace, args.output)

    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
