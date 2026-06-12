"""Incremental nonlinear dynamic inversion attitude teaching case.

This module supports case C06 in Volume 5. It uses a single pitch-axis model
to expose the core INDI evidence chain: measured angular acceleration,
filtering, actuator feedback, input delay, and incremental command limits.
It is not a complete three-axis INDI controller or six-DOF IGC implementation.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import rms
from hgv_control.models.attitude import AttitudeParams
from hgv_control.models.longitudinal import clamp


TRACKING_RMS_TARGET = math.radians(3.0)
PEAK_ERROR_TARGET = math.radians(7.5)
RATE_TARGET = math.radians(7.0)
SATURATION_FRACTION_TARGET = 0.20
ACCEL_ESTIMATION_TARGET = math.radians(0.75)


@dataclass(frozen=True)
class PitchIndiPlant:
    inertia_y: float = 52_000.0
    damping_q: float = 1_700.0
    actuator_tau: float = 0.12
    moment_effectiveness: float = 0.68
    command_delay_steps: int = 3


@dataclass(frozen=True)
class IndiControllerConfig:
    theta_gain: float = 1.5
    rate_gain: float = 1.5
    acceleration_filter_alpha: float = 0.25
    inertia_estimate: float = 52_000.0
    effectiveness_estimate: float = 0.70


def pitch_reference(time_s: float) -> float:
    if time_s < 1.0:
        return 0.0
    if time_s < 7.0:
        return math.radians(4.0)
    if time_s < 12.0:
        return math.radians(-3.0)
    return math.radians(2.0)


def disturbance_moment(time_s: float) -> float:
    if 4.5 <= time_s <= 5.4:
        return 1_200.0
    if 10.0 <= time_s <= 10.8:
        return -900.0
    return 0.0


def rate_sensor_noise(time_s: float) -> float:
    return math.radians(0.012) * math.sin(37.0 * time_s) + math.radians(0.006) * math.sin(91.0 * time_s)


def simulate(
    controller: str,
    plant: PitchIndiPlant | None = None,
    params: AttitudeParams | None = None,
    indi_config: IndiControllerConfig | None = None,
    duration: float = 18.0,
    dt: float = 0.02,
) -> dict[str, float | str | bool]:
    if controller not in {"model_pd", "indi"}:
        raise ValueError(f"unknown controller: {controller}")

    plant = plant or PitchIndiPlant()
    params = params or AttitudeParams()
    indi_config = indi_config or IndiControllerConfig()
    theta = math.radians(-1.2)
    pitch_rate = 0.0
    actual_moment = 0.0
    command = 0.0
    previous_command = 0.0
    measured_rate_previous = pitch_rate + rate_sensor_noise(0.0)
    acceleration_estimate = 0.0
    command_delay = [0.0 for _ in range(plant.command_delay_steps)]

    theta_errors: list[float] = []
    post_disturbance_errors: list[float] = []
    rate_values: list[float] = []
    acceleration_estimation_errors: list[float] = []
    command_increments: list[float] = []
    moment_rates: list[float] = []
    command_saturated_steps = 0
    actual_saturated_steps = 0

    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        theta_ref = pitch_reference(time_s)
        theta_error = theta_ref - theta
        measured_rate = pitch_rate + rate_sensor_noise(time_s)
        raw_acceleration = (measured_rate - measured_rate_previous) / dt if step > 0 else 0.0
        acceleration_estimate = (
            (1.0 - indi_config.acceleration_filter_alpha) * acceleration_estimate
            + indi_config.acceleration_filter_alpha * raw_acceleration
        )
        true_acceleration = (
            plant.moment_effectiveness * actual_moment
            + disturbance_moment(time_s)
            - plant.damping_q * pitch_rate
        ) / plant.inertia_y

        if controller == "model_pd":
            raw_command = 70_000.0 * theta_error - 12_000.0 * measured_rate
        else:
            desired_acceleration = indi_config.theta_gain * theta_error - indi_config.rate_gain * measured_rate
            incremental_moment = (
                indi_config.inertia_estimate
                / max(indi_config.effectiveness_estimate, 1e-6)
                * (desired_acceleration - acceleration_estimate)
            )
            raw_command = actual_moment + incremental_moment

        limited_command = clamp(raw_command, -params.moment_limit, params.moment_limit)
        rate_limited_command = clamp(
            limited_command,
            previous_command - params.moment_rate_limit * dt,
            previous_command + params.moment_rate_limit * dt,
        )
        if abs(limited_command) >= params.moment_limit:
            command_saturated_steps += 1
        command = rate_limited_command

        delayed_command = command
        if command_delay:
            delayed_command = command_delay.pop(0)
            command_delay.append(command)

        desired_moment_rate = (delayed_command - actual_moment) / max(plant.actuator_tau, 1e-6)
        moment_rate = clamp(desired_moment_rate, -params.moment_rate_limit, params.moment_rate_limit)
        if abs(actual_moment) >= params.moment_limit:
            actual_saturated_steps += 1

        theta_errors.append(theta_error)
        rate_values.append(abs(pitch_rate))
        acceleration_estimation_errors.append(acceleration_estimate - true_acceleration)
        command_increments.append(abs(command - previous_command))
        moment_rates.append(abs(moment_rate))
        if 5.4 <= time_s <= 7.0 or 10.8 <= time_s <= 12.5:
            post_disturbance_errors.append(theta_error)

        pitch_acceleration = true_acceleration
        theta += dt * pitch_rate
        pitch_rate += dt * pitch_acceleration
        actual_moment = clamp(actual_moment + dt * moment_rate, -params.moment_limit, params.moment_limit)
        measured_rate_previous = measured_rate
        previous_command = command

    attitude_rms = rms(theta_errors)
    attitude_peak = max(abs(error) for error in theta_errors)
    post_disturbance_rms = rms(post_disturbance_errors)
    rate_max = max(rate_values)
    acceleration_error_rms = rms(acceleration_estimation_errors)
    command_increment_rms = rms(command_increments)
    command_increment_max = max(command_increments)
    command_saturation_fraction = command_saturated_steps / len(theta_errors)
    actual_saturation_fraction = actual_saturated_steps / len(theta_errors)
    moment_rate_max = max(moment_rates)
    tracking_pass = attitude_rms <= TRACKING_RMS_TARGET and attitude_peak <= PEAK_ERROR_TARGET
    rate_pass = rate_max <= RATE_TARGET
    input_pass = command_saturation_fraction <= SATURATION_FRACTION_TARGET and moment_rate_max <= params.moment_rate_limit
    acceleration_feedback_pass = acceleration_error_rms <= ACCEL_ESTIMATION_TARGET

    return {
        "controller": controller,
        "theta_gain": 70_000.0 if controller == "model_pd" else indi_config.theta_gain,
        "rate_gain": 12_000.0 if controller == "model_pd" else indi_config.rate_gain,
        "acceleration_filter_alpha": 0.0 if controller == "model_pd" else indi_config.acceleration_filter_alpha,
        "inertia_y": plant.inertia_y,
        "damping_q": plant.damping_q,
        "actuator_tau": plant.actuator_tau,
        "moment_effectiveness": plant.moment_effectiveness,
        "command_delay_steps": plant.command_delay_steps,
        "attitude_rms_rad": attitude_rms,
        "attitude_peak_rad": attitude_peak,
        "post_disturbance_rms_rad": post_disturbance_rms,
        "rate_max_rad_s": rate_max,
        "accel_estimation_error_rms_rad_s2": acceleration_error_rms,
        "command_increment_rms_nm": command_increment_rms,
        "command_increment_max_nm": command_increment_max,
        "command_saturation_fraction": command_saturation_fraction,
        "actual_saturation_fraction": actual_saturation_fraction,
        "moment_rate_max_nm_s": moment_rate_max,
        "tracking_pass": tracking_pass,
        "rate_pass": rate_pass,
        "input_pass": input_pass,
        "acceleration_feedback_pass": acceleration_feedback_pass,
        "case_pass": tracking_pass and rate_pass and input_pass and acceleration_feedback_pass,
    }


def run(controller: str = "all") -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    if controller not in {"all", "model_pd", "indi"}:
        raise ValueError(f"unknown controller: {controller}")

    controllers = ["model_pd", "indi"] if controller == "all" else [controller]
    rows = [simulate(name) for name in controllers]
    summary: dict[str, float | bool] = {}
    if len(rows) == 2:
        baseline = rows[0]
        indi = rows[1]
        summary = {
            "pd_attitude_rms_rad": float(baseline["attitude_rms_rad"]),
            "indi_attitude_rms_rad": float(indi["attitude_rms_rad"]),
            "pd_post_disturbance_rms_rad": float(baseline["post_disturbance_rms_rad"]),
            "indi_post_disturbance_rms_rad": float(indi["post_disturbance_rms_rad"]),
            "pd_saturation_fraction": float(baseline["command_saturation_fraction"]),
            "indi_saturation_fraction": float(indi["command_saturation_fraction"]),
            "pd_case_pass": bool(baseline["case_pass"]),
            "indi_case_pass": bool(indi["case_pass"]),
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
    parser.add_argument("--controller", choices=["all", "model_pd", "indi"], default="all")
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
