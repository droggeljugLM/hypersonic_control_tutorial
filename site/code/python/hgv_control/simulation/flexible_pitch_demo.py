"""Run a low-order rigid-flexible pitch-axis teaching scenario."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path

from hgv_control.metrics.summary import rms
from hgv_control.models.flexible_pitch import (
    FlexiblePitchParams,
    FlexiblePitchState,
    input_effectiveness,
    limit_moment_command,
    make_initial_state,
    measured_pitch_rate,
    modal_energy,
    rk4_step,
)


@dataclass(frozen=True)
class PitchController:
    name: str
    kp: float
    kd: float
    use_measured_rate: bool
    description: str


CONTROLLERS: tuple[PitchController, ...] = (
    PitchController("rigid_rate_pd", 95_000.0, 15_500.0, False, "PD uses true rigid pitch rate"),
    PitchController("flex_sensor_pd", 95_000.0, 15_500.0, True, "PD uses sensor rate with flexible contamination"),
    PitchController("band_limited_pd", 45_000.0, 8_000.0, True, "Lower bandwidth PD uses contaminated rate"),
)


def pitch_reference(time_s: float) -> float:
    if time_s < 1.0:
        return 0.0
    if time_s < 8.0:
        return math.radians(3.0)
    return math.radians(-1.5)


def disturbance_moment(time_s: float) -> float:
    if 5.0 <= time_s <= 5.5:
        return 550.0
    if 11.0 <= time_s <= 11.4:
        return -420.0
    return 0.0


def controller_command(
    controller: PitchController,
    state: FlexiblePitchState,
    theta_ref: float,
    params: FlexiblePitchParams,
) -> float:
    rate_feedback = measured_pitch_rate(state, params) if controller.use_measured_rate else state.q
    raw_command = controller.kp * (theta_ref - state.theta) - controller.kd * rate_feedback
    return limit_moment_command(raw_command, params)


def summarize_trace(
    controller: PitchController,
    trace: list[dict[str, float]],
    params: FlexiblePitchParams,
) -> dict[str, float | str | bool]:
    theta_errors = [row["theta"] - row["theta_ref"] for row in trace]
    measured_rate_errors = [row["q_measured"] - row["q"] for row in trace]
    moment_commands = [abs(row["moment_cmd"]) for row in trace]
    moment_rates = [abs(row["moment_rate"]) for row in trace]
    modal_energy_values = [row["modal_energy"] for row in trace]
    eta_values = [abs(row["eta"]) for row in trace]
    eta_rate_values = [abs(row["eta_dot"]) for row in trace]
    effectiveness_values = [row["effectiveness"] for row in trace]
    saturated_steps = [row for row in trace if abs(row["moment_cmd"]) >= params.moment_limit]
    estimated_bandwidth = math.sqrt(max(controller.kp, 1.0) / params.inertia_y)
    bandwidth_ratio = estimated_bandwidth / params.flexible_frequency_rad_s
    modal_energy_max = max(modal_energy_values) if modal_energy_values else 0.0
    moment_rate_rms = rms(moment_rates)
    tracking_rms = rms(theta_errors)
    sensor_pollution_rms = rms(measured_rate_errors)
    saturation_fraction = len(saturated_steps) / len(trace) if trace else 0.0

    tracking_pass = tracking_rms <= math.radians(3.5)
    modal_pass = modal_energy_max <= 0.020
    actuator_pass = saturation_fraction <= 0.12 and moment_rate_rms <= params.moment_rate_limit
    bandwidth_pass = bandwidth_ratio <= 0.20

    return {
        "controller": controller.name,
        "controller_description": controller.description,
        "kp": controller.kp,
        "kd": controller.kd,
        "theta_rms_rad": tracking_rms,
        "theta_peak_rad": max(abs(error) for error in theta_errors) if theta_errors else 0.0,
        "sensor_pollution_rms_rad_s": sensor_pollution_rms,
        "eta_peak": max(eta_values) if eta_values else 0.0,
        "eta_dot_peak": max(eta_rate_values) if eta_rate_values else 0.0,
        "modal_energy_max": modal_energy_max,
        "moment_cmd_peak_nm": max(moment_commands) if moment_commands else 0.0,
        "moment_rate_rms_nm_s": moment_rate_rms,
        "moment_saturation_fraction": saturation_fraction,
        "effectiveness_min": min(effectiveness_values) if effectiveness_values else 1.0,
        "bandwidth_ratio": bandwidth_ratio,
        "tracking_pass": tracking_pass,
        "modal_pass": modal_pass,
        "actuator_pass": actuator_pass,
        "bandwidth_pass": bandwidth_pass,
        "case_pass": tracking_pass and modal_pass and actuator_pass and bandwidth_pass,
    }


def simulate_controller(
    controller: PitchController,
    duration: float = 16.0,
    dt: float = 0.01,
    params: FlexiblePitchParams | None = None,
    initial_state: FlexiblePitchState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | str | bool]]:
    params = params or FlexiblePitchParams()
    state = initial_state or make_initial_state()
    trace: list[dict[str, float]] = []
    previous_moment = state.moment
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        theta_ref = pitch_reference(time_s)
        command = controller_command(controller, state, theta_ref, params)
        moment_rate = (state.moment - previous_moment) / dt if step > 0 else 0.0
        trace.append(
            {
                "time": time_s,
                "theta": state.theta,
                "theta_ref": theta_ref,
                "q": state.q,
                "q_measured": measured_pitch_rate(state, params),
                "moment": state.moment,
                "moment_cmd": command,
                "moment_rate": moment_rate,
                "eta": state.eta,
                "eta_dot": state.eta_dot,
                "modal_energy": modal_energy(state, params),
                "effectiveness": input_effectiveness(state, params),
                "disturbance_moment": disturbance_moment(time_s),
            }
        )
        previous_moment = state.moment
        state = rk4_step(state, command, dt, params, disturbance_moment(time_s))

    return trace, summarize_trace(controller, trace, params)


def selected_controllers(name: str) -> list[PitchController]:
    if name == "all":
        return list(CONTROLLERS)
    for controller in CONTROLLERS:
        if controller.name == name:
            return [controller]
    raise ValueError(f"unknown controller: {name}")


def run(controller: str = "all") -> tuple[list[dict[str, float]], list[dict[str, float | str | bool]]]:
    traces: list[dict[str, float]] = []
    metrics: list[dict[str, float | str | bool]] = []
    for selected in selected_controllers(controller):
        trace, row = simulate_controller(selected)
        for sample in trace:
            sample["controller"] = selected.name
        traces.extend(trace)
        metrics.append(row)
    return traces, metrics


def sweep_parameter_sets(base: FlexiblePitchParams | None = None) -> list[tuple[str, FlexiblePitchParams]]:
    base = base or FlexiblePitchParams()
    rows: list[tuple[str, FlexiblePitchParams]] = []
    for omega in (6.0, 8.0, 12.0):
        for damping in (0.025, 0.050):
            for sensor_gain in (0.25, 0.45):
                scenario = f"wf{omega:g}_zf{damping:g}_cf{sensor_gain:g}"
                rows.append(
                    (
                        scenario,
                        replace(
                            base,
                            flexible_frequency_rad_s=omega,
                            flexible_damping_ratio=damping,
                            sensor_modal_rate_gain=sensor_gain,
                        ),
                    )
                )
    return rows


def run_sweep(controller: str = "all") -> list[dict[str, float | str | bool]]:
    rows: list[dict[str, float | str | bool]] = []
    for scenario, params in sweep_parameter_sets():
        for selected in selected_controllers(controller):
            _trace, metrics = simulate_controller(selected, params=params)
            metrics["sweep_scenario"] = scenario
            metrics["flexible_frequency_rad_s"] = params.flexible_frequency_rad_s
            metrics["flexible_damping_ratio"] = params.flexible_damping_ratio
            metrics["sensor_modal_rate_gain"] = params.sensor_modal_rate_gain
            metrics["effectiveness_modal_gain"] = params.effectiveness_modal_gain
            metrics["min_effectiveness"] = params.min_effectiveness
            rows.append(metrics)
    return rows


def write_csv(rows: list[dict[str, float | str | bool]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["all"] + [controller.name for controller in CONTROLLERS], default="all")
    parser.add_argument("--trace-output", type=Path, default=None)
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--sweep-output", type=Path, default=None)
    args = parser.parse_args()

    traces, metrics = run(controller=args.controller)
    sweep_rows = run_sweep(controller=args.controller) if args.sweep_output is not None else []
    if args.trace_output is not None and traces:
        write_csv(traces, args.trace_output)
    if args.metrics_output is not None and metrics:
        write_csv(metrics, args.metrics_output)
    if args.sweep_output is not None and sweep_rows:
        write_csv(sweep_rows, args.sweep_output)
    for row in metrics:
        print(
            f"{row['controller']}: pass={row['case_pass']}, "
            f"theta_rms={row['theta_rms_rad']:.6f}, "
            f"modal_energy_max={row['modal_energy_max']:.6f}, "
            f"moment_rate_rms={row['moment_rate_rms_nm_s']:.3f}"
        )
    if sweep_rows:
        failed = sum(1 for row in sweep_rows if not bool(row["case_pass"]))
        print(f"sweep_cases: {len(sweep_rows)}, sweep_failures: {failed}")


if __name__ == "__main__":
    main()
