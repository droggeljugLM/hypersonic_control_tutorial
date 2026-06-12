"""Run a teaching aerodynamic-table interpolation scenario."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.models.aero_table import AeroTableParams, evaluate_aero
from hgv_control.models.control_allocation import AllocatorState
from hgv_control.models.point_mass_3d import PointMass3DParams, PointMass3DState, dynamic_pressure_3d


def sample_conditions(time_s: float, aero_params: AeroTableParams) -> tuple[float, float, float, float, float, AllocatorState]:
    mach = 7.5 + 1.8 * math.sin(0.16 * time_s)
    altitude = 29_000.0 + 1_400.0 * math.sin(0.11 * time_s + 0.3)
    velocity = mach * aero_params.speed_of_sound_m_s
    alpha = math.radians(4.0 + 6.0 * math.sin(0.24 * time_s))
    beta = math.radians(3.0 * math.sin(0.19 * time_s + 0.5))
    control = AllocatorState(
        left_elevon=math.radians(3.0 * math.sin(0.31 * time_s)),
        right_elevon=math.radians(-2.5 * math.sin(0.31 * time_s + 0.2)),
        rudder=math.radians(2.0 * math.sin(0.27 * time_s - 0.4)),
        body_flap=math.radians(2.5 * math.sin(0.21 * time_s + 0.7)),
    )
    return mach, altitude, velocity, alpha, beta, control


def summarize_aero(trace: list[dict[str, float]], params: AeroTableParams) -> dict[str, float | bool]:
    qbar = 55_000.0
    no_control = AllocatorState()
    alpha_low = evaluate_aero(7.0, qbar, math.radians(2.0), 0.0, no_control, params)
    alpha_high = evaluate_aero(7.0, qbar, math.radians(10.0), 0.0, no_control, params)
    body_flap_down = evaluate_aero(7.0, qbar, math.radians(4.0), 0.0, AllocatorState(body_flap=math.radians(6.0)), params)
    beta_case = evaluate_aero(7.0, qbar, math.radians(4.0), math.radians(5.0), no_control, params)
    rudder_case = evaluate_aero(7.0, qbar, math.radians(4.0), 0.0, AllocatorState(rudder=math.radians(5.0)), params)

    alpha_delta = math.radians(8.0)
    clift_alpha_slope = (alpha_high.coefficients.clift - alpha_low.coefficients.clift) / alpha_delta
    pitch_body_flap_delta = body_flap_down.coefficients.cm_pitch - evaluate_aero(
        7.0, qbar, math.radians(4.0), 0.0, no_control, params
    ).coefficients.cm_pitch
    side_force_beta_delta = beta_case.coefficients.cy
    yaw_rudder_delta = rudder_case.coefficients.cn_yaw
    drag_values = [row["cdrag"] for row in trace]
    lift_to_drag_values = [row["lift_to_drag"] for row in trace if row["drag"] > 1e-9]
    moment_values = [
        abs(value)
        for row in trace
        for value in (row["mx"], row["my"], row["mz"])
    ]
    lift_slope_pass = clift_alpha_slope > 1.0
    drag_pass = min(drag_values) > 0.0
    pitch_control_pass = pitch_body_flap_delta < 0.0
    sideslip_pass = abs(side_force_beta_delta) > 1e-3
    rudder_pass = yaw_rudder_delta > 0.0
    return {
        "clift_alpha_slope_per_rad": clift_alpha_slope,
        "drag_min": min(drag_values),
        "lift_to_drag_min": min(lift_to_drag_values) if lift_to_drag_values else 0.0,
        "moment_max_nm": max(moment_values) if moment_values else 0.0,
        "pitch_body_flap_delta": pitch_body_flap_delta,
        "side_force_beta_delta": side_force_beta_delta,
        "yaw_rudder_delta": yaw_rudder_delta,
        "lift_slope_pass": lift_slope_pass,
        "drag_pass": drag_pass,
        "pitch_control_pass": pitch_control_pass,
        "sideslip_pass": sideslip_pass,
        "rudder_pass": rudder_pass,
        "pass": lift_slope_pass and drag_pass and pitch_control_pass and sideslip_pass and rudder_pass,
    }


def run(
    duration: float = 24.0,
    dt: float = 0.05,
    aero_params: AeroTableParams | None = None,
    point_params: PointMass3DParams | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    aero_params = aero_params or AeroTableParams()
    point_params = point_params or PointMass3DParams()
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        mach, altitude, velocity, alpha, beta, control = sample_conditions(time_s, aero_params)
        point_state = PointMass3DState(0.0, 0.0, altitude, velocity, 0.0, 0.0)
        qbar = dynamic_pressure_3d(point_state, point_params)
        evaluation = evaluate_aero(mach, qbar, alpha, beta, control, aero_params)
        coefficients = evaluation.coefficients
        loads = evaluation.loads
        trace.append(
            {
                "time": time_s,
                "mach": mach,
                "altitude": altitude,
                "velocity": velocity,
                "qbar": qbar,
                "alpha_rad": evaluation.alpha,
                "beta_rad": evaluation.beta,
                "left_elevon": control.left_elevon,
                "right_elevon": control.right_elevon,
                "rudder": control.rudder,
                "body_flap": control.body_flap,
                "clift": coefficients.clift,
                "cdrag": coefficients.cdrag,
                "cy": coefficients.cy,
                "cl_roll": coefficients.cl_roll,
                "cm_pitch": coefficients.cm_pitch,
                "cn_yaw": coefficients.cn_yaw,
                "lift": loads.lift,
                "drag": loads.drag,
                "side_force": loads.side_force,
                "mx": loads.mx,
                "my": loads.my,
                "mz": loads.mz,
                "lift_to_drag": loads.lift / max(loads.drag, 1e-9),
            }
        )
    return trace, summarize_aero(trace, aero_params)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=24.0)
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
