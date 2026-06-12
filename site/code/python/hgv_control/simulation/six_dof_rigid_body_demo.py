"""Run a standalone teaching six-DOF rigid-body propagation demo.

This is not a closed-loop IGC example. It only checks that body-frame forces and
moments can propagate ENU position, ENU velocity, quaternion attitude, and body
rates through a minimal rigid-body plant.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.metrics.summary import rms
from hgv_control.models.rigid_body_6dof import (
    BodyForceMoment,
    RigidBody6DofParams,
    RigidBody6DofState,
    body_rate_norm,
    euler_step,
    force_acceleration_enu,
    kinematics_health,
    limit_force_moment,
    make_level_initial_state,
    speed,
    state_is_finite,
)


def command_profile(time_s: float, params: RigidBody6DofParams) -> BodyForceMoment:
    return BodyForceMoment(
        fx=-950.0 + 250.0 * math.sin(0.45 * time_s),
        fy=260.0 * math.sin(0.32 * time_s),
        fz=params.mass * params.gravity + 1450.0 * math.sin(0.24 * time_s),
        mx=420.0 * math.sin(0.55 * time_s),
        my=620.0 * math.sin(0.27 * time_s),
        mz=520.0 * math.cos(0.38 * time_s),
    )


def trace_row(
    time_s: float,
    state: RigidBody6DofState,
    command: BodyForceMoment,
    params: RigidBody6DofParams,
) -> dict[str, float]:
    command = limit_force_moment(command, params)
    force_enu, acceleration = force_acceleration_enu(state, command, params)
    quaternion_norm_error, dcm_orthogonality_error = kinematics_health(state)
    return {
        "time": time_s,
        "east": state.east,
        "north": state.north,
        "altitude": state.altitude,
        "velocity_east": state.velocity_east,
        "velocity_north": state.velocity_north,
        "velocity_up": state.velocity_up,
        "speed": speed(state),
        "quat_w": state.quat_w,
        "quat_x": state.quat_x,
        "quat_y": state.quat_y,
        "quat_z": state.quat_z,
        "quaternion_norm_error": quaternion_norm_error,
        "dcm_orthogonality_error": dcm_orthogonality_error,
        "p": state.p,
        "q": state.q,
        "r": state.r,
        "body_rate_norm": body_rate_norm(state),
        "body_force_x": command.fx,
        "body_force_y": command.fy,
        "body_force_z": command.fz,
        "mx": command.mx,
        "my": command.my,
        "mz": command.mz,
        "force_east": force_enu[0],
        "force_north": force_enu[1],
        "force_up": force_enu[2],
        "accel_east": acceleration[0],
        "accel_north": acceleration[1],
        "accel_up": acceleration[2],
    }


def summarize(
    trace: list[dict[str, float]],
    initial_state: RigidBody6DofState,
) -> dict[str, float | bool]:
    speed_values = [row["speed"] for row in trace]
    altitude_values = [row["altitude"] for row in trace]
    body_rate_values = [row["body_rate_norm"] for row in trace]
    quaternion_errors = [row["quaternion_norm_error"] for row in trace]
    dcm_errors = [row["dcm_orthogonality_error"] for row in trace]
    accel_norms = [
        math.sqrt(row["accel_east"] ** 2 + row["accel_north"] ** 2 + row["accel_up"] ** 2)
        for row in trace
    ]
    finite_pass = all(math.isfinite(value) for row in trace for value in row.values())
    altitude_pass = min(altitude_values) >= 20_000.0
    kinematics_pass = max(quaternion_errors) <= 1e-9 and max(dcm_errors) <= 1e-9
    rate_pass = max(body_rate_values) <= 0.35
    return {
        "duration_s": trace[-1]["time"],
        "east_final_m": trace[-1]["east"],
        "north_final_m": trace[-1]["north"],
        "altitude_final_m": trace[-1]["altitude"],
        "altitude_change_m": trace[-1]["altitude"] - initial_state.altitude,
        "speed_change_m_s": trace[-1]["speed"] - speed(initial_state),
        "altitude_min_m": min(altitude_values),
        "body_rate_max_rad_s": max(body_rate_values),
        "accel_rms_m_s2": rms(accel_norms),
        "quaternion_norm_error_max": max(quaternion_errors),
        "dcm_orthogonality_error_max": max(dcm_errors),
        "finite_pass": finite_pass,
        "altitude_pass": altitude_pass,
        "kinematics_pass": kinematics_pass,
        "rate_pass": rate_pass,
        "pass": finite_pass and altitude_pass and kinematics_pass and rate_pass,
    }


def run(
    duration: float = 12.0,
    dt: float = 0.05,
    params: RigidBody6DofParams | None = None,
    initial_state: RigidBody6DofState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or RigidBody6DofParams()
    state = initial_state or make_level_initial_state()
    first_state = RigidBody6DofState(**state.__dict__)
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        command = command_profile(time_s, params)
        trace.append(trace_row(time_s, state, command, params))
        state = euler_step(state, command, dt, params)
        if not state_is_finite(state):
            break

    return trace, summarize(trace, first_state)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
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
