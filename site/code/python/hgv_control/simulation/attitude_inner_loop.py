"""Run a teaching three-axis attitude inner-loop scenario."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.metrics.summary import rms
from hgv_control.models.attitude import (
    AttitudeParams,
    AttitudeState,
    MomentCommand,
    attitude_error,
    limit_moment_command,
    rk4_step,
)


def make_initial_state() -> AttitudeState:
    return AttitudeState(
        phi=math.radians(1.0),
        theta=math.radians(-0.8),
        psi=math.radians(0.5),
        p=0.0,
        q=0.0,
        r=0.0,
        mx=0.0,
        my=0.0,
        mz=0.0,
    )


def attitude_reference(time_s: float) -> tuple[float, float, float]:
    roll_ref = math.radians(5.0) if time_s >= 2.0 else 0.0
    if time_s >= 13.0:
        roll_ref = math.radians(-3.0)

    pitch_ref = math.radians(2.0) if time_s >= 4.0 else 0.0
    if time_s >= 16.0:
        pitch_ref = math.radians(0.5)

    yaw_ref = math.radians(8.0) if time_s >= 7.0 else 0.0
    if time_s >= 20.0:
        yaw_ref = math.radians(4.0)
    return roll_ref, pitch_ref, yaw_ref


def pd_moment_command(
    state: AttitudeState,
    refs: tuple[float, float, float],
    params: AttitudeParams,
) -> MomentCommand:
    phi_ref, theta_ref, psi_ref = refs
    e_phi, e_theta, e_psi = attitude_error(state, phi_ref, theta_ref, psi_ref)
    command = MomentCommand(
        mx=7.2e4 * e_phi - 1.25e4 * state.p,
        my=9.5e4 * e_theta - 1.55e4 * state.q,
        mz=9.0e4 * e_psi - 1.65e4 * state.r,
    )
    return limit_moment_command(command, params)


def summarize_attitude(
    trace: list[dict[str, float]],
    params: AttitudeParams,
    dt: float,
) -> dict[str, float | bool]:
    roll_errors = [row["phi"] - row["phi_ref"] for row in trace]
    pitch_errors = [row["theta"] - row["theta_ref"] for row in trace]
    yaw_errors = [row["psi"] - row["psi_ref"] for row in trace]
    rate_values = [abs(value) for row in trace for value in (row["p"], row["q"], row["r"])]
    moment_values = [abs(value) for row in trace for value in (row["mx"], row["my"], row["mz"])]
    moment_rate_values = [abs(value) for row in trace for value in (row["mx_rate"], row["my_rate"], row["mz_rate"])]
    saturated_steps = [
        row
        for row in trace
        if abs(row["mx_cmd"]) >= params.moment_limit
        or abs(row["my_cmd"]) >= params.moment_limit
        or abs(row["mz_cmd"]) >= params.moment_limit
    ]

    roll_rms = rms(roll_errors)
    pitch_rms = rms(pitch_errors)
    yaw_rms = rms(yaw_errors)
    attitude_rms = math.sqrt((roll_rms * roll_rms + pitch_rms * pitch_rms + yaw_rms * yaw_rms) / 3.0)
    rate_max = max(rate_values) if rate_values else 0.0
    moment_max = max(moment_values) if moment_values else 0.0
    moment_rate_max = max(moment_rate_values) if moment_rate_values else 0.0
    saturation_fraction = len(saturated_steps) / len(trace) if trace else 0.0
    tracking_pass = attitude_rms <= math.radians(3.0)
    rate_pass = rate_max <= math.radians(28.0)
    moment_pass = moment_max <= params.moment_limit + 1e-9
    moment_rate_pass = moment_rate_max <= params.moment_rate_limit + 1e-9

    return {
        "roll_rms_rad": roll_rms,
        "pitch_rms_rad": pitch_rms,
        "yaw_rms_rad": yaw_rms,
        "attitude_rms_rad": attitude_rms,
        "rate_max_rad_s": rate_max,
        "moment_max_nm": moment_max,
        "moment_rate_max_nm_s": moment_rate_max,
        "moment_saturation_fraction": saturation_fraction,
        "tracking_pass": tracking_pass,
        "rate_pass": rate_pass,
        "moment_pass": moment_pass,
        "moment_rate_pass": moment_rate_pass,
        "pass": tracking_pass and rate_pass and moment_pass and moment_rate_pass,
    }


def run(
    duration: float = 26.0,
    dt: float = 0.02,
    params: AttitudeParams | None = None,
    initial_state: AttitudeState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or AttitudeParams()
    state = initial_state or make_initial_state()
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)
    previous_moment = (state.mx, state.my, state.mz)

    for step in range(steps + 1):
        time_s = step * dt
        refs = attitude_reference(time_s)
        command = pd_moment_command(state, refs, params)
        mx_rate = (state.mx - previous_moment[0]) / dt if step > 0 else 0.0
        my_rate = (state.my - previous_moment[1]) / dt if step > 0 else 0.0
        mz_rate = (state.mz - previous_moment[2]) / dt if step > 0 else 0.0
        trace.append(
            {
                "time": time_s,
                "phi": state.phi,
                "theta": state.theta,
                "psi": state.psi,
                "phi_ref": refs[0],
                "theta_ref": refs[1],
                "psi_ref": refs[2],
                "p": state.p,
                "q": state.q,
                "r": state.r,
                "mx": state.mx,
                "my": state.my,
                "mz": state.mz,
                "mx_cmd": command.mx,
                "my_cmd": command.my,
                "mz_cmd": command.mz,
                "mx_rate": mx_rate,
                "my_rate": my_rate,
                "mz_rate": mz_rate,
            }
        )
        previous_moment = (state.mx, state.my, state.mz)
        state = rk4_step(state, command, dt, params)

    return trace, summarize_attitude(trace, params, dt)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=26.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    trace, metrics = run(duration=args.duration, dt=args.dt)
    if args.output is not None:
        write_csv(trace, args.output)
    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
