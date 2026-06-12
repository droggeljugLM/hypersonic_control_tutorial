"""Run a teaching control-allocation scenario."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.metrics.summary import rms
from hgv_control.models.control_allocation import (
    AllocationParams,
    AllocatorState,
    MomentDemand,
    allocate_moment,
    state_to_tuple,
)


def desired_moment(time_s: float) -> MomentDemand:
    return MomentDemand(
        mx=2_600.0 * math.sin(0.34 * time_s),
        my=1_900.0 * math.sin(0.27 * time_s + 0.45),
        mz=2_400.0 * math.sin(0.23 * time_s - 0.25),
    )


def summarize_allocation(
    trace: list[dict[str, float]],
    params: AllocationParams,
) -> dict[str, float | bool]:
    residual_values = [row["residual_norm"] for row in trace]
    deflection_values = [
        abs(value)
        for row in trace
        for value in (row["left_elevon"], row["right_elevon"], row["rudder"], row["body_flap"])
    ]
    rate_values = [
        abs(value)
        for row in trace
        for value in (row["left_elevon_rate"], row["right_elevon_rate"], row["rudder_rate"], row["body_flap_rate"])
    ]
    saturated_count = sum(1 for row in trace if row["saturated"] > 0.5)
    rate_limited_count = sum(1 for row in trace if row["rate_limited"] > 0.5)
    residual_rms = rms(residual_values)
    residual_max = max(residual_values) if residual_values else 0.0
    deflection_max = max(deflection_values) if deflection_values else 0.0
    rate_max = max(rate_values) if rate_values else 0.0
    saturation_fraction = saturated_count / len(trace) if trace else 0.0
    rate_limited_fraction = rate_limited_count / len(trace) if trace else 0.0
    residual_pass = residual_rms <= 650.0 and residual_max <= 2_400.0
    deflection_pass = deflection_max <= params.deflection_limit + 1e-9
    rate_pass = rate_max <= params.deflection_rate_limit + 1e-9
    return {
        "allocation_residual_rms_nm": residual_rms,
        "allocation_residual_max_nm": residual_max,
        "deflection_max_rad": deflection_max,
        "deflection_rate_max_rad_s": rate_max,
        "saturation_fraction": saturation_fraction,
        "rate_limited_fraction": rate_limited_fraction,
        "residual_pass": residual_pass,
        "deflection_pass": deflection_pass,
        "rate_pass": rate_pass,
        "pass": residual_pass and deflection_pass and rate_pass,
    }


def run(
    duration: float = 24.0,
    dt: float = 0.02,
    params: AllocationParams | None = None,
    initial_state: AllocatorState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or AllocationParams()
    state = initial_state or AllocatorState()
    previous_values = state_to_tuple(state)
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        demand = desired_moment(time_s)
        result = allocate_moment(demand, state, dt, params)
        actual_values = state_to_tuple(result.actual)
        rates = tuple((actual_values[i] - previous_values[i]) / dt if step > 0 else 0.0 for i in range(4))
        trace.append(
            {
                "time": time_s,
                "mx_cmd": demand.mx,
                "my_cmd": demand.my,
                "mz_cmd": demand.mz,
                "mx_achieved": result.achieved.mx,
                "my_achieved": result.achieved.my,
                "mz_achieved": result.achieved.mz,
                "mx_residual": result.residual.mx,
                "my_residual": result.residual.my,
                "mz_residual": result.residual.mz,
                "residual_norm": result.residual_norm,
                "left_elevon": result.actual.left_elevon,
                "right_elevon": result.actual.right_elevon,
                "rudder": result.actual.rudder,
                "body_flap": result.actual.body_flap,
                "left_elevon_target": result.target.left_elevon,
                "right_elevon_target": result.target.right_elevon,
                "rudder_target": result.target.rudder,
                "body_flap_target": result.target.body_flap,
                "left_elevon_rate": rates[0],
                "right_elevon_rate": rates[1],
                "rudder_rate": rates[2],
                "body_flap_rate": rates[3],
                "saturated": 1.0 if result.saturated else 0.0,
                "rate_limited": 1.0 if result.rate_limited else 0.0,
            }
        )
        previous_values = actual_values
        state = result.actual
    return trace, summarize_allocation(trace, params)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--rudder-efficiency", type=float, default=1.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    params = AllocationParams(efficiency=(1.0, 1.0, args.rudder_efficiency, 1.0))
    trace, metrics = run(duration=args.duration, dt=args.dt, params=params)
    if args.output is not None:
        write_csv(trace, args.output)
    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
