"""Run a minimal longitudinal tracking case.

Example:
    python -m hgv_control.simulation.run_case --controller guarded
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.controllers.backstepping import BacksteppingController
from hgv_control.controllers.barrier import BarrierController
from hgv_control.controllers.lqr import LqrController
from hgv_control.controllers.ndi import NdiController
from hgv_control.controllers.pid import BaselineController
from hgv_control.controllers.safety_filter import guarded_reference
from hgv_control.controllers.sliding_mode import SlidingModeController
from hgv_control.metrics.summary import summarize
from hgv_control.models.longitudinal import LongitudinalParams, State, dynamic_pressure, rk4_step


def reference(time_s: float) -> tuple[float, float]:
    altitude_ref = 30_000.0 - 900.0 * min(1.0, time_s / 35.0)
    velocity_ref = 1_750.0 + 80.0 * min(1.0, time_s / 25.0)
    return altitude_ref, velocity_ref


def make_initial_state(
    velocity: float = 1_750.0,
    altitude: float = 30_000.0,
    gamma: float = math.radians(-0.5),
    alpha: float = math.radians(2.0),
    theta: float = math.radians(1.5),
) -> State:
    return State(
        velocity=velocity,
        altitude=altitude,
        gamma=gamma,
        alpha=alpha,
        pitch_rate=0.0,
        theta=theta,
        elevator=0.0,
        elevator_rate=0.0,
    )


def run(
    controller_name: str,
    duration: float = 45.0,
    dt: float = 0.02,
    params: LongitudinalParams | None = None,
    initial_state: State | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    params = params or LongitudinalParams()
    if controller_name in {"lqr", "guarded_lqr"}:
        controller = LqrController()
    elif controller_name in {"ndi", "guarded_ndi"}:
        controller = NdiController()
    elif controller_name in {"backstepping", "guarded_backstepping"}:
        controller = BacksteppingController()
    elif controller_name in {"smc", "guarded_smc"}:
        controller = SlidingModeController()
    elif controller_name in {"barrier", "guarded_barrier"}:
        controller = BarrierController()
    else:
        controller = BaselineController()
    state = initial_state or make_initial_state()

    trace: list[dict[str, float]] = []
    steps = int(duration / dt)
    for step in range(steps + 1):
        time_s = step * dt
        altitude_ref, velocity_ref = reference(time_s)
        if controller_name in {
            "guarded",
            "guarded_lqr",
            "guarded_ndi",
            "guarded_backstepping",
            "guarded_smc",
            "guarded_barrier",
        }:
            altitude_cmd, velocity_cmd = guarded_reference(state, altitude_ref, velocity_ref, params)
        else:
            altitude_cmd, velocity_cmd = altitude_ref, velocity_ref

        delta_cmd, throttle_cmd = controller.command(state, altitude_cmd, velocity_cmd, params)
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
                "qbar": qbar,
            }
        )
        state = rk4_step(state, delta_cmd, throttle_cmd, dt, params)

    metrics = summarize(
        trace,
        params.q_limit,
        dt,
        alpha_limit=math.radians(12.0),
        delta_limit=params.delta_limit,
        delta_rate_limit=params.delta_rate_limit,
    )
    return trace, metrics


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--controller",
        choices=[
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
        ],
        default="guarded",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    trace, metrics = run(args.controller)
    if args.output is not None:
        write_csv(trace, args.output)

    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
