"""Monte Carlo batch runner for the teaching example.

The perturbations are deliberately simple and transparent. They are intended
for control-education comparisons, not for certification-grade uncertainty
quantification.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import math
from pathlib import Path
import random
from statistics import mean

from hgv_control.models.longitudinal import LongitudinalParams, State
from hgv_control.simulation.run_case import make_initial_state, run


def _scale(rng: random.Random, spread: float) -> float:
    return 1.0 + rng.uniform(-spread, spread)


def sample_case(seed: int) -> tuple[LongitudinalParams, State]:
    rng = random.Random(seed)
    nominal = LongitudinalParams()
    params = replace(
        nominal,
        rho0=nominal.rho0 * _scale(rng, 0.08),
        thrust_max=nominal.thrust_max * _scale(rng, 0.10),
        cl_alpha=nominal.cl_alpha * _scale(rng, 0.12),
        cd_alpha2=nominal.cd_alpha2 * _scale(rng, 0.12),
        cm_alpha=nominal.cm_alpha * _scale(rng, 0.10),
        cm_delta=nominal.cm_delta * _scale(rng, 0.10),
        actuator_wn=nominal.actuator_wn * _scale(rng, 0.15),
    )
    state = make_initial_state(
        velocity=1_750.0 + rng.uniform(-45.0, 45.0),
        altitude=30_000.0 + rng.uniform(-350.0, 350.0),
        gamma=math.radians(-0.5 + rng.uniform(-0.15, 0.15)),
        alpha=math.radians(2.0 + rng.uniform(-0.25, 0.25)),
        theta=math.radians(1.5 + rng.uniform(-0.20, 0.20)),
    )
    return params, state


def run_monte_carlo(
    samples: int,
    controllers: list[str],
    duration: float = 45.0,
    dt: float = 0.02,
    seed_offset: int = 1,
) -> list[dict[str, float | int | str | bool]]:
    rows: list[dict[str, float | int | str | bool]] = []
    for index in range(samples):
        seed = seed_offset + index
        params, initial_state = sample_case(seed)
        for controller in controllers:
            _, metrics = run(controller, duration=duration, dt=dt, params=params, initial_state=initial_state)
            row: dict[str, float | int | str | bool] = {
                "case_id": index,
                "seed": seed,
                "controller": controller,
                "rho0": params.rho0,
                "thrust_max": params.thrust_max,
                "cl_alpha": params.cl_alpha,
                "cd_alpha2": params.cd_alpha2,
                "actuator_wn": params.actuator_wn,
            }
            row.update(metrics)
            rows.append(row)
    return rows


def write_metrics_csv(rows: list[dict[str, float | int | str | bool]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_rows(rows: list[dict[str, float | int | str | bool]]) -> list[dict[str, float | str]]:
    controllers = sorted({str(row["controller"]) for row in rows})
    summary: list[dict[str, float | str]] = []
    for controller in controllers:
        subset = [row for row in rows if row["controller"] == controller]
        pass_rate = mean(1.0 if row["pass"] else 0.0 for row in subset) if subset else 0.0
        summary.append(
            {
                "controller": controller,
                "samples": float(len(subset)),
                "pass_rate": pass_rate,
                "mean_h_rms": mean(float(row["h_rms"]) for row in subset) if subset else 0.0,
                "mean_v_rms": mean(float(row["v_rms"]) for row in subset) if subset else 0.0,
                "mean_q_violation_integral": mean(float(row["q_violation_integral"]) for row in subset) if subset else 0.0,
                "worst_q_max": max((float(row["q_max"]) for row in subset), default=0.0),
            }
        )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument(
        "--controllers",
        nargs="+",
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
        default=["baseline", "guarded"],
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rows = run_monte_carlo(args.samples, args.controllers)
    if args.output is not None:
        write_metrics_csv(rows, args.output)

    for row in summarize_rows(rows):
        print(
            f"{row['controller']}: samples={int(row['samples'])}, "
            f"pass_rate={row['pass_rate']:.3f}, "
            f"mean_h_rms={row['mean_h_rms']:.3f}, "
            f"mean_v_rms={row['mean_v_rms']:.3f}, "
            f"mean_q_violation_integral={row['mean_q_violation_integral']:.3f}, "
            f"worst_q_max={row['worst_q_max']:.3f}"
        )


if __name__ == "__main__":
    main()
