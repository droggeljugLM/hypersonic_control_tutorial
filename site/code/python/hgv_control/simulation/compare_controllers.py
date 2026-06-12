"""Run Monte Carlo and emit paired controller comparison tables."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from hgv_control.metrics.comparison import Row, paired_deltas, summarize_pairs
from hgv_control.simulation.monte_carlo import run_monte_carlo, summarize_rows, write_metrics_csv


def write_rows(rows: list[Row], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=8)
    choices = [
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
    ]
    parser.add_argument("--baseline", choices=choices, default="baseline")
    parser.add_argument("--candidate", choices=choices, default="guarded")
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--pairs-output", type=Path, default=None)
    args = parser.parse_args()

    rows = run_monte_carlo(args.samples, [args.baseline, args.candidate])
    pairs = paired_deltas(rows, baseline=args.baseline, candidate=args.candidate)
    pair_summary = summarize_pairs(pairs)

    if args.metrics_output is not None:
        write_metrics_csv(rows, args.metrics_output)
    if args.pairs_output is not None:
        write_rows(pairs, args.pairs_output)

    for row in summarize_rows(rows):
        print(
            f"{row['controller']}: samples={int(row['samples'])}, "
            f"pass_rate={row['pass_rate']:.3f}, "
            f"mean_q_violation_integral={row['mean_q_violation_integral']:.3f}, "
            f"worst_q_max={row['worst_q_max']:.3f}"
        )

    print(
        f"paired {args.candidate} - {args.baseline}: "
        f"pairs={pair_summary['pairs']}, "
        f"safety_improved_rate={pair_summary['safety_improved_rate']:.3f}, "
        f"mean_delta_q_violation_integral={pair_summary['mean_delta_q_violation_integral']:.3f}, "
        f"mean_delta_q_max={pair_summary['mean_delta_q_max']:.3f}, "
        f"mean_delta_h_rms={pair_summary['mean_delta_h_rms']:.3f}, "
        f"mean_delta_v_rms={pair_summary['mean_delta_v_rms']:.3f}"
    )


if __name__ == "__main__":
    main()
