"""Generate SVG figures for the teaching attitude inner-loop example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_attitude_figures(
    trace_csv: Path,
    output_dir: Path,
    moment_limit: float = 4_800.0,
) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("attitude trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)

    time = column(rows, "time")
    outputs = [
        output_dir / "attitude_tracking.svg",
        output_dir / "attitude_rates.svg",
        output_dir / "attitude_moments.svg",
    ]
    line_plot(
        outputs[0],
        "Attitude Inner-Loop Tracking",
        "time (s)",
        "angle (rad)",
        [
            ("roll ref", time, column(rows, "phi_ref"), "#222222"),
            ("roll", time, column(rows, "phi"), "#2f6fbb"),
            ("pitch ref", time, column(rows, "theta_ref"), "#555555"),
            ("pitch", time, column(rows, "theta"), "#258a45"),
            ("yaw ref", time, column(rows, "psi_ref"), "#888888"),
            ("yaw", time, column(rows, "psi"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[1],
        "Attitude Inner-Loop Body Rates",
        "time (s)",
        "rate (rad/s)",
        [
            ("p", time, column(rows, "p"), "#2f6fbb"),
            ("q", time, column(rows, "q"), "#258a45"),
            ("r", time, column(rows, "r"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[2],
        "Attitude Inner-Loop Body Moments",
        "time (s)",
        "moment (N m)",
        [
            ("Mx", time, column(rows, "mx"), "#2f6fbb"),
            ("My", time, column(rows, "my"), "#258a45"),
            ("Mz", time, column(rows, "mz"), "#8a4fbf"),
        ],
        hlines=[("moment limit", moment_limit, "#c23b3b")],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--moment-limit", type=float, default=4_800.0)
    args = parser.parse_args()

    for output in generate_attitude_figures(args.trace_csv, args.output_dir, args.moment_limit):
        print(output)


if __name__ == "__main__":
    main()
