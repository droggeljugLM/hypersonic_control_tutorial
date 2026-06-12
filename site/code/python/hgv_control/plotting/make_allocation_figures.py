"""Generate SVG figures for the teaching control-allocation example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_allocation_figures(
    trace_csv: Path,
    output_dir: Path,
    deflection_limit_rad: float = 0.3141592653589793,
) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("allocation trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    time = column(rows, "time")
    outputs = [
        output_dir / "allocation_moments.svg",
        output_dir / "allocation_actuators.svg",
        output_dir / "allocation_residual.svg",
    ]
    line_plot(
        outputs[0],
        "Control Allocation Moments",
        "time (s)",
        "moment (N m)",
        [
            ("Mx cmd", time, column(rows, "mx_cmd"), "#222222"),
            ("Mx achieved", time, column(rows, "mx_achieved"), "#2f6fbb"),
            ("My cmd", time, column(rows, "my_cmd"), "#555555"),
            ("My achieved", time, column(rows, "my_achieved"), "#258a45"),
            ("Mz cmd", time, column(rows, "mz_cmd"), "#888888"),
            ("Mz achieved", time, column(rows, "mz_achieved"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[1],
        "Control Allocation Actuators",
        "time (s)",
        "deflection (rad)",
        [
            ("left elevon", time, column(rows, "left_elevon"), "#2f6fbb"),
            ("right elevon", time, column(rows, "right_elevon"), "#258a45"),
            ("rudder", time, column(rows, "rudder"), "#8a4fbf"),
            ("body flap", time, column(rows, "body_flap"), "#c47f1f"),
        ],
        hlines=[("deflection limit", deflection_limit_rad, "#c23b3b")],
    )
    line_plot(
        outputs[2],
        "Control Allocation Residual",
        "time (s)",
        "residual norm (N m)",
        [("residual", time, column(rows, "residual_norm"), "#c23b3b")],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--deflection-limit-rad", type=float, default=0.3141592653589793)
    args = parser.parse_args()

    for output in generate_allocation_figures(args.trace_csv, args.output_dir, args.deflection_limit_rad):
        print(output)


if __name__ == "__main__":
    main()
