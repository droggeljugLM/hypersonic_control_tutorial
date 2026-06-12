"""Generate SVG figures for the teaching aerodynamic-table example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_aero_table_figures(trace_csv: Path, output_dir: Path) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("aero table trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    time = column(rows, "time")
    outputs = [
        output_dir / "aero_table_coefficients.svg",
        output_dir / "aero_table_forces.svg",
        output_dir / "aero_table_moments.svg",
    ]
    line_plot(
        outputs[0],
        "Teaching Aero Table Coefficients",
        "time (s)",
        "coefficient",
        [
            ("CL", time, column(rows, "clift"), "#2f6fbb"),
            ("CD", time, column(rows, "cdrag"), "#c23b3b"),
            ("CY", time, column(rows, "cy"), "#258a45"),
            ("Cm", time, column(rows, "cm_pitch"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[1],
        "Teaching Aero Table Forces",
        "time (s)",
        "force (N)",
        [
            ("lift", time, column(rows, "lift"), "#2f6fbb"),
            ("drag", time, column(rows, "drag"), "#c23b3b"),
            ("side force", time, column(rows, "side_force"), "#258a45"),
        ],
    )
    line_plot(
        outputs[2],
        "Teaching Aero Table Moments",
        "time (s)",
        "moment (N m)",
        [
            ("Mx", time, column(rows, "mx"), "#2f6fbb"),
            ("My", time, column(rows, "my"), "#8a4fbf"),
            ("Mz", time, column(rows, "mz"), "#258a45"),
        ],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    for output in generate_aero_table_figures(args.trace_csv, args.output_dir):
        print(output)


if __name__ == "__main__":
    main()
