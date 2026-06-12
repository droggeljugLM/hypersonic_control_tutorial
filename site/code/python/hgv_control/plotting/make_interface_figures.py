"""Generate SVG figures for the teaching guidance-attitude interface example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_interface_figures(trace_csv: Path, output_dir: Path) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("interface trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    time = column(rows, "time")
    outputs = [
        output_dir / "interface_guidance_errors.svg",
        output_dir / "interface_attitude_refs.svg",
        output_dir / "interface_allocation_residual.svg",
    ]
    line_plot(
        outputs[0],
        "Guidance Interface Errors",
        "time (s)",
        "error (rad)",
        [
            ("gamma error", time, column(rows, "gamma_error"), "#2f6fbb"),
            ("heading error", time, column(rows, "heading_error"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[1],
        "Guidance-to-Attitude References",
        "time (s)",
        "angle (rad)",
        [
            ("roll ref", time, column(rows, "roll_ref"), "#222222"),
            ("roll", time, column(rows, "roll"), "#2f6fbb"),
            ("pitch ref", time, column(rows, "pitch_ref"), "#555555"),
            ("pitch", time, column(rows, "pitch"), "#258a45"),
            ("yaw ref", time, column(rows, "yaw_ref"), "#888888"),
            ("yaw", time, column(rows, "yaw"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[2],
        "Interface Allocation Residual",
        "time (s)",
        "residual norm (N m)",
        [("residual", time, column(rows, "allocation_residual_norm"), "#c23b3b")],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    for output in generate_interface_figures(args.trace_csv, args.output_dir):
        print(output)


if __name__ == "__main__":
    main()
