"""Generate SVG figures for the standalone six-DOF rigid-body demo."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_six_dof_rigid_body_figures(trace_csv: Path, output_dir: Path) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("six-DOF rigid-body trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    time = column(rows, "time")
    outputs = [
        output_dir / "sixdof_rigidbody_position_speed.svg",
        output_dir / "sixdof_rigidbody_body_rates.svg",
        output_dir / "sixdof_rigidbody_kinematics_health.svg",
    ]
    line_plot(
        outputs[0],
        "Teaching Six-DOF Rigid Body Position and Speed",
        "time (s)",
        "value",
        [
            ("east / 1000", time, [value / 1000.0 for value in column(rows, "east")], "#2f6fbb"),
            ("north / 1000", time, [value / 1000.0 for value in column(rows, "north")], "#258a45"),
            ("altitude / 1000", time, [value / 1000.0 for value in column(rows, "altitude")], "#8a4fbf"),
            ("speed", time, column(rows, "speed"), "#222222"),
        ],
    )
    line_plot(
        outputs[1],
        "Teaching Six-DOF Rigid Body Rates",
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
        "Teaching Six-DOF Kinematics Health",
        "time (s)",
        "error",
        [
            ("quaternion norm error", time, column(rows, "quaternion_norm_error"), "#2f6fbb"),
            ("DCM orthogonality error", time, column(rows, "dcm_orthogonality_error"), "#c23b3b"),
        ],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    for output in generate_six_dof_rigid_body_figures(args.trace_csv, args.output_dir):
        print(output)


if __name__ == "__main__":
    main()
