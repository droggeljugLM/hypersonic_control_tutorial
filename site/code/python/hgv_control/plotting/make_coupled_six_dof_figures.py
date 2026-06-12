"""Generate SVG figures for the coupled six-DOF teaching skeleton."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def generate_coupled_figures(trace_csv: Path, output_dir: Path) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("coupled six-DOF trace is empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    time = column(rows, "time")
    outputs = [
        output_dir / "coupled6dof_ground_track.svg",
        output_dir / "coupled6dof_guidance_rates.svg",
        output_dir / "coupled6dof_attitude_refs.svg",
        output_dir / "coupled6dof_allocation_residual.svg",
        output_dir / "coupled6dof_aero_angles.svg",
        output_dir / "coupled6dof_aero_moments.svg",
        output_dir / "coupled6dof_force_projection.svg",
        output_dir / "coupled6dof_kinematics_health.svg",
        output_dir / "coupled6dof_force_transform.svg",
    ]
    line_plot(
        outputs[0],
        "Coupled Skeleton Ground Track",
        "east (m)",
        "north (m)",
        [("vehicle", column(rows, "east"), column(rows, "north"), "#2f6fbb")],
    )
    line_plot(
        outputs[1],
        "Guidance Rates: Commanded vs Achieved",
        "time (s)",
        "rate (rad/s)",
        [
            ("gamma cmd", time, column(rows, "gamma_rate_cmd"), "#222222"),
            ("gamma achieved", time, column(rows, "gamma_rate_achieved"), "#2f6fbb"),
            ("heading cmd", time, column(rows, "heading_rate_cmd"), "#555555"),
            ("heading achieved", time, column(rows, "heading_rate_achieved"), "#8a4fbf"),
        ],
    )
    line_plot(
        outputs[2],
        "Coupled Skeleton Attitude References",
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
        outputs[3],
        "Coupled Skeleton Allocation Residual",
        "time (s)",
        "residual norm (N m)",
        [("residual", time, column(rows, "allocation_residual_norm"), "#c23b3b")],
    )
    line_plot(
        outputs[4],
        "Coupled Skeleton Aero Angles",
        "time (s)",
        "angle (rad)",
        [
            ("alpha", time, column(rows, "alpha_rad"), "#2f6fbb"),
            ("beta", time, column(rows, "beta_rad"), "#258a45"),
        ],
    )
    line_plot(
        outputs[5],
        "Coupled Skeleton Aero Moments",
        "time (s)",
        "moment (N m)",
        [
            ("Mx cmd", time, column(rows, "mx_cmd"), "#222222"),
            ("Mx aero", time, column(rows, "mx_aero"), "#2f6fbb"),
            ("My cmd", time, column(rows, "my_cmd"), "#555555"),
            ("My aero", time, column(rows, "my_aero"), "#8a4fbf"),
            ("Mz cmd", time, column(rows, "mz_cmd"), "#888888"),
            ("Mz aero", time, column(rows, "mz_aero"), "#258a45"),
        ],
    )
    line_plot(
        outputs[6],
        "Coupled Skeleton Force Projection",
        "time (s)",
        "force (N)",
        [
            ("tangent", time, column(rows, "force_tangent"), "#222222"),
            ("normal", time, column(rows, "force_normal"), "#2f6fbb"),
            ("lateral", time, column(rows, "force_lateral"), "#258a45"),
        ],
    )
    line_plot(
        outputs[7],
        "Coupled Skeleton Kinematics Health",
        "time (s)",
        "error",
        [
            ("quaternion norm error", time, column(rows, "quaternion_norm_error"), "#2f6fbb"),
            ("DCM orthogonality error", time, column(rows, "dcm_orthogonality_error"), "#c23b3b"),
        ],
    )
    line_plot(
        outputs[8],
        "Coupled Skeleton DCM Force Transform Check",
        "time (s)",
        "force (N)",
        [
            ("direct east", time, column(rows, "force_east"), "#222222"),
            ("DCM east", time, column(rows, "dcm_force_east"), "#2f6fbb"),
            ("direct up", time, column(rows, "force_up"), "#555555"),
            ("DCM up", time, column(rows, "dcm_force_up"), "#258a45"),
            ("delta norm", time, column(rows, "dcm_force_delta_norm"), "#c23b3b"),
        ],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    for output in generate_coupled_figures(args.trace_csv, args.output_dir):
        print(output)


if __name__ == "__main__":
    main()
