"""Generate SVG figures for the teaching 3-D guidance example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def _target_marker(center: float, width: float = 900.0) -> list[float]:
    return [center - 0.5 * width, center + 0.5 * width]


def generate_guidance_figures(
    trace_csv: Path,
    output_dir: Path,
    q_limit: float = 70_000.0,
    load_factor_limit: float = 4.0,
) -> list[Path]:
    rows = read_numeric_csv(trace_csv)
    if not rows:
        raise ValueError("guidance trace is empty")

    target_east = float(rows[-1]["target_east"])
    target_north = float(rows[-1]["target_north"])
    target_altitude = float(rows[-1]["target_altitude"])
    output_dir.mkdir(parents=True, exist_ok=True)

    outputs = [
        output_dir / "guidance3d_ground_track.svg",
        output_dir / "guidance3d_altitude_profile.svg",
        output_dir / "guidance3d_dynamic_pressure.svg",
        output_dir / "guidance3d_load_factor.svg",
    ]

    line_plot(
        outputs[0],
        "3-D Guidance Ground Track",
        "east (m)",
        "north (m)",
        [
            ("vehicle", column(rows, "east"), column(rows, "north"), "#2f6fbb"),
            ("target", _target_marker(target_east), [target_north, target_north], "#c23b3b"),
        ],
    )
    line_plot(
        outputs[1],
        "3-D Guidance Altitude Profile",
        "time (s)",
        "altitude (m)",
        [("vehicle", column(rows, "time"), column(rows, "altitude"), "#258a45")],
        hlines=[("target altitude", target_altitude, "#c23b3b")],
    )
    line_plot(
        outputs[2],
        "3-D Guidance Dynamic Pressure",
        "time (s)",
        "qbar (Pa)",
        [("qbar", column(rows, "time"), column(rows, "qbar"), "#8a4fbf")],
        hlines=[("q limit", q_limit, "#c23b3b")],
    )
    line_plot(
        outputs[3],
        "3-D Guidance Load Factor",
        "time (s)",
        "load factor",
        [("load factor", column(rows, "time"), column(rows, "load_factor"), "#c47f1f")],
        hlines=[("load limit", load_factor_limit, "#c23b3b")],
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--q-limit", type=float, default=70_000.0)
    parser.add_argument("--load-factor-limit", type=float, default=4.0)
    args = parser.parse_args()

    for output in generate_guidance_figures(args.trace_csv, args.output_dir, args.q_limit, args.load_factor_limit):
        print(output)


if __name__ == "__main__":
    main()
